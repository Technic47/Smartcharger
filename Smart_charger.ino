// Bench power supply firmware v0.8.0  (based on v0.7.7)
// =====================================================================
// v0.8.0: MANUAL VOLTAGE TABLE CALIBRATION.
//         Added a new manual calibration mode to replace the auto-sweep,
//         which was unreliable due to hardware timeouts. On the CAL screen:
//         - Short press: toggle between AUTO and MANUAL calibration modes.
//         - Long press: start the selected calibration.
//         In MANUAL mode, the MCU prompts a target voltage, the user
//         adjusts the PWM duty with the encoder until the multimeter reads
//         the target, then short-presses to accept and move to the next point.
//         Long press aborts the manual calibration at any time.
// =====================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define FW_VERSION "PSU 0.8.0"

// ---------------- pins ----------------
#define PIN_CV_PWM    5           // Timer3 OC3A (SERVOS 2.1)
#define PIN_CC_PWM    44          // Timer5 OC5C (AUX-2)
#define PIN_SWITCH    40          // AUX-2
#define PIN_VBAT      A5          // D59, AUX-2
#define PIN_ISENSE    A10         // D64, AUX-2
#define PIN_NTC_BAT   A13         // NTC connector E0
#define PIN_NTC_HEAT  A15         // NTC connector E1
#define PIN_NTC_INT   A14         // NTC connector BED
#define PIN_FAN       10          // screw terminal E0
#define PIN_ENC_A     31
#define PIN_ENC_B     33
#define PIN_ENC_BTN   35
#define PIN_BEEP      37          // piezo on the display, EXP1
#define BEEP_ENABLE   1
#define BEEP_FREQ     2500

// ---------------- hardware constants ----------------
#define VREF            5.0f       // ADC reference, absorbed by calibration
#define R_SERIES        0.014f     // switch + wires in negative lead, ohm
#define I_SIGN_DEFAULT  1.0f       // default sensor direction
#define ENC_PULSES      4          // transitions per detent
#define ENC_DIR         1          // set -1 to reverse rotation

#define NTC_PULLUP      4700.0f    // pull-up on the board
#define NTC_R25         100000.0f
#define NTC_BETA        3950.0f
#define CV_R_FB         4270.0f    // upper leg of the FB divider

// ---------------- limits ----------------
#define V_FAULT         25.0f      // hard overvoltage trip (module ceiling is ~15 V)
#define I_FAULT         12.0f
#define T_BAT_MAX       50.0f      // only if the 'battery' NTC is connected
#define T_HEAT_MAX      80.0f
#define T_INT_MAX       75.0f
#define V_BATT_FAULT    15.3f

#define I_MIN_SET       0.10f      // lowest current limit you can set
#define READY_BAND      0.3f       // ON allowed when Vout <= Vset + this
#define TRIM_WHEN_OFF   1          // 1 = output switch installed: trim runs when OFF too
#define OUT_TAU_MS      9000.0f    // output discharge time constant
#define TRIM_MAX        1000.0f    // trim authority in PWM counts
#define VSET_GUARD      0.15f      // CV demand never exceeds vSet + this
#define CC_CV_MARGIN    0.20f      // in CC the CV demand tracks vbat + this

// ---------------- digital CC loop ----------------
#define CC_PI           1          // 1 = digital current PI
#define CC_PI_KP        150.0f     // proportional term
#define CC_PI_KI        300.0f     // integral term
#define CC_PI_DUTY_ON   40         // ccDuty above this = current limit active
#define CC_PI_DUTY_OFF  15         // ccDuty below this = current limit released
#define TRIM_LIMIT_ERR  0.4f       // loaded and Vout far below Vset = current limit
#define V_STEP          0.05f      // volts per encoder click
#define I_STEP          0.05f      // amps per encoder click

#define FAN_ENABLE      1
#define FAN_ON_C        45.0f
#define FAN_OFF_C       40.0f
#define FAN_ON_AMPS     2.0f

// ---------------- config in EEPROM ----------------
struct Cfg {
  uint16_t magic;
  float kOpen;
  float kClosed;
  float iGain;
  float vCeil;
  float iCeil;
  float i0;
  float iMax;
  int8_t  iSign;
  float ccKp;
  float ccKi;
  float vSlew;
  float vDown;
  float vUp;
};
#define CFG_MAGIC 0xC47B
static Cfg cfg;

static void cfgDefaults() {
  cfg.magic   = CFG_MAGIC;
  cfg.kOpen   = 8.0f;
  cfg.kClosed = 142.0f / 22.0f;
  cfg.iGain   = 50.0f / 1023.0f;
  cfg.vCeil   = 14.9f;
  cfg.iCeil   = 8.5f;
  cfg.i0      = 0.30f;
  cfg.iMax    = 8.0f;
  cfg.iSign   = (int8_t)I_SIGN_DEFAULT;
  cfg.ccKp    = CC_PI_KP;
  cfg.ccKi    = CC_PI_KI;
  cfg.vSlew   = 2.0f;
  cfg.vDown   = 8.0f;
  cfg.vUp     = 20.0f;
}
static void cfgLoad() {
  Cfg t;
  EEPROM.get(0, t);
  if (t.magic == CFG_MAGIC) cfg = t; else cfgDefaults();
  if (!(cfg.i0 >= 0.0f && cfg.i0 < 2.0f)) cfg.i0 = 0.30f;
  if (!(cfg.iCeil > cfg.i0 + 0.5f && cfg.iCeil < 30.0f)) cfg.iCeil = 8.5f;
  if (!(cfg.iMax >= 0.5f && cfg.iMax <= 30.0f)) cfg.iMax = 8.0f;
  if (cfg.iSign != 1 && cfg.iSign != -1) cfg.iSign = (int8_t)I_SIGN_DEFAULT;
  if (!(cfg.ccKp >= 10.0f && cfg.ccKp <= 1000.0f)) cfg.ccKp = CC_PI_KP;
  if (!(cfg.ccKi >= 10.0f && cfg.ccKi <= 5000.0f)) cfg.ccKi = CC_PI_KI;
  if (!(cfg.vSlew >= 0.0f && cfg.vSlew <= 20.0f))  cfg.vSlew = 2.0f;
  if (!(cfg.vDown >= 1.0f && cfg.vDown <= 50.0f))  cfg.vDown = 8.0f;
  if (!(cfg.vUp   >= 1.0f && cfg.vUp   <= 50.0f))  cfg.vUp   = 20.0f;
}
static void cfgSave() { EEPROM.put(0, cfg); }
static float iLimit() { return cfg.iMax < cfg.iCeil ? cfg.iMax : cfg.iCeil; }

// ---------------- V calibration table ----------------
static uint16_t cvVoltsToDuty(float v);
#define TAB_V0    2.5f
#define TAB_STEP  0.5f
#define TAB_N     26
#define TAB_MAGIC 0x7AB1
#define TAB_ADDR  128
struct TabCfg { uint16_t magic; uint16_t d[TAB_N]; };
static uint16_t tab[TAB_N];
static bool     tabValid = false;
static bool     calRun = false;
static uint8_t  calIdx = 0;
static uint32_t calPtT = 0;
static uint16_t calWinT = 0;
static float    calVSetSaved = 0;
static bool     calWasOff = false, calWasTrim = true;

static void tabLoad() {
  TabCfg t;
  EEPROM.get(TAB_ADDR, t);
  if (t.magic == TAB_MAGIC) { memcpy(tab, t.d, sizeof tab); tabValid = true; }
}
static void tabSave() {
  TabCfg t; t.magic = TAB_MAGIC; memcpy(t.d, tab, sizeof tab);
  EEPROM.put(TAB_ADDR, t);
}
static uint16_t tabDuty(float v) {
  if (!tabValid) return cvVoltsToDuty(v);
  float x = (v - TAB_V0) / TAB_STEP;
  int i = (int)floorf(x);
  if (i < 0) i = 0;
  if (i > TAB_N - 2) i = TAB_N - 2;
  float f = x - (float)i;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  float d = tab[i] + (tab[i + 1] - tab[i]) * f;
  if (d < 0 || d > 1023) return cvVoltsToDuty(v);
  return (uint16_t)(d + 0.5f);
}

struct Sp { uint16_t magic; float v; float i; };
#define SP_MAGIC 0x5B01
#define SP_ADDR  64

// ---------------- display ----------------
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /*clock=*/23, /*data=*/17, /*cs=*/16);

// ---------------- state ----------------
static float    vSet = 5.0f, iSet = 1.0f;
static float    vRamp = 5.0f;
static bool     spDirty = false;
static uint32_t tSpChange = 0;

static uint16_t cvDuty = 1023;
static const char* cvWho = "boot";
static uint16_t    cvWn  = 0;
#define CV_TRACE_N 10
static float   cvT_vr[CV_TRACE_N]; static uint16_t cvT_b[CV_TRACE_N];
static float   cvT_tr[CV_TRACE_N]; static uint16_t cvT_d[CV_TRACE_N];
static uint16_t cvT_min[CV_TRACE_N]; static uint16_t cvT_t20[CV_TRACE_N];
static uint8_t cvT_i = 0;
static bool   drawOn = true;
static uint16_t ccDuty = 1023;
static float    ccInt = 0;
static uint32_t tCcManual = 0;
static float    trim = 0;
static float    trimGood = 0;
static bool     trimEnabled = true;
static float    vLast = 0;

static bool     swState = false;
static bool     fault = false;
static const char* faultMsg = "";
static bool     logOn = false;
static bool     checksOn = true;

static float fVc = 0, fIc = 0;
static bool  fInit = false;
static float iZero = 512.0f;
static float vbat = 0, ibat = 0;
static float vEst = 15.0f;
static float tBat = NAN, tHeat = NAN, tInt = NAN;
static uint32_t tSwOn = 0;
static bool fanOn = false;

static uint8_t uiField = 0;             // 0 = V, 1 = I, 2 = info screen, 3 = CAL
static bool calManualMode = false;      // v0.8.0: true if manual calibration selected

// ---------------- encoder ----------------
static volatile int8_t encDelta = 0;
static volatile bool   encPress = false;
static volatile bool   encLong = false;
static volatile uint16_t encTicks = 0;
static volatile uint16_t encBad = 0;

ISR(TIMER4_COMPB_vect) {
  static const int8_t tbl[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  static uint8_t prev = 0;
  static int8_t acc = 0;
  static uint16_t btnCnt = 0;

  uint8_t cur = (digitalRead(PIN_ENC_A) ? 1 : 0) | (digitalRead(PIN_ENC_B) ? 2 : 0);
  if (cur != prev) {
    int8_t st = tbl[(prev << 2) | cur];
    if (st) encTicks++; else encBad++;
    acc += st;
    prev = cur;
    if (acc >= ENC_PULSES)       { encDelta += ENC_DIR; acc = 0; }
    else if (acc <= -ENC_PULSES) { encDelta -= ENC_DIR; acc = 0; }
  }
  if (digitalRead(PIN_ENC_BTN) == LOW) {
    if (btnCnt < 60000) btnCnt++;
    if (btnCnt == 1000) encLong = true;
  } else {
    if (btnCnt >= 30 && btnCnt < 1000) encPress = true;
    btnCnt = 0;
  }
}

// ---------------- PWM ----------------
static void pwmInit() {
  TCCR3A = _BV(WGM31) | _BV(WGM30);
  TCCR3B = _BV(WGM32) | _BV(CS30);
  OCR3A = 0;
  TCCR5A = _BV(WGM51) | _BV(WGM50);
  TCCR5B = _BV(WGM52) | _BV(CS50);
  OCR5C = 0;
}
static void setCvDuty(uint16_t d) {
  if (d > 1023) d = 1023;
  cvDuty = d; cvWn++;
  if (d == 0) { TCCR3A &= ~_BV(COM3A1); digitalWrite(PIN_CV_PWM, LOW); }
  else        { OCR3A = d; TCCR3A |= _BV(COM3A1); }
}
static void setCcDuty(uint16_t d) {
  if (d > 1023) d = 1023;
  ccDuty = d;
  if (d == 0) { TCCR5A &= ~_BV(COM5C1); digitalWrite(PIN_CC_PWM, LOW); }
  else        { OCR5C = d; TCCR5A |= _BV(COM5C1); }
}

// ---------------- mappings ----------------
#define CV_R_SERIES 780.0f
#define CV_V_LUMP   1.60f
static float cvDutyToVolts(uint16_t d) {
  float vn = 5.0f * d / 1023.0f;
  float inj = (vn - CV_V_LUMP) / CV_R_SERIES;
  if (inj < 0) inj = 0;
  return cfg.vCeil - CV_R_FB * inj;
}
static uint16_t cvVoltsToDuty(float v) {
  float inj = (cfg.vCeil - v) / CV_R_FB;
  if (inj <= 0) return 0;
  float d = (CV_V_LUMP + CV_R_SERIES * inj) / 5.0f;
  if (d > 1.0f) d = 1.0f;
  return (uint16_t)(d * 1023.0f + 0.5f);
}
static float ccDutyToAmps(uint16_t d) {
  return cfg.i0 + (cfg.iCeil - cfg.i0) * d / 1023.0f;
}
static uint16_t ccAmpsToDuty(float a) {
  float d = (a - cfg.i0) / (cfg.iCeil - cfg.i0);
  if (d < 0) d = 0;
  if (d > 1) d = 1;
  return (uint16_t)(d * 1023.0f + 0.5f);
}

// ---------------- beeper ----------------
static uint8_t  beepLeft = 0;
static uint16_t beepOnMs = 0, beepOffMs = 0;
static uint32_t beepT = 0;
static bool     beepOn = false;
static void beep(uint8_t n, uint16_t onMs, uint16_t offMs) {
#if BEEP_ENABLE
  beepLeft = n; beepOnMs = onMs; beepOffMs = offMs;
  beepOn = false;
  beepT = millis() - offMs;
#endif
}
static void beepTask() {
#if BEEP_ENABLE
  uint32_t now = millis();
  if (beepOn) {
    if (now - beepT >= beepOnMs) {
      noTone(PIN_BEEP);
      digitalWrite(PIN_BEEP, LOW);
      beepOn = false;
      beepT = now;
      if (beepLeft) beepLeft--;
    }
  } else if (beepLeft && now - beepT >= beepOffMs) {
    tone(PIN_BEEP, BEEP_FREQ);
    beepOn = true;
    beepT = now;
  }
#endif
}

// ---------------- setpoints -> PWM ----------------
static void applyCv() {
  if (fault) return;
  uint16_t b = tabDuty(vRamp);                       
  if (b == 0) { cvWho = "applyCv"; setCvDuty(0); return; }   
  float d = (float)b + trim;
  uint16_t dMin = tabDuty(vRamp + VSET_GUARD);
  if (dMin > 1023) dMin = 1023;
  if (d < (float)dMin) d = (float)dMin;
  if (d < 1.0f) d = 1.0f;
  if (d > 1023.0f) d = 1023.0f;
  cvWho = "applyCv";
  cvT_vr[cvT_i] = vRamp; cvT_b[cvT_i] = b; cvT_tr[cvT_i] = trim;
  cvT_d[cvT_i] = (uint16_t)(d + 0.5f);
  cvT_min[cvT_i] = dMin;
  cvT_t20[cvT_i] = tabValid ? tab[20] : 0xFFFF;
  cvT_i = (cvT_i + 1) % CV_TRACE_N;
  setCvDuty((uint16_t)(d + 0.5f));
}
static void applyCc() {
  if (fault) return;
#if CC_PI
  setCcDuty(swState ? ccDuty : 0);
#else
  setCcDuty(swState ? ccAmpsToDuty(iSet) : 0);
#endif
}
static void spTouch() { spDirty = true; tSpChange = millis(); }
static void setV(float v) {
  float lo = TAB_V0, hi = cfg.vCeil;
  if (v < lo) { v = lo; Serial.println(F("note: clamped to the 2.0 V floor")); }
  if (v > hi) { v = hi; Serial.println(F("note: clamped to vCeil")); }
  v = floor(v * 100.0f + 0.5f) / 100.0f;
  if (fabs(v - vSet) < 0.001f) return;
  vSet = v;
  spTouch();
  applyCv();
}
static void setI(float a) {
  if (a < I_MIN_SET) a = I_MIN_SET;
  if (a > iLimit()) a = iLimit();
  a = floor(a * 100.0f + 0.5f) / 100.0f;
  if (fabs(a - iSet) < 0.001f) return;
  iSet = a;
  spTouch();
  applyCc();
}
static void spSaveNow() {
  Sp t; t.magic = SP_MAGIC; t.v = vSet; t.i = iSet;
  EEPROM.put(SP_ADDR, t);
  spDirty = false;
}
static void spLoad() {
  Sp t;
  EEPROM.get(SP_ADDR, t);
  if (t.magic == SP_MAGIC && t.v >= 1.0f && t.v <= 40.0f && t.i >= 0.05f && t.i <= 30.0f) {
    vSet = t.v; iSet = t.i;
  }
  if (vSet > cfg.vCeil) vSet = cfg.vCeil;
  if (iSet > iLimit()) iSet = iLimit();
}

// ---------------- output switch ----------------
static bool outReady() { return (vEst - vSet) <= READY_BAND; }
static uint16_t waitSeconds() {
  if (outReady()) return 0;
  float s = (OUT_TAU_MS / 1000.0f) * log((vEst - vSet) / READY_BAND);
  if (s < 1) s = 1;
  return (uint16_t)(s + 0.5f);
}
static void openSwitch() { digitalWrite(PIN_SWITCH, LOW); swState = false; }
static bool psuOn(bool force) {
  if (swState) return true;
  if (fault) { Serial.println(F("refused: fault active (clr)")); beep(3, 40, 60); return false; }
  if (!force && !outReady()) {
    Serial.print(F("refused: output is still above the set value, wait "));
    Serial.print(waitSeconds()); Serial.println(F(" s (on force to ignore)"));
    beep(3, 40, 60);
    return false;
  }
  applyCv();
  applyCc();
  digitalWrite(PIN_SWITCH, HIGH);
  swState = true;
  applyCc();
  tSwOn = millis();
  ccInt = 0;
  beep(1, 60, 0);
  return true;
}
static void psuOff() {
  if (!swState) return;
  openSwitch();
  trim = trimGood;
  ccInt = 0;
  ccDuty = 0;
  applyCv();
  applyCc();
  beep(2, 60, 80);
}
static void safeOutputs() {
  openSwitch();
  cvWho = "safe";
  setCvDuty(1023);
  setCcDuty(0);
}
static void trip(const char* msg) {
  if (fault) return;
  fault = true;
  faultMsg = msg;
  safeOutputs();
  vEst = cfg.vCeil;
  beep(5, 300, 150);
  Serial.print(F("FAULT: ")); Serial.println(msg);
}
static void clearFault() {
  fault = false;
  faultMsg = "";
  applyCv();
  applyCc();
  Serial.println(F("fault cleared, output stays OFF"));
}

// ---------------- measurements ----------------
static float adcAvg(uint8_t pin, uint8_t n) {
  analogRead(pin);
  uint32_t s = 0;
  for (uint8_t i = 0; i < n; i++) s += analogRead(pin);
  return (float)s / n;
}
static float ntcTemp(float counts) {
  if (counts > 1018.0f || counts < 3.0f) return NAN;
  float r = NTC_PULLUP * counts / (1023.0f - counts);
  return 1.0f / (1.0f / 298.15f + log(r / NTC_R25) / NTC_BETA) - 273.15f;
}
static void measureZero() {
  uint32_t s = 0;
  for (uint16_t i = 0; i < 512; i++) s += analogRead(PIN_ISENSE);
  iZero = (float)s / 512.0f;
}
static void measure() {
  float vc = adcAvg(PIN_VBAT, 32);
  float ic = adcAvg(PIN_ISENSE, 32);
  if (!fInit) { fVc = vc; fIc = ic; fInit = true; }
  else { fVc += 0.3f * (vc - fVc); fIc += 0.3f * (ic - fIc); }

  ibat = (fIc - iZero) * cfg.iGain * cfg.iSign;
  float vnode = fVc * VREF / 1023.0f;
  vbat = vnode * cfg.kClosed - (swState ? ibat * R_SERIES : 0.0f);

  tBat  = ntcTemp(adcAvg(PIN_NTC_BAT, 8));
  tHeat = ntcTemp(adcAvg(PIN_NTC_HEAT, 8));
  tInt  = ntcTemp(adcAvg(PIN_NTC_INT, 8));

  if (!fault) {
    if (checksOn && vbat > V_FAULT) trip("OVERVOLT");
    else if (checksOn && swState && vbat > V_BATT_FAULT) trip("BATT OVERVOLT");
    else if (checksOn && fabs(ibat) > I_FAULT) trip("OVERCURRENT");
    else if (!isnan(tBat)  && tBat  > T_BAT_MAX)  trip("BATTERY HOT");
    else if (!isnan(tHeat) && tHeat > T_HEAT_MAX) trip("HEATSINK HOT");
    else if (!isnan(tInt)  && tInt  > T_INT_MAX)  trip("INSIDE HOT");
  }
}
static void estimateOutput(float dt) {
  if (checksOn) { vEst = vbat; return; }
  float tgt = cvDutyToVolts(cvDuty);
  if (vEst > tgt) vEst = tgt + (vEst - tgt) * exp(-dt * 1000.0f / OUT_TAU_MS);
  else            vEst = tgt;
}
static bool ccLatch = false;
static bool inCurrentLimit() {
  if (!swState || !checksOn) { ccLatch = false; return false; }
#if CC_PI
  if (!ccLatch && ccDuty >= CC_PI_DUTY_ON) ccLatch = true;
  else if (ccLatch && ccDuty <= CC_PI_DUTY_OFF) ccLatch = false;
  return ccLatch;
#else
  if (!ccLatch && ibat > iSet * 0.92f) ccLatch = true;
  else if (ccLatch && ibat < iSet * 0.80f) ccLatch = false;
  return ccLatch;
#endif
}
static void ccPiTask(float dt) {
#if CC_PI
  if (!swState || fault || !checksOn) {
    if (ccInt != 0.0f || ccDuty != 0) { ccInt = 0; ccDuty = 0; setCcDuty(0); }
    return;
  }
  if (millis() < tCcManual) return;
  float e = ibat - iSet;
  bool satHi = (ccDuty >= 1023 && e > 0);
  bool satLo = (ccDuty <= 0 && e < 0);
  if (!satHi && !satLo) ccInt += cfg.ccKi * e * dt;
  if (ccInt < 0) ccInt = 0;
  if (ccInt > 1023) ccInt = 1023;
  float d = cfg.ccKp * e + ccInt;
  if (d < 0) d = 0;
  if (d > 1023) d = 1023;
  ccDuty = (uint16_t)(d + 0.5f);
  setCcDuty(ccDuty);
#endif
}

static void trimTask() {
  if (calRun && calManualMode) return; // В ручной калибровке trim не работает
  
  float vNow = vbat;
  bool stationary = fabs(vNow - vLast) < 0.04f;
  vLast = vNow;
  if ((!swState && !TRIM_WHEN_OFF) || fault || !trimEnabled || !checksOn) return;
  if (swState && millis() - tSwOn < 3000UL) return;
  if (vNow < 1.0f) return;
  float err = vRamp - vNow;
  bool loaded = swState && ibat > 0.3f;
  if (inCurrentLimit()) {
    float cvT = vbat + CC_CV_MARGIN;
    if (cvT > vRamp) cvT = vRamp;
    if (cvT < 1.0f) cvT = 1.0f;
    float t = (float)cvVoltsToDuty(cvT) - (float)cvVoltsToDuty(vSet);
    if (t > 800.0f) t = 800.0f;
    if (t < -TRIM_MAX) t = -TRIM_MAX;
    trim = t;
    applyCv();
    return;
  }
  if (loaded && err > TRIM_LIMIT_ERR) {
    float d = trimGood - trim;
    if (d > 30.0f) d = 30.0f;
    if (d < -30.0f) d = -30.0f;
    if (d != 0.0f) { trim += d; applyCv(); }
    return;
  }
  if (inCurrentLimit() || !stationary) return;
  if (err > 2.5f) return;
  float step;
  if (err > 0.03f) {
    step = -err * 40.0f;
    if (step < -cfg.vUp) step = -cfg.vUp;
  } else if (err < -0.06f) {
    step = -err * 40.0f;
    float maxUp = loaded ? cfg.vDown * 0.5f : cfg.vDown;
    if (step > maxUp) step = maxUp;
  } else {
    trimGood = trim;
    return;
  }
  trim += step;
  if (trim > TRIM_MAX) trim = TRIM_MAX;
  if (trim < -TRIM_MAX) trim = -TRIM_MAX;
  applyCv();
}

// ---------------- Calibration (Auto and Manual) ----------------
static uint8_t calTmo = 0;

static void calStart() {
  if (calRun) return;
  if (fault) { Serial.println(F("refused: clear the fault first")); return; }
  if (!checksOn) { Serial.println(F("refused: sensors are off (faults 1 first)")); return; }
  calWasOff = !swState;
  if (calWasOff) psuOn(true);
  measure();
  if (ibat > 0.15f) {
    if (calWasOff) psuOff();
    Serial.println(F("refused: disconnect the load, table is calibrated at no load"));
    return;
  }
  calVSetSaved = vSet;
  calWasTrim = trimEnabled; trimEnabled = true;
  
  tabValid = false;
  { TabCfg t; t.magic = 0; EEPROM.put(TAB_ADDR, t); }
  calIdx = 0; calWinT = 0; calPtT = millis();
  calTmo = 0;
  calRun = true;
  
  if (calManualMode) {
    vSet = TAB_V0; vRamp = vSet;
    setCvDuty(1023); // Начинаем с максимума скважности (минимум напряжения)
    trim = 0; trimGood = 0; 
    Serial.println(F("cal manual: sweep 2.5..15.0 V, adjust duty and press to accept"));
  } else {
    Serial.println(F("cal tab: sweep 2.0..14.5 V, keep the load disconnected"));
  }
}

static void calAbort() {
  calRun = false;
  vSet = calVSetSaved; vRamp = vSet;
  trimEnabled = calWasTrim;
  applyCv();
  if (calWasOff) psuOff();
  Serial.println(F("cal aborted"));
}

static void calTask() {
  if (!calRun) return;
  
  if (calManualMode) {
    return; 
  }

  float target = TAB_V0 + (float)calIdx * TAB_STEP;
  if (vSet != target) { vSet = target; vRamp = target; applyCv(); }
  if (fabs(vbat - target) < 0.03f) {
    if (calWinT < 600) calWinT += 100;
  } else calWinT = 0;
  bool timeout = millis() - calPtT > 15000UL;
  if (calWinT < 600 && !timeout) return;
  if (timeout && calWinT < 600) {
    calTmo++;
    Serial.print(F("cal: settle timeout at ")); Serial.print(target, 1);
    Serial.print(F(" V (")); Serial.print(calTmo); Serial.println(F("/3), point NOT recorded"));
    if (calTmo >= 3) {
      Serial.println(F("cal: too many timeouts - sweep aborted, table left INVALID"));
      calAbort();
      return;
    }
    if (calIdx > 0) tab[calIdx] = tab[calIdx - 1];
    else tab[calIdx] = cvVoltsToDuty(target);
  } else {
    calTmo = 0;
    uint16_t rec = (uint16_t)(tabDuty(target) + trim + 0.5f);
    if (rec < 1) rec = 1;
    if (rec > 1023) rec = 1023;
    if (calIdx > 0 && rec > tab[calIdx - 1]) rec = tab[calIdx - 1];
    tab[calIdx] = rec;
    Serial.print(F("cal ")); Serial.print(target, 1);
    Serial.print(F(" V -> duty ")); Serial.println(rec);
  }
  calIdx++; calWinT = 0; calPtT = millis();
  if (calIdx >= TAB_N) {
    tabValid = true;
    tabSave();
    calRun = false;
    vSet = calVSetSaved; vRamp = vSet;
    trimEnabled = calWasTrim;
    
    trim = 0;
    trimGood = 0;

    applyCv();
    if (calWasOff) psuOff();
    beep(2, 80, 80);
    Serial.println(F("cal tab done, table saved"));
  }
}

static void calAcceptPoint() {
  if (!calRun || !calManualMode) return;
  
  tab[calIdx] = cvDuty;
  if (calIdx > 0 && tab[calIdx] > tab[calIdx - 1]) tab[calIdx] = tab[calIdx - 1];
  
  Serial.print(F("cal manual ")); Serial.print(TAB_V0 + (float)calIdx * TAB_STEP, 1);
  Serial.print(F(" V -> duty ")); Serial.println(tab[calIdx]);
  
  calIdx++;
  
  if (calIdx >= TAB_N) {
    tabValid = true;
    tabSave();
    calRun = false;
    vSet = calVSetSaved; vRamp = vSet;
    trimEnabled = calWasTrim;
    trim = 0;
    trimGood = 0;
    applyCv();
    if (calWasOff) psuOff();
    beep(2, 80, 80);
    Serial.println(F("cal manual done, table saved"));
  } else {
    float target = TAB_V0 + (float)calIdx * TAB_STEP;
    vSet = target; vRamp = target;
    beep(1, 50, 0);
  }
}

// ---------------- ДОБАВЛЕНА ФУНКЦИЯ updateFan ----------------
static void updateFan() {
#if FAN_ENABLE
  bool hasSensor = !isnan(tHeat) || !isnan(tInt);
  float tMax = -100;
  if (!isnan(tHeat)) tMax = tHeat;
  if (!isnan(tInt) && tInt > tMax) tMax = tInt;
  if (hasSensor) {
    if (!fanOn && tMax > FAN_ON_C) fanOn = true;
    else if (fanOn && tMax < FAN_OFF_C) fanOn = false;
  } else {
    if (!fanOn && fabs(ibat) > FAN_ON_AMPS) fanOn = true;
    else if (fanOn && fabs(ibat) < FAN_ON_AMPS * 0.5f) fanOn = false;
  }
  digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
#endif
}
// -----------------------------------------------------------

// ---------------- formatting ----------------
static const char* f2(float v, uint8_t w, uint8_t p) {
  static char b[4][14];
  static uint8_t i = 0;
  i = (i + 1) & 3;
  dtostrf(v, w, p, b[i]);
  return b[i];
}
static const char* fT(float t) {
  if (isnan(t)) return " --";
  return f2(t, 3, 0);
}

// ---------------- display ----------------
static void drawBig(uint8_t baseline, const char* val, const char* unit) {
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(0, baseline, val);
  uint8_t w = u8g2.getStrWidth(val);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(w + 2, baseline, unit);
}
static void drawSetLine(uint8_t baseline, const char* txt, bool sel) {
  u8g2.setFont(u8g2_font_6x10_tf);
  if (sel) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(83, baseline - 9, 45, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(84, baseline, txt);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(84, baseline, txt);
  }
}
static void drawMain() {
  char b[28];
  float iShow = ibat;
  if (fabs(iShow) < 0.025f) iShow = 0;

  drawBig(20, checksOn ? f2(vbat, 5, 2) : "--.--", "V");
  drawBig(44, checksOn ? f2(iShow, 5, 2) : "--.--", "A");

  u8g2.setFont(u8g2_font_6x10_tf);
  const char* mode = fault ? "FLT" : (!swState ? "OFF" : (inCurrentLimit() ? "CC" : "CV"));
  u8g2.drawStr(84, 9, mode);
  if (checksOn) {
    snprintf(b, sizeof b, "%sW", f2(vbat * iShow, 5, 1));
    u8g2.drawStr(84, 21, b);
  }
  snprintf(b, sizeof b, "V%s", f2(vSet, 6, 2));
  drawSetLine(36, b, uiField == 0);
  snprintf(b, sizeof b, "I%sA", f2(iSet, 5, 2));
  drawSetLine(49, b, uiField == 1);

  u8g2.setFont(u8g2_font_6x10_tf);
  if (!checksOn) {
    u8g2.drawStr(0, 62, "NO V/I CHECKS");
  } else if (swState) {
    uint32_t t = (millis() - tSwOn) / 1000UL;
    snprintf(b, sizeof b, "ON %02u:%02u:%02u", (unsigned)(t / 3600), (unsigned)((t / 60) % 60), (unsigned)(t % 60));
    u8g2.drawStr(0, 62, b);
  } else if (!outReady()) {
    snprintf(b, sizeof b, "OFF wait %us", (unsigned)waitSeconds());
    u8g2.drawStr(0, 62, b);
  } else {
    u8g2.drawStr(0, 62, "OUT OFF");
  }
  float th = isnan(tHeat) ? tInt : tHeat;
  snprintf(b, sizeof b, "Th%s", fT(th));
  u8g2.drawStr(96, 62, b);
}
static void drawInfo() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(b, sizeof b, "ADC V%4d I%4d", (int)(fVc + 0.5f), (int)(fIc + 0.5f));
  u8g2.drawStr(0, 9, b);
  snprintf(b, sizeof b, "CV %4u tr%+d", cvDuty, (int)(trim + (trim >= 0 ? 0.5f : -0.5f)));
  u8g2.drawStr(0, 21, b);
  snprintf(b, sizeof b, "CC %4u ~%sA", ccDuty, f2(ibat, 1, 1));
  u8g2.drawStr(0, 33, b);
  snprintf(b, sizeof b, "Tb%s Th%s Ti%s", fT(tBat), fT(tHeat), fT(tInt));
  u8g2.drawStr(0, 45, b);
  uint16_t tk, bd;
  noInterrupts(); tk = encTicks; bd = encBad; interrupts();
  snprintf(b, sizeof b, "%s %03ux%02u", FW_VERSION, (unsigned)(tk % 1000), (unsigned)(bd % 100));
  u8g2.drawStr(0, 57, b);
}
static void drawCal() {
  char b[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  
  if (calRun) {
    if (calManualMode) {
      float target = TAB_V0 + (float)calIdx * TAB_STEP;
      snprintf(b, sizeof b, "MANUAL %u/%u", (unsigned)calIdx + 1, (unsigned)TAB_N);
      u8g2.drawStr(0, 12, b);
      
      // Используем dtostrf для надежного вывода float
      char tbuf[10];
      dtostrf(target, 4, 1, tbuf); 
      snprintf(b, sizeof b, "Target: %s V", tbuf);
      u8g2.drawStr(0, 26, b);
      
      snprintf(b, sizeof b, "Duty: %u", cvDuty);
      u8g2.drawStr(0, 40, b);
      u8g2.drawStr(0, 54, "Press=OK, Hold=STOP");
    } else {
      snprintf(b, sizeof b, "AUTO RUN %u/%u", (unsigned)calIdx + 1, (unsigned)TAB_N);
      u8g2.drawStr(0, 12, b);
      u8g2.drawStr(0, 30, "Please wait...");
      u8g2.drawStr(0, 60, "Hold IN RUN = STOP");
    }
  } else {
    u8g2.drawStr(0, 12, "V TABLE CAL");
    snprintf(b, sizeof b, "TABLE: %s", tabValid ? "VALID" : "EMPTY");
    u8g2.drawStr(0, 26, b);
    
    if (calManualMode) u8g2.drawStr(0, 40, "> MANUAL CAL <");
    else u8g2.drawStr(0, 40, "> AUTO CAL <");
    
    u8g2.drawStr(0, 54, "Press=Mode, Hold=RUN");
  }
}
static void draw() {
  u8g2.clearBuffer();
  if (uiField == 2) drawInfo();
  else if (uiField == 3) drawCal();
  else drawMain();
  if (fault) {
    char b[28];
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 53, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    snprintf(b, sizeof b, "FAULT %s", faultMsg);
    u8g2.drawStr(2, 62, b);
    u8g2.setDrawColor(1);
  }
  u8g2.sendBuffer();
}

// ---------------- serial ----------------
static void printStatus() {
  Serial.print(F("out=")); Serial.print(swState ? F("ON") : F("OFF"));
  Serial.print(F(" Vset=")); Serial.print(vSet, 2);
  Serial.print(F(" Iset=")); Serial.print(iSet, 2);
  Serial.print(F(" Imax=")); Serial.print(iLimit(), 2);
  Serial.print(F(" V=")); Serial.print(vbat, 2);
  Serial.print(F(" I=")); Serial.print(ibat, 2);
  Serial.print(F(" mode=")); Serial.print(!swState ? F("-") : (inCurrentLimit() ? F("CC") : F("CV")));
  Serial.print(F(" cv=")); Serial.print(cvDuty);
  Serial.print(F(" cvW=")); Serial.print(cvWho);
  Serial.print(F(" cvN=")); Serial.print(cvWn);
  Serial.print(F(" trim=")); Serial.print(trim, 1);
  Serial.print(F(" tg=")); Serial.print(trimGood, 1);
  Serial.print(F(" cc=")); Serial.print(ccDuty);
  Serial.print(F(" vr=")); Serial.print(vRamp, 2);
  Serial.print(F(" tab=")); Serial.print(tabValid ? 1 : 0);
  Serial.print(F(" ceil=")); Serial.print(cfg.vCeil, 1);
  Serial.print(F(" sl=")); Serial.print(cfg.vSlew, 2);
  Serial.print(F(" vEst=")); Serial.print(vEst, 2);
  Serial.print(F(" Tb=")); Serial.print(tBat, 1);
  Serial.print(F(" Th=")); Serial.print(tHeat, 1);
  Serial.print(F(" Ti=")); Serial.print(tInt, 1);
  Serial.print(F(" fan=")); Serial.print(fanOn ? 1 : 0);
  Serial.print(F(" adcV=")); Serial.print(fVc, 1);
  Serial.print(F(" adcI=")); Serial.print(fIc, 1);
  Serial.print(F(" zero=")); Serial.print(iZero, 1);
  Serial.print(F(" fault=")); Serial.println(fault ? faultMsg : "none");
}
static void printHelp() {
  Serial.println(F("help st log | v V | i A | on [force] | off | kill | trim 0/1"));
  Serial.println(F("clr zero faults 0/1 | isign [-1|1] | ccpi [KP KI] | vslew [V/s] | vdown [n] | vup [n] | cal vc V | cal i A | cal tab | cal lim A | ceil v V | ceil i A | ceil i0 A | imax [A] | cc duty | save | defaults"));
}
static void execCmd(char* s) {
  char* cmd = strtok(s, " ");
  char* a1 = strtok(NULL, " ");
  char* a2 = strtok(NULL, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) { printHelp(); return; }
  if (!strcmp(cmd, "st")) { printStatus(); return; }
  if (!strcmp(cmd, "draw") && a1) { drawOn = atoi(a1) != 0; Serial.println(drawOn ? F("draw on") : F("draw OFF (frozen)")); return; }
  if (!strcmp(cmd, "log")) { logOn = a1 && atoi(a1); Serial.println(logOn ? F("log on") : F("log off")); return; }
  if (!strcmp(cmd, "off")) { psuOff(); printStatus(); return; }
  if (!strcmp(cmd, "cvdump")) {
    Serial.print(F("cvWho=")); Serial.println(cvWho);
    Serial.print(F("cvN=")); Serial.println(cvWn);
    for (uint8_t k = 0; k < CV_TRACE_N; k++) {
      uint8_t j = (cvT_i + k) % CV_TRACE_N;
      Serial.print(F("["));
      if (k < 10) Serial.print(k);
      Serial.print(F("] vr=")); Serial.print(cvT_vr[j], 2);
      Serial.print(F(" b=")); Serial.print(cvT_b[j]);
      Serial.print(F(" trim=")); Serial.print(cvT_tr[j], 1);
      Serial.print(F(" d=")); Serial.print(cvT_d[j]);
      Serial.print(F(" dMin=")); Serial.print(cvT_min[j]);
      Serial.print(F(" t20=")); Serial.println(cvT_t20[j]);
    }
    return;
  }
  if (!strcmp(cmd, "kill")) { safeOutputs(); Serial.println(F("output off, PWM at safe values (v/i or clr restores)")); return; }
  if (!strcmp(cmd, "on")) { psuOn(a1 && !strcmp(a1, "force")); printStatus(); return; }
  if (!strcmp(cmd, "clr")) { clearFault(); return; }
  if (!strcmp(cmd, "zero")) {
    if (swState) { Serial.println(F("refused: switch the output off first")); return; }
    measureZero(); Serial.print(F("zero=")); Serial.println(iZero, 1); return;
  }
  if (!strcmp(cmd, "isign")) {
    if (a1) {
      int s = atoi(a1);
      if (s != 1 && s != -1) { Serial.println(F("usage: isign [-1|1]")); return; }
      if (s != cfg.iSign) {
        cfg.iSign = (int8_t)s;
        cfg.iGain = -cfg.iGain;
        ccInt = 0; ccDuty = 0; applyCc();
        Serial.println(F("isign flipped, iGain sign auto-corrected (not saved until \"save\")"));
      }
    }
    Serial.print(F("isign=")); Serial.println(cfg.iSign);
    return;
  }
  if (!strcmp(cmd, "ccpi")) {
    if (a1 && a2) {
      float p = atof(a1), i = atof(a2);
      if (p >= 10 && p <= 1000 && i >= 10 && i <= 5000) {
        cfg.ccKp = p; cfg.ccKi = i; ccInt = 0;
        Serial.println(F("ccpi set (not saved until \"save\")"));
      } else { Serial.println(F("range: KP 10..1000, KI 10..5000")); return; }
    }
    Serial.print(F("KP=")); Serial.print(cfg.ccKp, 0);
    Serial.print(F(" KI=")); Serial.println(cfg.ccKi, 0);
    return;
  }
  if (!strcmp(cmd, "vslew")) {
    if (a1) {
      float s2 = atof(a1);
      if (s2 >= 0 && s2 <= 20) { cfg.vSlew = s2; Serial.println(F("vslew set (not saved until \"save\")")); }
      else { Serial.println(F("range: 0..20 V/s (0 = instant)")); return; }
    }
    Serial.print(F("vslew=")); Serial.print(cfg.vSlew, 1); Serial.println(F(" V/s"));
    return;
  }
  if (!strcmp(cmd, "vdown")) {
    if (a1) {
      float s2 = atof(a1);
      if (s2 >= 1 && s2 <= 50) { cfg.vDown = s2; Serial.println(F("vdown set (not saved until \"save\")")); }
      else { Serial.println(F("range: 1..50 counts/tick")); return; }
    }
    Serial.print(F("vdown=")); Serial.println(cfg.vDown, 0);
    return;
  }
  if (!strcmp(cmd, "vup")) {
    if (a1) {
      float s2 = atof(a1);
      if (s2 >= 1 && s2 <= 50) { cfg.vUp = s2; Serial.println(F("vup set (not saved until \"save\")")); }
      else { Serial.println(F("range: 1..50 counts/tick")); return; }
    }
    Serial.print(F("vup=")); Serial.println(cfg.vUp, 0);
    return;
  }
  if (!strcmp(cmd, "faults")) {
    if (swState) { Serial.println(F("refused: switch the output off first")); return; }
    checksOn = a1 && atoi(a1);
    if (checksOn) Serial.println(F("V/I checks ON"));
    else { fault = false; faultMsg = ""; Serial.println(F("V/I checks OFF (bench without sensors only, until reboot)")); }
    applyCv(); applyCc();
    return;
  }
  if (!strcmp(cmd, "trim")) {
    trimEnabled = a1 && atoi(a1);
    if (!trimEnabled) { trim = 0; trimGood = 0; applyCv(); }
    Serial.println(trimEnabled ? F("trim on") : F("trim off (trim reset)"));
    return;
  }
  if (!strcmp(cmd, "save")) { cfgSave(); spSaveNow(); Serial.println(F("saved")); return; }
  if (!strcmp(cmd, "defaults")) { cfgDefaults(); trim = 0; trimGood = 0; ccInt = 0; if (iSet > iLimit()) iSet = iLimit(); Serial.println(F("defaults loaded (not saved)")); applyCv(); applyCc(); return; }

  if (!strcmp(cmd, "v") && a1) { setV(atof(a1)); Serial.print(F("Vset=")); Serial.println(vSet, 2); return; }
  if (!strcmp(cmd, "i") && a1) { setI(atof(a1)); Serial.print(F("Iset=")); Serial.println(iSet, 2); return; }
  if (!strcmp(cmd, "imax")) {
    if (a1) {
      float m = atof(a1);
      if (m < 0.5f) m = 0.5f;
      if (m > 30.0f) m = 30.0f;
      cfg.iMax = m;
      if (iSet > iLimit()) { iSet = iLimit(); spTouch(); }
      applyCc();
    }
    Serial.print(F("Imax=")); Serial.print(iLimit(), 2);
    Serial.print(F(" (imax=")); Serial.print(cfg.iMax, 2);
    Serial.print(F(", ceil i=")); Serial.print(cfg.iCeil, 2);
    Serial.println(F(") - not saved until \"save\""));
    return;
  }

  if (!strcmp(cmd, "cal") && a1) {
    if (!strcmp(a1, "tab")) { calManualMode = false; calStart(); return; }
    if (!strcmp(a1, "manual")) { calManualMode = true; calStart(); return; }
    if (!strcmp(a1, "dump")) {
      Serial.print(F("tabValid=")); Serial.println(tabValid ? 1 : 0);
      for (uint8_t i = 0; i < TAB_N; i++) {
        Serial.print(F("tab[")); Serial.print(i);
        Serial.print(F("] ")); Serial.print(TAB_V0 + i * TAB_STEP, 1);
        Serial.print(F(" V -> ")); Serial.println(tab[i]);
      }
      return;
    }
    if (!a2) return;
    float val = atof(a2);
    float vnode = fVc * VREF / 1023.0f;
    if (!strcmp(a1, "vc")) {
      if (vnode < 0.05f) { Serial.println(F("no voltage on the divider")); return; }
      cfg.kClosed = (val + (swState ? ibat * R_SERIES : 0.0f)) / vnode;
      Serial.print(F("kClosed=")); Serial.println(cfg.kClosed, 4);
    } else if (!strcmp(a1, "i")) {
      float dc = (fIc - iZero) * cfg.iSign;
      if (fabs(dc) < 5.0f) { Serial.println(F("current too small")); return; }
      cfg.iGain = val / dc; Serial.print(F("iGain=")); Serial.println(cfg.iGain, 5);
    } else if (!strcmp(a1, "lim")) {
      if (!swState || !checksOn || val < 0.2f) { Serial.println(F("need: output ON, sensors on, a real current > 0.2 A")); return; }
      if (ccDuty < 50) { Serial.println(F("limit too low for calibration, set it higher")); return; }
      cfg.iCeil = cfg.i0 + (val - cfg.i0) * 1023.0f / ccDuty;
      Serial.print(F("iCeil=")); Serial.println(cfg.iCeil, 3);
      if (iSet > iLimit()) iSet = iLimit();
      applyCc();
    }
    return;
  }
  if (!strcmp(cmd, "ceil") && a1 && a2) {
    float val = atof(a2);
    if (!strcmp(a1, "v")) { cfg.vCeil = val; trim = 0; trimGood = 0; if (vSet > cfg.vCeil) vSet = cfg.vCeil; applyCv(); }
    else if (!strcmp(a1, "i")) { cfg.iCeil = val; if (iSet > iLimit()) iSet = iLimit(); applyCc(); }
    else if (!strcmp(a1, "i0")) { cfg.i0 = val; applyCc(); }
    Serial.println(F("ok")); return;
  }
  if (!strcmp(cmd, "cc") && a1) {
    long d = atol(a1);
    if (d < 0) d = 0;
    if (d > 1023) d = 1023;
#if CC_PI
    ccDuty = (uint16_t)d;
    setCcDuty(ccDuty);
    tCcManual = millis() + 3000UL;
    Serial.print(F("raw CC duty=")); Serial.print(ccDuty);
    Serial.println(F(" (PI frozen 3 s)"));
#else
    { long dm = ccAmpsToDuty(iLimit()); if (d > dm) d = dm; }
    setCcDuty((uint16_t)d);
    Serial.print(F("raw CC duty=")); Serial.print(ccDuty);
    Serial.print(F(" (~")); Serial.print(ccDutyToAmps(ccDuty), 2); Serial.println(F(" A by the map)"));
#endif
    return;
  }
  Serial.println(F("unknown command, try help"));
}
static char lineBuf[48];
static uint8_t lineLen = 0;
static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen) { lineBuf[lineLen] = 0; execCmd(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------------- encoder / UI ----------------
static void handleEncoder() {
  noInterrupts();
  int8_t d = encDelta; encDelta = 0;
  bool p = encPress; encPress = false;
  bool l = encLong; encLong = false;
  interrupts();

  if (l) {
    if (calRun) { 
      calAbort(); 
      beep(1, 50, 0); 
    }
    else if (fault) { clearFault(); beep(1, 100, 0); }
    else if (uiField == 3) { 
      // Долгое нажатие запускает калибровку (того режима, который выбран)
      calStart(); 
      if (calManualMode) beep(2, 50, 50); else beep(1, 100, 0);
    }
    else if (swState) psuOff();
    else psuOn(false);
  }
  
  // 1. Обработка короткого нажатия (переключение пунктов)
  if (p && !calRun) {
    if (uiField == 0) {
      uiField = 1; // V -> I
    } else if (uiField == 1) {
      uiField = 2; // I -> Info
    } else if (uiField == 2) {
      uiField = 3; // Info -> CAL
    } else if (uiField == 3) {
      // На экране CAL: нажатие сначала меняет режим
      // Но чтобы выйти на главный экран, нужно нажать еще раз.
      // Мы используем флаг, чтобы понять, что мы уже переключили режим.
      static bool calModeSelected = false;
      if (!calModeSelected) {
        calManualMode = !calManualMode;
        beep(1, 50, 0);
        calModeSelected = true; // Следующее нажатие уйдет на главный экран
      } else {
        uiField = 0; // Возврат на главный экран
        calModeSelected = false; // Сброс флага
      }
    }
  }
  
  // 2. Обработка вращения энкодера (изменение значения)
  if (d) {
    if (calRun) {
      // Во время калибровки
      if (calManualMode) {
        long newDuty = (long)cvDuty + d;
        if (newDuty < 0) newDuty = 0;
        if (newDuty > 1023) newDuty = 1023;
        setCvDuty((uint16_t)newDuty);
        cvWho = "manual";
      }
    } else {
      // Вне калибровки
      if (uiField == 0) {
        setV(vSet + d * V_STEP); // Меняем напряжение
      } else if (uiField == 1) {
        setI(iSet + d * I_STEP); // Меняем ток
      } else if (uiField == 3) {
        // На экране CAL: вращение переключает режим (AUTO/MANUAL)
        calManualMode = !calManualMode;
        beep(1, 50, 0);
        // Сбрасываем флаг, чтобы следующее нажатие снова переключило режим,
        // а не сразу вышло на главный экран.
        // (Для этого используем статическую переменную, объявленную выше)
      }
    }
  }
  
  // Принятие точки в ручном режиме (короткое нажатие во время калибровки)
  if (p && calRun && calManualMode) {
    calAcceptPoint();
  }
}

// ---------------- setup / loop ----------------
void setup() {
  pinMode(PIN_SWITCH, OUTPUT); digitalWrite(PIN_SWITCH, LOW);
  pinMode(PIN_CV_PWM, OUTPUT); digitalWrite(PIN_CV_PWM, LOW);
  pinMode(PIN_CC_PWM, OUTPUT); digitalWrite(PIN_CC_PWM, LOW);
  pinMode(PIN_FAN, OUTPUT);    digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BEEP, OUTPUT);   digitalWrite(PIN_BEEP, LOW);
  pwmInit();
  cvWho = "boot";
  setCvDuty(1023);
  setCcDuty(0);

  Serial.begin(115200);
  cfgLoad();
  tabLoad();
  spLoad();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40);
  OCR4A = 249;
  OCR4B = 100;
  TIMSK4 = _BV(OCIE4B);
  interrupts();

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "Bench PSU");
  u8g2.drawStr(0, 24, FW_VERSION);
  u8g2.drawStr(0, 40, "zeroing current...");
  u8g2.sendBuffer();

  delay(300);
  measureZero();

  vEst = cfg.vCeil;
  applyCv();
  applyCc();

  beep(1, 80, 0);
  Serial.print(F("Bench PSU fw ")); Serial.println(FW_VERSION);
  Serial.print(F("ACS zero counts: ")); Serial.println(iZero, 1);
  printHelp();
}

void loop() {
  static uint32_t tMeas = 0, tDraw = 0, tLog = 0, tTrim = 0;
  uint32_t now = millis();

  handleSerial();
  handleEncoder();
  beepTask();

  if (now - tMeas >= 100) {
    float dt = (now - tMeas) / 1000.0f;
    if (dt > 1.0f) dt = 0.1f;
    tMeas = now;
    measure();
    estimateOutput(dt);
    ccPiTask(dt);
    if (calRun) calTask();
    
    // В режиме ручной калибровки мы полностью управляем ШИМ вручную.
    // Автоматика не должна вмешиваться и сбрасывать cvDuty.
    if (!calRun || !calManualMode) {
if (vSet > vRamp) {
      if (cfg.vSlew > 0) {
        vRamp += cfg.vSlew * dt;
        if (vRamp > vSet) vRamp = vSet;
        applyCv();
      } else { 
        vRamp = vSet; 
        applyCv(); 
      }
    } else if (vRamp > vSet) {
      vRamp = vSet;
      trim = 0;
      trimGood = 0;
      applyCv();
    }
    }
    updateFan();
  }
  if (now - tTrim >= 500) {
    tTrim = now;
    trimTask();
  }
  if (spDirty && now - tSpChange > 5000UL) spSaveNow();
  if (drawOn && now - tDraw >= 150) {
    tDraw = now;
    draw();
  }
  if (logOn && now - tLog >= 1000) {
    tLog = now;
    printStatus();
  }
}