// =====================================================================
// Bench power supply firmware v0.5.3  (based on v0.5.2)
// v0.5.3: fixed the slow CC dynamics introduced in v0.5.2.
//         - In CC trim now TRACKS the target directly instead of slewing
//           4 counts/0.5 s tick: entering CC at a low output parked trim
//           hundreds of counts away from the target, so the limit took
//           ~1 min to settle and the current floated above the setpoint
//           until it arrived.
//         - The "relax to trimGood" path (loaded, big CV error) is now
//           proportional (10%/tick, cap 6 counts) instead of a fixed 2
//           counts/tick: releasing CC after a long low-current limit hands
//           the output back to CV in seconds, not ~1 min.
// Bench power supply firmware v0.5.2  (based on v0.5.1)
// v0.5.2: the set voltage is an ABSOLUTE ceiling - never exceeded, in any
//         mode. Two changes:
//         1. applyCv() hard-clamps the CV demand: the effective target is
//            never higher than vSet + VSET_GUARD (0.15 V), no matter what
//            trim says. Previously trim parked at -TRIM_MAX in CC, so when
//            the current limit released, the output jumped ABOVE the set
//            voltage (toward ~14 V for a 12 V setting) until trim returned.
//         2. In current limit trim no longer runs to -TRIM_MAX. It now
//            tracks vbat + CC_CV_MARGIN (0.2 V, capped at vSet): the analog
//            CC loop gets the minimum injection stress, and releasing the
//            limit hands over a CV demand already equal to ~the measured
//            output - no overshoot above vSet at all.
// Bench power supply firmware v0.5.1  (based on v0.5)
// v0.5.1: in current limit the trim loop now WITHDRAWS the CV injection
//         (trim -> -TRIM_MAX, 4 counts per 0.5 s tick) instead of freezing.
//         With trim frozen the CV chain kept injecting ~0.7-1.3 mA into the
//         FB node, so the analog CC loop (max ~3.4 mA) could not pull the
//         output down: measured limit floor was ~2.7 A at iSet 0.1 A with
//         I_lim ~= 2.7 + 0.55 * iSet. After the fix I_lim = W / Rshunt.
// Bench power supply firmware v0.5  (based on v0.4.2)
// v0.5: trim loop no longer winds up under the module current limit.
//       Bench result: with trim off the supply held 5.8 W, with trim on it slid to 5.2 W and,
//       after OFF, the no-load output jumped to 8-9 V and came down very slowly.
//       - "loaded" (I > 0.3 A) and the output more than TRIM_LIMIT_ERR (0.4 V) below the setting
//         = current limit, not a CV error: trim is frozen and relaxes (2 counts / 0.5 s) back
//         to trimGood, the last trim value that gave a correct voltage;
//       - on OFF the trim jumps straight back to trimGood (no-load output returns to the set value);
//       - without a load, trim goes down (lower voltage) up to 3 counts per step instead of 1;
//       - "st" shows tg= (trimGood). "trim 0" = off + trim and trimGood reset, "trim 1" = on.
// Bench power supply firmware v0.4.2  (based on v0.4.1)
// v0.4.2: fixes the "stuck at 5.4 V" trap. CC is now detected by current only (I > 0.88 x Iset);
//         the old "voltage far below the setting" rule froze the trim loop under load.
//         Trim authority 60 -> 120 counts, freeze only when the error exceeds 2.5 V,
//         trim runs every 0.5 s with bigger steps (up to 20 counts, gain 40 counts/V).
// Bench power supply firmware v0.4.1  (based on v0.4)
// v0.4.1: output switch is installed: the voltage trim loop now also runs while the output is
//         OFF (TRIM_WHEN_OFF), so the output settles to the set value before ON and the
//         "wait" message does not hang at the open-loop level (e.g. 7.5 V for a 7.0 V setting).
// Bench power supply firmware v0.4  (based on v0.3)
// v0.4: imax <A> = separate hard cap for the current SETPOINT (default 8.0 A, saved with
//       "save"). The CC map (ceil i / cal lim) is not touched by it: the effective maximum is
//       min(imax, ceil i). Applies to the encoder, "i", "cc" and the stored setpoint.
// v0.2: CC mode also detected by a low output voltage (limit below the set value),
//       trim loop is frozen when the error is outside its authority (no wind-up in CC).
// v0.3: NEW CC reference. D44 drives the module CC pad 2 directly (PWM -> 7.45k -> node,
//       node: 2.36k to GND + 1..10 uF to GND, pad 2 = node, pad 1 = GND, pad 3 not used,
//       no 2N7000). Full scale ~1.2 V on pad 2 = highest current limit.
//       Higher duty = HIGHER limit (v0.2 was inverted). Current limit mapping:
//         I(duty) = i0 + (iCeil - i0) * duty / 1023
//       When the output is OFF (and at boot / fault) the duty is 0 = lowest limit (~0.1 A).
//       New commands: cc <duty 0..1023> (raw CC duty), ceil i0 <A>.
// Board : MKS Gen v1.4 (ATmega2560, RAMPS 1.4 pinout)
// LCD   : RepRapDiscount Full Graphic Smart Controller (ST7920, U8g2)
// Two settings: output voltage and current limit. Output on/off.
//
// Library: U8g2 (Library Manager). Board: Arduino Mega 2560.
//
// Hardware is the same as in the charger wiring document:
//   D5   CV PWM  Timer3 OC3A   SERVOS header, column 2.1 (signal pin)
//   D44  CC PWM  Timer5 OC5C   AUX-2, bottom row, 4th column (-> 7.45k -> CC pad 2)
//   D40  output switch gate    AUX-2, top row, 3rd column (HIGH = on)
//   D59 (A5)  output voltage divider 120k/22k   AUX-2, bottom row, 2nd column
//   D64 (A10) ACS712 VIOUT                       AUX-2, bottom row, 3rd column
//   D67 (A13) NTC 'battery' (not required)       NTC connector E0
//   D69 (A15) NTC heatsink                       NTC connector E1
//   D68 (A14) NTC inside converter               NTC connector BED
//   D10  fan MOSFET            screw terminal E0 (= supply voltage, 24 V fan)
//   LCD  EXP1: D16 CS, D17 data, D23 clock, D35 button, D37 beeper, 5V, GND
//        EXP2: D31, D33 encoder A/B (and GND)
//
// How it works
//   Voltage: open-loop PWM into the module FB, then a slow trim loop (1 Hz)
//            corrects the duty from the measured output voltage.
//            Trim works only when the output is ON, sensors are trusted and
//            the unit is not in current limit.
//   Current: open-loop PWM into the CC network (no trim). Calibrate once with
//            'cal lim' (see below).
//   Buck cannot sink current: after lowering the voltage with no load the
//            output falls slowly (tau = C x ~4.3k). The unit refuses to switch
//            ON while the output is more than 0.3 V above the set value
//            ('wait' on the screen).
//
// Encoder
//   Press       : select V -> I -> info screen -> V
//   Rotate      : change selected value, 0.05 V / 0.05 A per click (V_STEP, I_STEP)
//   Long press  : output ON / OFF  (when a fault is latched: clear the fault)
//   Display reset button = hardware stop (MCU reboots, output OFF).
//   Boot always starts with the output OFF.
//
// Serial 115200 commands (end with Enter):
//   help | st | log 0/1
//   v <volts>   set voltage        i <amps>   set current limit
//   on | on force | off | kill     output on / on ignoring 'wait' / off / off + safe PWM
//   trim 0/1    voltage trim loop (default 1)
//   clr         clear fault        zero       ACS712 zero (output must be OFF)
//   faults 0/1  V/I checks and measurements trusted (0 = bench without sensors,
//               until reboot; the screen shows '--' for V and I)
//   cal vc <V>  calibrate voltage: measure the module output with a multimeter
//               (output ON or OFF), type the value
//   cal i <A>   calibrate current gain with a known load current
//   cal lim <A> calibrate current limit: output ON, load heavy enough that the
//               unit sits in current limit, type the real current (multimeter)
//   imax [A]    cap of the current setpoint (no argument = show); saved by "save"
//   ceil v <V> | ceil i <A> | ceil i0 <A>   ceilings used by the PWM mappings
//               (ceil i = limit at duty 1023, ceil i0 = limit at duty 0)
//   cc <duty>   raw CC duty 0..1023 for measuring the limit curve (any v/i/on/off
//               command writes the duty again)
//   save | defaults
// =====================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define FW_VERSION "PSU 0.5.3"

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
#define I_SIGN          1.0f       // set -1.0f if current shows inverted
#define ENC_PULSES      4          // transitions per detent
#define ENC_DIR         1          // set -1 to reverse rotation

#define NTC_PULLUP      4700.0f    // pull-up on the board
#define NTC_R25         100000.0f
#define NTC_BETA        3950.0f

// ---------------- limits ----------------
#define V_FAULT         16.0f      // hard overvoltage trip (module ceiling is ~15 V)
#define I_FAULT         12.0f
#define T_BAT_MAX       50.0f      // only if the 'battery' NTC is connected
#define T_HEAT_MAX      80.0f
#define T_INT_MAX       75.0f

#define I_MIN_SET       0.10f      // lowest current limit you can set
#define READY_BAND      0.3f       // ON allowed when Vout <= Vset + this
#define TRIM_WHEN_OFF   1          // 1 = output switch (IRLB3034 pair) installed: V is valid when OFF, trim runs when OFF too
#define OUT_TAU_MS      9000.0f    // output discharge time constant (model, used with 'faults 0')
#define TRIM_MAX        120.0f     // trim authority in PWM counts (~2.3 V)
#define VSET_GUARD      0.15f      // CV demand never exceeds vSet + this (safety ceiling)
#define CC_CV_MARGIN    0.20f      // in CC the CV demand tracks vbat + this (<= vSet)
#define TRIM_LIMIT_ERR  0.4f       // loaded and Vout this far below Vset = current limit: trim is frozen
#define V_STEP          0.05f      // volts per encoder click (always the same speed)
#define I_STEP          0.05f      // amps per encoder click

#define FAN_ENABLE      1
#define FAN_ON_C        45.0f      // on above this (max of heatsink, inside)
#define FAN_OFF_C       40.0f
#define FAN_ON_AMPS     2.0f       // used only when no heatsink/inside sensor

// ---------------- config in EEPROM ----------------
struct Cfg {
  uint16_t magic;
  float kOpen;     // unused in the PSU firmware (kept for layout compatibility)
  float kClosed;   // Vout = Vnode * kClosed
  float iGain;     // amps per ADC count
  float vCeil;     // CV ceiling, volts (no injection)
  float iCeil;     // CC limit at duty 1023, amps
  float i0;        // CC limit at duty 0, amps (v0.3; old EEPROM images have garbage here)
  float iMax;      // cap of the current setpoint, amps (v0.4; old images: garbage -> 8.0)
};
#define CFG_MAGIC 0xC47B
static Cfg cfg;

static void cfgDefaults() {
  cfg.magic   = CFG_MAGIC;
  cfg.kOpen   = 8.0f;
  cfg.kClosed = 142.0f / 22.0f;          // 6.4545
  cfg.iGain   = 50.0f / 1023.0f;         // ACS712-20A ratiometric, ~48.9 mA/count
  cfg.vCeil   = 14.9f;
  cfg.iCeil   = 8.5f;
  cfg.i0      = 0.30f;
  cfg.iMax    = 8.0f;
}
static void cfgLoad() {
  Cfg t;
  EEPROM.get(0, t);
  if (t.magic == CFG_MAGIC) cfg = t; else cfgDefaults();
  if (!(cfg.i0 >= 0.0f && cfg.i0 < 2.0f)) cfg.i0 = 0.30f;          // field added in v0.3
  if (!(cfg.iCeil > cfg.i0 + 0.5f && cfg.iCeil < 30.0f)) cfg.iCeil = 8.5f;
  if (!(cfg.iMax >= 0.5f && cfg.iMax <= 30.0f)) cfg.iMax = 8.0f;   // field added in v0.4
}
static void cfgSave() { EEPROM.put(0, cfg); }
// highest allowed current setpoint: imax, but never above the CC map ceiling
static float iLimit() { return cfg.iMax < cfg.iCeil ? cfg.iMax : cfg.iCeil; }

struct Sp { uint16_t magic; float v; float i; };
#define SP_MAGIC 0x5B01
#define SP_ADDR  64

// ---------------- display ----------------
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /*clock=*/23, /*data=*/17, /*cs=*/16);

// ---------------- state ----------------
static float    vSet = 5.0f, iSet = 1.0f;
static bool     spDirty = false;
static uint32_t tSpChange = 0;

static uint16_t cvDuty = 1023;
static uint16_t ccDuty = 1023;
static float    trim = 0;               // PWM counts added to the open-loop CV duty
static float    trimGood = 0;           // last trim value that gave the right voltage (|err| small)
static bool     trimEnabled = true;
static uint32_t tSetChange = 0;         // last change of the voltage setting / switch on
static float    vLast = 0;

static bool     swState = false;        // output ON
static bool     fault = false;
static const char* faultMsg = "";
static bool     logOn = false;
static bool     checksOn = true;        // sensors trusted; 'faults 0' = bench without sensors

static float fVc = 0, fIc = 0;          // filtered ADC counts
static bool  fInit = false;
static float iZero = 512.0f;            // ACS712 zero, counts
static float vbat = 0, ibat = 0;        // output voltage / current (measured)
static float vEst = 15.0f;              // best knowledge of the module output voltage
static float tBat = NAN, tHeat = NAN, tInt = NAN;
static uint32_t tSwOn = 0;
static bool fanOn = false;

static uint8_t uiField = 0;             // 0 = V, 1 = I, 2 = info screen

// ---------------- encoder (Timer4 compare-B, 1 kHz poll) ----------------
// NOTE: Arduino's tone() owns the COMPA vectors of Timer0..5, so we use COMPB.
static volatile int8_t encDelta = 0;
static volatile bool   encPress = false;
static volatile bool   encLong = false;
static volatile uint16_t encTicks = 0;   // valid quadrature steps seen by the ISR
static volatile uint16_t encBad = 0;     // invalid (skipped/bounced) transitions

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

// ---------------- PWM (Timer3 -> CV, Timer5 -> CC; 10 bit, 15.6 kHz) ----------------
static void pwmInit() {
  TCCR3A = _BV(WGM31) | _BV(WGM30);       // fast PWM 10 bit
  TCCR3B = _BV(WGM32) | _BV(CS30);        // prescaler 1
  OCR3A = 0;
  TCCR5A = _BV(WGM51) | _BV(WGM50);
  TCCR5B = _BV(WGM52) | _BV(CS50);
  OCR5C = 0;
}
// duty 0 -> pin held LOW (no 1-clock spike of fast PWM at OCR=0)
static void setCvDuty(uint16_t d) {
  if (d > 1023) d = 1023;
  cvDuty = d;
  if (d == 0) { TCCR3A &= ~_BV(COM3A1); digitalWrite(PIN_CV_PWM, LOW); }
  else        { OCR3A = d; TCCR3A |= _BV(COM3A1); }
}
static void setCcDuty(uint16_t d) {
  if (d > 1023) d = 1023;
  ccDuty = d;
  if (d == 0) { TCCR5A &= ~_BV(COM5C1); digitalWrite(PIN_CC_PWM, LOW); }
  else        { OCR5C = d; TCCR5A |= _BV(COM5C1); }
}

// mappings (open loop, from the design calculations)
static float cvDutyToVolts(uint16_t d) {
  float vn = 5.0f * d / 1023.0f;
  float inj = (vn - 1.9f) / 1030.0f;
  if (inj < 0) inj = 0;
  return cfg.vCeil - 3900.0f * inj;
}
static uint16_t cvVoltsToDuty(float v) {
  float inj = (cfg.vCeil - v) / 3900.0f;
  if (inj <= 0) return 0;
  float d = (1.9f + 1030.0f * inj) / 5.0f;
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

// ---------------- beeper (non-blocking) ----------------
static uint8_t  beepLeft = 0;
static uint16_t beepOnMs = 0, beepOffMs = 0;
static uint32_t beepT = 0;
static bool     beepOn = false;
static void beep(uint8_t n, uint16_t onMs, uint16_t offMs) {
#if BEEP_ENABLE
  beepLeft = n; beepOnMs = onMs; beepOffMs = offMs;
  beepOn = false;
  beepT = millis() - offMs;       // start right away
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
  uint16_t b = cvVoltsToDuty(vSet);
  if (b == 0) { setCvDuty(0); return; }              // at the ceiling, no injection
  float d = (float)b + trim;
  // Absolute ceiling: the demand must never exceed vSet + VSET_GUARD,
  // regardless of trim state (v0.5.2). Duty is inverted: a smaller duty
  // means a higher demanded voltage, so clamp from below.
  uint16_t dMin = cvVoltsToDuty(vSet + VSET_GUARD);
  if (d < (float)dMin) d = (float)dMin;
  if (d < 1.0f) d = 1.0f;
  if (d > 1023.0f) d = 1023.0f;
  setCvDuty((uint16_t)(d + 0.5f));
}
static void applyCc() {
  if (fault) return;
  setCcDuty(swState ? ccAmpsToDuty(iSet) : 0);     // OFF = lowest limit (~0.1 A)
}
static void spTouch() { spDirty = true; tSpChange = millis(); }
static void setV(float v) {
  float lo = cvDutyToVolts(1023) + 0.1f, hi = cfg.vCeil;
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  v = floor(v * 100.0f + 0.5f) / 100.0f;
  if (fabs(v - vSet) < 0.001f) return;
  if (fabs(v - vSet) > 2.0f) { trim = 0; trimGood = 0; }
  vSet = v;
  tSetChange = millis();
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
  applyCc();                           // now the real limit
  tSwOn = millis();
  tSetChange = millis();               // let the output settle before trimming
  beep(1, 60, 0);
  return true;
}
static void psuOff() {
  if (!swState) return;
  openSwitch();
  trim = trimGood;                     // drop any wind-up from the loaded period
  applyCv();
  applyCc();                           // back to the lowest limit
  beep(2, 60, 80);
}
static void safeOutputs() {
  openSwitch();
  setCvDuty(1023);
  setCcDuty(0);
}
static void trip(const char* msg) {
  if (fault) return;
  fault = true;
  faultMsg = msg;
  safeOutputs();
  vEst = cfg.vCeil;                    // assume the worst after a fault
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
  analogRead(pin);                        // settle mux
  uint32_t s = 0;
  for (uint8_t i = 0; i < n; i++) s += analogRead(pin);
  return (float)s / n;
}
static float ntcTemp(float counts) {
  if (counts > 1018.0f || counts < 3.0f) return NAN;   // open / shorted
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

  ibat = (fIc - iZero) * cfg.iGain * I_SIGN;
  float vnode = fVc * VREF / 1023.0f;
  vbat = vnode * cfg.kClosed - (swState ? ibat * R_SERIES : 0.0f);

  tBat  = ntcTemp(adcAvg(PIN_NTC_BAT, 8));
  tHeat = ntcTemp(adcAvg(PIN_NTC_HEAT, 8));
  tInt  = ntcTemp(adcAvg(PIN_NTC_INT, 8));

  if (!fault) {
    if (checksOn && vbat > V_FAULT) trip("OVERVOLT");
    else if (checksOn && fabs(ibat) > I_FAULT) trip("OVERCURRENT");
    else if (!isnan(tBat)  && tBat  > T_BAT_MAX)  trip("BATTERY HOT");
    else if (!isnan(tHeat) && tHeat > T_HEAT_MAX) trip("HEATSINK HOT");
    else if (!isnan(tInt)  && tInt  > T_INT_MAX)  trip("INSIDE HOT");
  }
}
// what we know about the module output voltage (used to allow switching ON)
static void estimateOutput(float dt) {
  if (checksOn) { vEst = vbat; return; }
  float tgt = cvDutyToVolts(cvDuty);
  if (vEst > tgt) vEst = tgt + (vEst - tgt) * exp(-dt * 1000.0f / OUT_TAU_MS);
  else            vEst = tgt;
}
static bool ccLatch = false;
static bool inCurrentLimit() {
  if (!swState || !checksOn) { ccLatch = false; return false; }
  // hysteresis: enter CC at 0.92 x Iset, release below 0.80 x Iset.
  // Without it the CV and CC controllers flap at the boundary (~2 Hz).
  if (!ccLatch && ibat > iSet * 0.92f) ccLatch = true;
  else if (ccLatch && ibat < iSet * 0.80f) ccLatch = false;
  return ccLatch;
}
// 2 Hz: slow correction of the output voltage from the measured value
static void trimTask() {
  float vNow = vbat;
  bool stationary = fabs(vNow - vLast) < 0.04f;
  vLast = vNow;
  if ((!swState && !TRIM_WHEN_OFF) || fault || !trimEnabled || !checksOn) return;
  if (millis() - tSetChange < 3000UL) return;           // let it settle first
  if (vNow < 1.0f) return;
  float err = vSet - vNow;                              // > 0: output too low
  bool loaded = swState && ibat > 0.3f;
  // In current limit the analog CC loop owns the FB node. The CV demand
  // follows the measured output + a small margin (never above vSet): the CC
  // loop gets minimal injection stress, and when the limit releases the
  // demand is already ~the measured output, so the voltage can never jump
  // above the set value (v0.5.2).
  if (inCurrentLimit()) {
    float cvT = vbat + CC_CV_MARGIN;
    if (cvT > vSet) cvT = vSet;
    if (cvT < 1.0f) cvT = 1.0f;
    float t = (float)cvVoltsToDuty(cvT) - (float)cvVoltsToDuty(vSet);
    if (t > 800.0f) t = 800.0f;
    if (t < -TRIM_MAX) t = -TRIM_MAX;
    // track directly (not by slow 4-count steps): the CV chain RC filter
    // smooths the demand, and the analog servo holds the current anyway.
    // Slew-limited stepping here made the limit settle for ~a minute and
    // left trim parked far from zero, so releasing CC recovered for ~1 min.
    trim = t;
    applyCv();
    return;
  }
  if (loaded && err > TRIM_LIMIT_ERR) {
    // module current limit (or overload), not a CV error: never wind up, relax to the last good value.
    // Proportional approach (cap 6 counts/tick): after CC the trim can be parked hundreds of
    // counts away, and a fixed 2-count step took ~1 min to hand the output back to CV.
    float d = (trimGood - trim) * 0.10f;
    if (d > 6.0f) d = 6.0f;
    if (d < -6.0f) d = -6.0f;
    if (d != 0.0f) { trim += d; applyCv(); }
    return;
  }
  if (inCurrentLimit() || !stationary) return;
  if (err > 2.5f) return;                               // beyond trim authority
  float step;
  if (err > 0.03f) {
    step = -err * 40.0f;                                // fewer counts = higher voltage
    if (step < -20.0f) step = -20.0f;
  } else if (err < -0.06f) {
    // too high: without a load the output only falls slowly, do not wind up
    float maxUp = loaded ? 4.0f : 3.0f;
    step = -err * 40.0f;
    if (step > maxUp) step = maxUp;
  } else {
    trimGood = trim;                                    // in the dead band: remember this value
    return;
  }
  trim += step;
  if (trim > TRIM_MAX) trim = TRIM_MAX;
  if (trim < -TRIM_MAX) trim = -TRIM_MAX;
  applyCv();
}
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

// ---------------- formatting ----------------
static const char* f2(float v, uint8_t w, uint8_t p) {
  static char b[4][14];
  static uint8_t i = 0;
  i = (i + 1) & 3;
  dtostrf(v, w, p, b[i]);
  return b[i];
}
static const char* fT(float t) {            // temperature or "--"
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
  snprintf(b, sizeof b, "CC %4u ~%sA", ccDuty, f2(ccDutyToAmps(ccDuty), 1, 1));
  u8g2.drawStr(0, 33, b);
  snprintf(b, sizeof b, "Tb%s Th%s Ti%s", fT(tBat), fT(tHeat), fT(tInt));
  u8g2.drawStr(0, 45, b);
  uint16_t tk, bd;
  noInterrupts(); tk = encTicks; bd = encBad; interrupts();
  snprintf(b, sizeof b, "%s %03ux%02u", FW_VERSION, (unsigned)(tk % 1000), (unsigned)(bd % 100));
  u8g2.drawStr(0, 57, b);
}
static void draw() {
  u8g2.clearBuffer();
  if (uiField == 2) drawInfo(); else drawMain();
  if (fault) {                              // inverted bar, visible on every screen
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
  Serial.print(F(" trim=")); Serial.print(trim, 1);
  Serial.print(F(" tg=")); Serial.print(trimGood, 1);
  Serial.print(F(" cc=")); Serial.print(ccDuty);
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
  Serial.println(F("clr zero faults 0/1 | cal vc V | cal i A | cal lim A | ceil v V | ceil i A | ceil i0 A | imax [A] | cc duty | save | defaults"));
}
static void execCmd(char* s) {
  char* cmd = strtok(s, " ");
  char* a1 = strtok(NULL, " ");
  char* a2 = strtok(NULL, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) { printHelp(); return; }
  if (!strcmp(cmd, "st")) { printStatus(); return; }
  if (!strcmp(cmd, "log")) { logOn = a1 && atoi(a1); Serial.println(logOn ? F("log on") : F("log off")); return; }
  if (!strcmp(cmd, "off")) { psuOff(); printStatus(); return; }
  if (!strcmp(cmd, "kill")) { safeOutputs(); Serial.println(F("output off, PWM at safe values (v/i or clr restores)")); return; }
  if (!strcmp(cmd, "on")) { psuOn(a1 && !strcmp(a1, "force")); printStatus(); return; }
  if (!strcmp(cmd, "clr")) { clearFault(); return; }
  if (!strcmp(cmd, "zero")) {
    if (swState) { Serial.println(F("refused: switch the output off first")); return; }
    measureZero(); Serial.print(F("zero=")); Serial.println(iZero, 1); return;
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
  if (!strcmp(cmd, "defaults")) { cfgDefaults(); trim = 0; trimGood = 0; if (iSet > iLimit()) iSet = iLimit(); Serial.println(F("defaults loaded (not saved)")); applyCv(); applyCc(); return; }

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

  if (!strcmp(cmd, "cal") && a1 && a2) {
    float val = atof(a2);
    float vnode = fVc * VREF / 1023.0f;
    if (!strcmp(a1, "vc")) {
      if (vnode < 0.05f) { Serial.println(F("no voltage on the divider")); return; }
      cfg.kClosed = (val + (swState ? ibat * R_SERIES : 0.0f)) / vnode;
      Serial.print(F("kClosed=")); Serial.println(cfg.kClosed, 4);
    } else if (!strcmp(a1, "i")) {
      float dc = (fIc - iZero) * I_SIGN;
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
    { long dm = ccAmpsToDuty(iLimit()); if (d > dm) d = dm; }   // raw duty is capped by imax too
    setCcDuty((uint16_t)d);
    Serial.print(F("raw CC duty=")); Serial.print(ccDuty);
    Serial.print(F(" (~")); Serial.print(ccDutyToAmps(ccDuty), 2); Serial.println(F(" A by the map)"));
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
    if (fault) { clearFault(); beep(1, 100, 0); }
    else if (swState) psuOff();
    else psuOn(false);
  }
  if (p) uiField = (uiField + 1) % 3;
  if (d) {
    if (uiField == 0)      setV(vSet + d * V_STEP);
    else if (uiField == 1) setI(iSet + d * I_STEP);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  // outputs to a safe state first
  pinMode(PIN_SWITCH, OUTPUT); digitalWrite(PIN_SWITCH, LOW);
  pinMode(PIN_CV_PWM, OUTPUT); digitalWrite(PIN_CV_PWM, LOW);
  pinMode(PIN_CC_PWM, OUTPUT); digitalWrite(PIN_CC_PWM, LOW);
  pinMode(PIN_FAN, OUTPUT);    digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BEEP, OUTPUT);   digitalWrite(PIN_BEEP, LOW);
  pwmInit();
  setCvDuty(1023);                // minimum voltage until the settings are loaded
  setCcDuty(0);                   // minimum current limit

  Serial.begin(115200);
  cfgLoad();
  spLoad();

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40);   // CTC (TOP = OCR4A), /64
  OCR4A = 249;                                    // 1 kHz period
  OCR4B = 100;                                    // compare-B fires once per period
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
  measureZero();                  // output is OFF: no current through the sensor

  vEst = cfg.vCeil;               // module may sit at the ceiling right after power-up
  applyCv();                      // pre-set the module to the stored voltage (output stays OFF)
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
    updateFan();
  }
  if (now - tTrim >= 500) {
    tTrim = now;
    trimTask();
  }
  if (spDirty && now - tSpChange > 5000UL) spSaveNow();
  if (now - tDraw >= 150) {
    tDraw = now;
    draw();
  }
  if (logOn && now - tLog >= 1000) {
    tLog = now;
    printStatus();
  }
}