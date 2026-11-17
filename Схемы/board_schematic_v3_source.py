# -*- coding: utf-8 -*-
"""Схема платы обвязки V3 (на базе V2.2). Чистый CAD-стиль, координаты в мм, холст 200x140."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

WIRE = "#1a1a1a"
BOX_F, BOX_E = "#b7dcd9", "#2b6b68"
RES_F = "#f2f2f2"
GND_C = "#2e8b57"
RED = "#c0392b"
TXT = "#111111"

fig = plt.figure(figsize=(13.33, 9.33), dpi=225)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 200); ax.set_ylim(0, 140)
ax.set_aspect("equal"); ax.axis("off")

def wire(*pts):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=WIRE, lw=1.6,
            solid_capstyle="round", zorder=2)

def dot(x, y):
    ax.plot([x], [y], marker="o", ms=4.5, color=WIRE, zorder=5)

def txt(x, y, s, size=3.2, color=TXT, ha="left", va="center", weight="normal", rot=0):
    ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va, rotation=rot,
            family="DejaVu Sans", weight=weight, zorder=6)

def res_h(x1, x2, y, label=None, below=False):
    ax.add_patch(Rectangle((x1, y - 1.6), x2 - x1, 3.2, facecolor=RES_F,
                           edgecolor=WIRE, lw=1.4, zorder=3))
    if label:
        txt((x1 + x2) / 2, y - 4.4 if below else y + 3.8, label, ha="center")

def res_v(x, y1, y2, label=None, side="right"):
    ax.add_patch(Rectangle((x - 1.6, y1), 3.2, y2 - y1, facecolor=RES_F,
                           edgecolor=WIRE, lw=1.4, zorder=3))
    if label:
        dx = 2.8 if side == "right" else -2.8
        txt(x + dx, (y1 + y2) / 2, label, ha="left" if side == "right" else "right")

def cap_v(x, ytop, ybot, label=None, plus=False):
    pt, pb = ytop - 2.0, ytop - 4.2
    wire((x, ytop), (x, pt))
    ax.plot([x - 3, x + 3], [pt, pt], color=WIRE, lw=1.6, zorder=3)
    ax.plot([x - 3, x + 3], [pb, pb], color=WIRE, lw=1.6, zorder=3)
    wire((x, pb), (x, ybot))
    if label: txt(x + 4.2, pt - 1.1, label)
    if plus: txt(x - 5.8, pt + 0.6, "+", color=RED)

def diode_v_up(x, ybar, label=None):
    ax.plot([x - 2.5, x + 2.5], [ybar, ybar], color=WIRE, lw=2.0, zorder=3)
    ax.add_patch(Polygon([(x - 2.5, ybar - 5.2), (x + 2.5, ybar - 5.2), (x, ybar)],
                         closed=True, facecolor=RES_F, edgecolor=WIRE, lw=1.4, zorder=3))
    if label: txt(x + 4.2, ybar - 2.6, label)

def diode_h_right(xa, xc, y, label=None):
    ax.add_patch(Polygon([(xa, y - 2.5), (xa, y + 2.5), (xc, y)],
                         closed=True, facecolor=RES_F, edgecolor=WIRE, lw=1.4, zorder=3))
    ax.plot([xc, xc], [y - 2.5, y + 2.5], color=WIRE, lw=2.0, zorder=3)
    if label: txt((xa + xc) / 2, y + 4.4, label, ha="center")

def gnd(x, y):
    ax.plot([x, x], [y, y - 1.0], color=GND_C, lw=1.6, zorder=3)
    for dy, w in [(0, 8), (-1.2, 5), (-2.3, 2.5)]:
        ax.plot([x - w / 2, x + w / 2], [y - 1.0 + dy] * 2, color=GND_C, lw=1.6, zorder=3)

def conn(x1, x2, y1, y2, pins, names=None, stub="down"):
    ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, facecolor=BOX_F,
                           edgecolor=BOX_E, lw=1.6, zorder=4))
    for i, px in enumerate(pins):
        if stub == "down": wire((px, y1), (px, y1 - 3))
        else:              wire((px, y2), (px, y2 + 3))
        if names: txt(px, (y1 + y2) / 2, names[i], ha="center", size=3.0)

def cross(x, y, color=RED):
    ax.plot([x - 1.6, x + 1.6], [y - 1.6, y + 1.6], color=color, lw=1.6, zorder=5)
    ax.plot([x - 1.6, x + 1.6], [y + 1.6, y - 1.6], color=color, lw=1.6, zorder=5)

# ================= ЗАГОЛОВОК =================
txt(100, 135, "Плата обвязки — V3", size=6.5, ha="center", weight="bold")

# ================= J1: к CC модулю =================
txt(27, 124, "J1 · к CC модулю", size=3.6, ha="center", weight="bold")
conn(12, 42, 108, 120, [17, 25, 33], ["3", "2", "1 IN−"], stub="down")
cross(17, 99); txt(17, 95.5, "не исп.", color=RED, size=2.9, ha="center")
wire((33, 105), (33, 100)); gnd(33, 100)
wire((25, 105), (25, 90)); dot(25, 90)
txt(28.5, 92.8, "W")

# делитель уставки CC
wire((25, 90), (25, 82)); res_v(25, 74, 82, "2,36 кОм", side="left")
wire((25, 74), (25, 68)); res_v(25, 60, 68, "464 Ом", side="left")
wire((25, 60), (25, 56)); gnd(25, 56)
wire((25, 90), (33, 90)); wire((33, 90), (33, 77))
cap_v(33, 77, 70, "10 мкФ", plus=True); gnd(33, 70)

# 123,8 кОм -> D44 -> J4
wire((25, 90), (46, 90)); res_h(46, 62, 90, "123,8 кОм", below=True)
wire((62, 90), (74, 90)); wire((74, 90), (74, 60))
wire((74, 60), (27, 60)); wire((27, 60), (27, 25))

txt(48, 97.0, "полная шкала на J1-2 ≈ 0,11 В (ШИМ 1023)", size=3.0)
txt(48, 93.8, "потолок тока ≈ 11 А", size=3.0)
txt(48, 84.8, "D44: ШИМ 0…5 В,", size=3.0)
txt(48, 81.5, "выше скважность = выше предел тока", size=3.0)

# ================= J2: выход =================
txt(106, 124, "J2 · выход", size=3.6, ha="center", weight="bold")
conn(92, 120, 108, 120, [98, 112], ["OUT−", "BATT+"], stub="down")
wire((98, 105), (98, 100))
txt(46, 102.5, "J2 OUT−: к ключу —", size=3.0)
txt(46, 99.2, "на клемму минуса (после Q2)", size=3.0)
wire((112, 105), (112, 100)); dot(112, 100)
wire((112, 100), (112, 92)); res_v(112, 84, 92, "1 кОм", side="left")
wire((112, 84), (112, 82)); gnd(112, 82)
wire((112, 104), (118, 104)); dot(112, 104)
wire((118, 104), (118, 92)); res_v(118, 84, 92, "120 кОм")
wire((118, 84), (118, 76))
# узел делителя (шина y=76)
wire((80, 76), (134, 76))
dot(80, 76); dot(112, 76); dot(118, 76); dot(124, 76); dot(134, 76)
wire((112, 76), (112, 68)); res_v(112, 60, 68, "22 кОм")
wire((112, 60), (112, 58)); gnd(112, 58)
wire((124, 76), (124, 75)); cap_v(124, 75, 66, "100 нФ"); gnd(124, 66)
wire((134, 76), (134, 75)); diode_v_up(134, 75, "1N4007")
wire((134, 69.8), (134, 64)); gnd(134, 64)
wire((80, 76), (80, 25))                      # A5 вниз к J5

# ================= J3: FB + затворы +5V =================
txt(167, 124, "J3 · FB + затворы +5V", size=3.6, ha="center", weight="bold")
conn(148, 186, 108, 120, [154, 166], ["FB", "затв."], stub="down")
txt(181, 114, "+5V", ha="center", size=3.0)
wire((154, 105), (154, 98))                   # FB к цепи CV
wire((166, 105), (166, 100)); res_v(166, 92, 100, "100 Ом", side="left")
wire((166, 92), (166, 28)); wire((166, 28), (173, 28)); wire((173, 28), (173, 25))
# +5V: от J4 по нижней и правой кромке, вход сверху в J3
wire((19, 25), (12, 25)); wire((12, 25), (12, 8.5)); wire((12, 8.5), (193, 8.5))
wire((193, 8.5), (193, 127)); wire((193, 127), (181, 127))
wire((181, 127), (181, 120)); dot(181, 120)
txt(195.2, 70, "+5V", rot=90, size=3.0, ha="center")

# ================= цепь CV =================
wire((158, 25), (158, 34)); wire((158, 34), (88, 34))
wire((88, 34), (88, 50)); wire((88, 50), (92, 50))
res_h(92, 106, 50, "470 Ом")
wire((106, 50), (110, 50)); dot(110, 50)
txt(107.5, 54.8, "A")
cap_v(110, 50, 41, "100 мкФ", plus=True); gnd(110, 41)
wire((110, 50), (115, 50)); wire((115, 50), (115, 46))
res_v(115, 38, 46, "100 кОм"); wire((115, 38), (115, 36)); gnd(115, 36)
wire((110, 50), (120, 50)); res_h(120, 134, 50, "560 Ом")
wire((134, 50), (138, 50)); diode_h_right(138, 144, 50, "1N4007")
wire((144, 50), (148, 50)); wire((148, 50), (148, 86))
wire((148, 86), (154, 86)); wire((154, 86), (154, 98))

# ================= нижние разъёмы =================
txt(30, 11.5, "J4 · к MKS Gen", size=3.4, ha="center", weight="bold")
conn(14, 46, 14, 22, [19, 27, 35], ["+5V", "D44", "GND"], stub="up")
wire((35, 25), (35, 28)); gnd(35, 28)

txt(86, 11.5, "J5 · к MKS (A5)", size=3.4, ha="center", weight="bold")
conn(72, 100, 14, 22, [80, 92], ["A5", "—"], stub="up")
cross(92, 28)

txt(166, 11.5, "J6 · к MKS D40", size=3.4, ha="center", weight="bold")
conn(152, 180, 14, 22, [158, 173], ["D5", "D40"], stub="up")

# ================= блок изменений V3 =================
txt(46, 40, "Изменения V3:", size=3.2, weight="bold")
txt(46, 37.2, "• J4 → левее, 3 пина:", size=3.0)
txt(48, 34.4, "+5V / D44 / GND", size=3.0)
txt(46, 31.6, "• J5 посередине: A5", size=3.0)
txt(46, 28.8, "• J3: пин 3 = +5V от J4", size=3.0)
txt(46, 26.0, "• старый J5 → J6:", size=3.0)
txt(48, 23.2, "D5 | D40", size=3.0)

txt(100, 4.5, "Все знаки земли — общая GND платы · V3", size=3.0, ha="center")

fig.savefig(r"C:\Users\Techn\Documents\kimi\tasks\2026-11-16\13-20-02-0e78e1db\board_schematic_v3_draft.png")
print("ok")
