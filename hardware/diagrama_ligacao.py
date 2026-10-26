"""Gera o diagrama de ligacao (prototipo de bancada e modulo definitivo no drone).

Saida: evidencias/fig_diagrama_ligacao.png
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import plot_style as ps  # noqa: E402

ps.apply()
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

BLUE, ORANGE, AQUA, YELLOW, MAGENTA = ps.CAT[0], ps.CAT[1], ps.CAT[2], ps.CAT[3], ps.CAT[4]


def box(ax, x, y, w, h, title, lines=(), color=BLUE, fs=8.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc="white",
                                ec=color, lw=2.0))
    ax.add_patch(FancyBboxPatch((x, y + h - 0.42), w, 0.42, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=color, ec=color, lw=2.0))
    ax.text(x + w / 2, y + h - 0.21, title, ha="center", va="center", color="white", fontsize=fs,
            fontweight="bold")
    for i, ln in enumerate(lines):
        ax.text(x + 0.12, y + h - 0.68 - i * 0.30, ln, ha="left", va="center", fontsize=7.4, color=ps.INK)


def wire(ax, p0, p1, label="", color=ps.INK2, style="-", rad=0.0, lpos=0.5, dy=0.13, arrow="-"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=arrow, mutation_scale=10, lw=1.6, color=color, ls=style,
                                 connectionstyle=f"arc3,rad={rad}"))
    if label:
        x = p0[0] + (p1[0] - p0[0]) * lpos
        y = p0[1] + (p1[1] - p0[1]) * lpos + dy
        ax.text(x, y, label, ha="center", va="bottom", fontsize=7.0, color=ps.INK,
                bbox=dict(boxstyle="round,pad=0.15", fc=ps.SURFACE, ec="none"))


def antenna(ax, x, y, label, color=ps.INK, side="left"):
    ax.plot([x, x], [y, y + 0.55], color=color, lw=2)
    ax.plot([x - 0.18, x, x + 0.18], [y + 0.75, y + 0.55, y + 0.75], color=color, lw=2)
    dx = -0.3 if side == "left" else 0.3
    ax.text(x + dx, y + 0.35, label, ha="right" if side == "left" else "left", va="center", fontsize=7.2,
            color=ps.INK)


fig, axs = plt.subplots(2, 1, figsize=(12, 10.2))
for ax in axs:
    ax.set_xlim(0, 16)
    ax.set_aspect("equal")
    ax.axis("off")

# ------------------------------------------------------------------ (a) bancada
ax = axs[0]
ax.set_ylim(0, 6.3)
ax.set_title("(a) Protótipo de bancada / teste de alcance — 2 × Heltec WiFi LoRa 32 V3", loc="left")
box(ax, 0.2, 1.6, 2.6, 2.4, "Notebook (solo)",
    ["gateway_dashboard.py", "linhas JSON → CSV/PNG", "pio device monitor", "porta COMx 115200"], color=ps.INK2)
box(ax, 4.0, 1.2, 3.6, 3.2, "GATEWAY — Heltec V3",
    ["ESP32-S3FN8 (240 MHz)", "SX1262 + TCXO 1,8 V", "OLED SSD1306 128×64", "NSS8 SCK9 MOSI10 MISO11",
     "RST12 BUSY13 DIO1 14", "firmware: heltec_v3_gateway"], color=BLUE)
box(ax, 9.0, 1.2, 3.6, 3.2, "NODE — Heltec V3",
    ["mesmo hardware do gateway", "firmware: heltec_v3_node(_gps)", "telemetria POS/HEARTBEAT", "ACK + retentativas + CAD",
     "GPIO47 RX / GPIO48 TX", "VBAT: GPIO1 (ctrl GPIO37)"], color=ORANGE)
box(ax, 13.4, 3.0, 2.4, 1.6, "GPS (opcional)", ["NEO-6M / M8N", "UART 9600 bps", "3V3 · GND"], color=AQUA)
box(ax, 13.4, 0.5, 2.4, 1.9, "Bateria LiPo 1S", ["3,7 V · 1000–2000 mAh", "conector JST 1,25 mm", "carga pela USB-C"],
    color=YELLOW)
wire(ax, (2.8, 2.8), (4.0, 2.8), "USB-C (5 V + dados)", arrow="<->")
wire(ax, (12.6, 3.7), (13.4, 3.7), "TX→47", lpos=0.5)
wire(ax, (12.6, 3.35), (13.4, 3.35), "", lpos=0.5)
ax.text(13.0, 3.05, "48→RX", ha="center", fontsize=7.0)
wire(ax, (12.6, 1.5), (13.4, 1.5), "VBAT/JST", lpos=0.5)
antenna(ax, 5.8, 4.4, "antena 915 MHz 3 dBi\n(U.FL→SMA, vertical)")
antenna(ax, 10.8, 4.4, "dipolo 915 MHz 2 dBi", side="right")
ax.add_patch(FancyArrowPatch((6.25, 5.15), (10.35, 5.15), arrowstyle="<->", mutation_scale=14, lw=1.8,
                             color=MAGENTA, ls="--"))
ax.text(8.3, 5.4, "LoRa 916,8 MHz (AU915 canal 8) · SF7–SF10 · BW 125 kHz · CR 4/5 · 14 dBm", ha="center",
        fontsize=7.8, color=ps.INK)
ax.text(8.3, 0.35, "Não ligar o rádio sem antena (risco ao PA do SX1262). Distância de teste: protocolo em hardware/protocolo_teste_alcance.md",
        ha="center", fontsize=7.2, color=ps.INK2, style="italic")

# ------------------------------------------------------------------ (b) modulo no drone
ax = axs[1]
ax.set_ylim(0, 6.6)
ax.set_title("(b) Módulo de comunicação definitivo no drone (projeto)", loc="left")
box(ax, 0.2, 3.6, 2.9, 2.3, "Bateria do drone", ["LiPo 4S/6S", "via PDB do frame"], color=YELLOW)
box(ax, 0.2, 0.6, 2.9, 2.4, "BEC 5 V / 2 A", ["regulador dedicado", "filtro LC opcional", "(isola ruído dos ESCs)"],
    color=YELLOW)
box(ax, 4.2, 3.3, 3.8, 2.8, "Computador de bordo", ["Raspberry Pi / Jetson", "planejamento de cobertura", "contagem de pessoas (visão)",
                                                      "MAVLink ↔ controladora"], color=ps.INK2)
box(ax, 4.2, 0.55, 3.8, 2.25, "Controladora de voo", ["Pixhawk / ArduPilot", "GPS + bússola do drone", "telemetria MAVLink"],
    color=ps.INK2)
box(ax, 9.3, 1.4, 3.9, 3.6, "MÓDULO LoRa CERISE", ["Heltec V3 em gabinete PETG", "ESP32-S3 + SX1262", "5 V pelo pino 5V/VIN",
                                                    "UART1: GPIO47 RX · GPIO48 TX", "3,3 V TTL · 115200 8N1",
                                                    "comandos: POS / COUNT / DONE", "log JSON espelhado na UART",
                                                    "OLED desligável em voo"], color=ORANGE)
box(ax, 13.8, 1.4, 2.0, 2.2, "Antena", ["dipolo 1/2 onda", "915 MHz, SMA", "vertical p/ baixo"], color=MAGENTA)
wire(ax, (1.65, 3.6), (1.65, 3.0), "", arrow="->")
ax.plot([3.1, 3.6, 3.6, 10.6], [1.2, 1.2, 0.12, 0.12], color=ps.INK2, lw=1.6)
wire(ax, (10.6, 0.12), (10.6, 1.4), "", arrow="->")
ax.text(7.0, 0.2, "5 V · GND (JST-GH 2 vias)", ha="center", va="bottom", fontsize=7.0, color=ps.INK,
        bbox=dict(boxstyle="round,pad=0.15", fc=ps.SURFACE, ec="none"))
wire(ax, (8.0, 4.4), (9.3, 4.4), "UART TTL", lpos=0.5, arrow="<->")
wire(ax, (6.1, 3.3), (6.1, 2.8), "", arrow="<->")
ax.text(6.25, 3.0, "MAVLink (UART/USB)", fontsize=7.0, va="center")
wire(ax, (13.2, 2.5), (13.8, 2.5), "SMA", lpos=0.5, arrow="-")
ax.text(14.8, 4.2, "≥ 10 cm do GPS e de\ncabos de ESC/motor;\nfora da sombra do\nframe de carbono", ha="center",
        fontsize=7.0, color=ps.INK2, va="bottom")
ax.text(8.3, 6.3, "Fluxo: drone envia POS/HEARTBEAT/COUNT_REPORT/COVERAGE_DONE → estação de solo; solo envia AREA_ASSIGN → drone (após ACK)",
        ha="center", fontsize=7.6, color=ps.INK)
out = ROOT / "evidencias" / "fig_diagrama_ligacao.png"
ps.save(fig, out)
