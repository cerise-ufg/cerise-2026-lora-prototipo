"""Figuras do protocolo: formato do quadro (bytes) e diagrama de sequencia node <-> gateway.

Saidas: evidencias/fig_formato_quadro.png, evidencias/fig_sequencia.png
"""
from pathlib import Path

import cerise_proto as cp
import plot_style as ps

ps.apply()
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "evidencias"


def fields_row(ax, y, fields, x0=0.0, unit=1.0, h=0.8, title=None):
    x = x0
    for name, size, col in fields:
        ax.add_patch(Rectangle((x, y), size * unit, h, fc=col, ec="white", lw=2))
        ax.text(x + size * unit / 2, y + h / 2 + 0.12, name, ha="center", va="center",
                fontsize=7.3 if size > 1 else 6.4, color="white", fontweight="bold")
        ax.text(x + size * unit / 2, y + h / 2 - 0.2, f"{size} B", ha="center", va="center", fontsize=6.8,
                color="white")
        x += size * unit
    if title:
        ax.text(x0 - 0.3, y + h / 2, title, ha="right", va="center", fontsize=8.5, color=ps.INK, fontweight="bold")
    return x


def frame_format():
    fig, ax = plt.subplots(figsize=(12.5, 8.6))
    ax.set_xlim(-7.5, 37)
    ax.set_ylim(-4.6, 9.6)
    U = 1.3
    ax.axis("off")
    H, P, C = ps.INK2, ps.CAT[0], ps.CAT[1]
    hdr = [("v|tp", 1, H), ("flags", 1, H), ("net", 1, H), ("src", 1, H), ("dst", 1, H), ("seq", 2, H),
           ("len", 1, H)]
    y = 8.0
    x = fields_row(ax, y, hdr, unit=U, title="Cabeçalho (8 B)")
    x = fields_row(ax, y, [("payload (0–32 B, layout fixo por tipo)", 14, P)], x0=x, unit=U)
    fields_row(ax, y, [("CRC16", 2, C)], x0=x, unit=U)
    ax.text(0, y + 1.05, "Quadro cerise_proto v1 — campos multibyte em little-endian; CRC-16/CCITT-FALSE sobre cabeçalho + payload",
            fontsize=9, color=ps.INK)
    rows = [
        ("POS (14 B)", [("lat·1e7", 4, P), ("lon·1e7", 4, P), ("alt dm", 2, P), ("bat%", 1, P), ("pessoas", 2, P),
                        ("área", 1, P)]),
        ("HEARTBEAT (8 B)", [("bat%", 1, P), ("bat mV", 2, P), ("est.", 1, P), ("uptime s", 4, P)]),
        ("AREA_ASSIGN (22 B)", [("área", 1, P), ("tar.", 1, P), ("lat_min", 4, P), ("lon_min", 4, P),
                                ("lat_max", 4, P), ("lon_max", 4, P), ("alt dm", 2, P), ("esp. dm", 2, P)]),
        ("COVERAGE_DONE (7 B)", [("área", 1, P), ("tar.", 1, P), ("cob%", 1, P), ("duração", 2, P),
                                 ("pessoas", 2, P)]),
        ("COUNT_REPORT (13 B)", [("área", 1, P), ("tar.", 1, P), ("pessoas", 2, P), ("conf", 1, P),
                                 ("lat·1e7", 4, P), ("lon·1e7", 4, P)]),
        ("ACK (6 B)", [("seq conf.", 2, P), ("tipo", 1, P), ("stat", 1, P), ("−RSSI", 1, P), ("SNR·4", 1, P)]),
    ]
    for k, (name, flds) in enumerate(rows):
        yy = 6.4 - k * 1.1
        fields_row(ax, yy, flds, x0=7.0, unit=U, h=0.8, title=name)
    # tabela de tamanhos/tempo no ar
    tx, ty = 7.0, -0.2
    ax.text(tx, ty, "Quadro total e tempo no ar em ms (BW 125 kHz, CR 4/5, preâmbulo 8)", fontsize=8.5,
            fontweight="bold")
    ax.text(tx, ty - 0.45, f"{'tipo':<14}{'B':>3}{'SF7':>7}{'SF9':>7}{'SF10':>7}", fontsize=8, family="monospace")
    for k, (nm, t) in enumerate([("POS", cp.POS), ("HEARTBEAT", cp.HEARTBEAT), ("AREA_ASSIGN", cp.AREA_ASSIGN),
                                 ("COVERAGE_DONE", cp.COVERAGE_DONE), ("COUNT_REPORT", cp.COUNT_REPORT),
                                 ("ACK", cp.ACK)]):
        fl = cp.frame_len(t)
        vals = [cp.airtime_us(sf, payload=fl) / 1000 for sf in (7, 9, 10)]
        txt = f"{nm:<14}{fl:>3}" + "".join(f"{v:>7.1f}" for v in vals)
        ax.text(tx, ty - 0.9 - k * 0.42, txt, fontsize=8, family="monospace",
                color=ps.CAT[7] if vals[2] > 400 else ps.INK)
    ax.text(tx + 17, ty - 1.2, "em vermelho: tempo no ar > 400 ms", fontsize=7.5, color=ps.INK2)
    ax.text(tx + 17, ty - 2.0, "flags: 0x01 ACK_REQ · 0x02 RETRY\n0x04 DL_PENDING · 0x08 TESTE", fontsize=7.5,
            color=ps.INK2)
    ax.text(tx + 17, ty - 3.1, "extensão proposta pela frente de drones (aditiva, v1):\n7 AREA_POLY (4 + 4n B, n ≤ 7 vértices)\n8 TASK_BID (4 B) · 9 TASK_AWARD (3 B)",
            fontsize=7.5, color=ps.INK2)
    ps.save(fig, OUT / "fig_formato_quadro.png")


def sequence():
    fig, ax = plt.subplots(figsize=(10.5, 7.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 13.2)
    ax.axis("off")
    xn, xg = 2.0, 8.0
    for x, lab, col in ((xn, "NODE (drone)", ps.CAT[1]), (xg, "GATEWAY (solo)", ps.CAT[0])):
        ax.add_patch(Rectangle((x - 1.1, 12.3), 2.2, 0.7, fc=col, ec=col))
        ax.text(x, 12.65, lab, ha="center", va="center", color="white", fontweight="bold", fontsize=9)
        ax.plot([x, x], [0.3, 12.3], color=ps.INK2, lw=1, ls=":")

    def msg(y0, y1, left_to_right, label, col=ps.INK, style="-", lost=False):
        a, b = (xn, xg) if left_to_right else (xg, xn)
        if lost:
            bx = a + (b - a) * 0.62
            ax.add_patch(FancyArrowPatch((a, y0), (bx, y0 - (y0 - y1) * 0.62), arrowstyle="-", lw=1.6, color=col,
                                         ls=style))
            ax.text(bx, y0 - (y0 - y1) * 0.62, "✕", ha="center", va="center", fontsize=14, color=ps.CAT[7])
        else:
            ax.add_patch(FancyArrowPatch((a, y0), (b, y1), arrowstyle="-|>", mutation_scale=12, lw=1.6, color=col,
                                         ls=style))
        ax.text((a + b) / 2, (y0 + y1) / 2 + 0.18, label, ha="center", va="bottom", fontsize=8, color=ps.INK,
                bbox=dict(boxstyle="round,pad=0.15", fc=ps.SURFACE, ec="none"))

    def note(x, y, txt, ha="left"):
        ax.text(x, y, txt, ha=ha, va="center", fontsize=7.4, color=ps.INK2, style="italic")

    note(xn - 0.25, 11.9, "CAD: canal livre?", ha="right")
    msg(11.7, 11.1, True, "POS seq=120 [ACK_REQ]  (24 B, 205,8 ms @SF9)", col=ps.CAT[1])
    note(xg + 0.25, 11.0, "CRC ok · dedup: novo\nJSON na serial")
    msg(10.6, 10.0, False, "ACK seq_conf=120 · RSSI/SNR medidos", col=ps.CAT[0])
    note(xn - 0.25, 9.9, "latência, RSSI do gw\npróximo ciclo em 5 s ±10 %", ha="right")
    msg(9.0, 8.4, True, "POS seq=121 [ACK_REQ]", col=ps.CAT[1], lost=True)
    note(xn - 0.25, 7.9, "timeout = ToA(ACK)+10+150 ms\nrecuo U(0,(ToA+espera)·2^k)", ha="right")
    msg(7.3, 6.7, True, "POS seq=121 [ACK_REQ|RETRY]", col=ps.CAT[1])
    msg(6.4, 5.8, False, "ACK seq_conf=121 [DL_PENDING]", col=ps.CAT[0])
    note(xg + 0.25, 6.6, "fila de downlink\npara o drone 1")
    note(xn - 0.25, 5.6, "abre janela de RX", ha="right")
    msg(5.2, 4.6, False, "AREA_ASSIGN área=3 tarefa=7 [ACK_REQ] (32 B)", col=ps.CAT[0])
    note(xn - 0.25, 4.3, "aplica nova área\n(planejamento de cobertura)", ha="right")
    msg(4.0, 3.4, True, "ACK seq_conf=AREA_ASSIGN", col=ps.CAT[1])
    note(xg + 0.25, 3.3, "downlink confirmado")
    msg(2.6, 2.0, True, "COUNT_REPORT / COVERAGE_DONE [ACK_REQ]", col=ps.CAT[1])
    msg(1.6, 1.0, False, "ACK", col=ps.CAT[0])
    ax.set_title("Troca de mensagens do protótipo (ACK, retentativa, downlink sob demanda)", fontsize=11)
    ps.save(fig, OUT / "fig_sequencia.png")


if __name__ == "__main__":
    frame_format()
    sequence()
