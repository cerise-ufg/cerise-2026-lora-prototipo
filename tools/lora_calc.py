"""Calculadora LoRa: tempo no ar (Semtech) + orcamento de enlace e alcance estimado.

Saidas (padrao): resultados/airtime_*.csv, resultados/enlace_*.csv, resultados/lora_calc.json
                 evidencias/fig_airtime.png, evidencias/fig_alcance_sf.png, evidencias/fig_alcance_altitude.png

Uso:  python tools/lora_calc.py [--ptx 14] [--alt 60] [--out-res resultados] [--out-fig evidencias]

Modelos de propagacao (f = 916,8 MHz):
  - espaco livre (Friis): FSPL = 20 log10(d_km) + 20 log10(f_MHz) + 32,44
  - log-distancia: PL(d) = FSPL(1 m) + 10 n log10(d / 1 m), n = 2,0 / 2,7 / 3,5
  - Okumura-Hata (cidade pequena/media, suburbano e area aberta), tratando o drone como
    a antena "alta" (hb = altitude, validade 30-200 m) e a estacao de solo como a "movel"
    (hm = 2 m); validade 150-1500 MHz e 1-20 km.
Sensibilidade teorica: S = -174 + 10 log10(BW) + NF + SNR_lim(SF), NF = 6 dB.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

import cerise_proto as cp
import plot_style as ps

F_MHZ = cp.au915_uplink_hz(8) / 1e6  # 916,8 MHz
BW = 125_000
SFS = list(range(7, 13))
SNR_LIM = {7: -7.5, 8: -10.0, 9: -12.5, 10: -15.0, 11: -17.5, 12: -20.0}  # Semtech
NF_DB = 6.0
DWELL_MS = 400.0
MSGS = [("HEARTBEAT", cp.HEARTBEAT), ("POS", cp.POS), ("AREA_ASSIGN", cp.AREA_ASSIGN),
        ("COVERAGE_DONE", cp.COVERAGE_DONE), ("COUNT_REPORT", cp.COUNT_REPORT), ("ACK", cp.ACK)]

# orcamento de enlace (valores de projeto, nao medidos)
LINK = {
    "gt_dbi": 2.0,       # dipolo 1/2 onda no drone
    "gr_dbi": 3.0,       # antena omni de solo (<= 6 dBi: ANATEL 10.5 sem reducao de potencia)
    "cable_db": 1.5,     # conectores + cabos (U.FL/SMA) nos dois lados
    "fade_db": 10.0,     # margem de desvanecimento / orientacao da antena
    "hm_m": 2.0,         # altura da antena da estacao de solo
}


def sensitivity_dbm(sf: int, bw: int = BW) -> float:
    return -174.0 + 10 * math.log10(bw) + NF_DB + SNR_LIM[sf]


def bitrate_bps(sf: int, bw: int = BW, cr_denom: int = 5) -> float:
    return sf * bw / (2 ** sf) * 4.0 / cr_denom


def fspl_db(d_km, f_mhz=F_MHZ):
    return 20 * np.log10(d_km) + 20 * math.log10(f_mhz) + 32.44


def range_fspl_km(mapl, f_mhz=F_MHZ):
    return 10 ** ((mapl - 20 * math.log10(f_mhz) - 32.44) / 20)


def range_logdist_km(mapl, n, f_mhz=F_MHZ):
    pl0 = float(fspl_db(0.001, f_mhz))  # FSPL a 1 m
    return 10 ** ((mapl - pl0) / (10 * n)) / 1000.0


def hata_urban_db(d_km, hb, hm, f=F_MHZ):
    a_hm = (1.1 * math.log10(f) - 0.7) * hm - (1.56 * math.log10(f) - 0.8)
    return 69.55 + 26.16 * math.log10(f) - 13.82 * math.log10(hb) - a_hm + (44.9 - 6.55 * math.log10(hb)) * np.log10(d_km)


def hata_db(d_km, hb, hm, env, f=F_MHZ):
    lu = hata_urban_db(d_km, hb, hm, f)
    if env == "urbano":
        return lu
    if env == "suburbano":
        return lu - 2 * (math.log10(f / 28)) ** 2 - 5.4
    if env == "aberto":
        return lu - 4.78 * (math.log10(f)) ** 2 + 18.33 * math.log10(f) - 40.94
    raise ValueError(env)


def range_hata_km(mapl, hb, hm, env, f=F_MHZ):
    # PL(d) = A + B log10(d)  ->  d = 10^((MAPL - A) / B)
    a = float(hata_db(1.0, hb, hm, env, f))
    b = 44.9 - 6.55 * math.log10(hb)
    return 10 ** ((mapl - a) / b)


def radio_horizon_km(h1, h2):
    return 4.12 * (math.sqrt(h1) + math.sqrt(h2))  # k = 4/3


def fresnel_mid_m(d_km, f_mhz=F_MHZ):
    return 17.32 * math.sqrt(d_km / (4 * f_mhz / 1000.0))


def mapl_db(ptx, sf):
    return ptx + LINK["gt_dbi"] + LINK["gr_dbi"] - LINK["cable_db"] - sensitivity_dbm(sf) - LINK["fade_db"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ptx", type=float, default=14.0, help="potencia de TX (dBm)")
    ap.add_argument("--alt", type=float, default=60.0, help="altitude do drone (m)")
    root = Path(__file__).resolve().parent.parent
    ap.add_argument("--out-res", default=str(root / "resultados"))
    ap.add_argument("--out-fig", default=str(root / "evidencias"))
    args = ap.parse_args()
    res, fig_dir = Path(args.out_res), Path(args.out_fig)
    res.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)
    ps.apply()
    import matplotlib.pyplot as plt

    summary = {"f_mhz": F_MHZ, "bw_hz": BW, "cr": "4/5", "preamble": 8, "link": LINK, "ptx_dbm": args.ptx,
               "alt_m": args.alt}

    # ---------------- 1) tempo no ar por mensagem e SF
    rows = []
    for name, t in MSGS:
        fl = cp.frame_len(t)
        row = {"mensagem": name, "quadro_B": fl}
        for sf in SFS:
            row[f"SF{sf}_ms"] = round(cp.airtime_us(sf, BW, 5, 8, fl) / 1000.0, 1)
        rows.append(row)
    with open(res / "airtime_mensagens.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    summary["airtime_ms"] = rows

    sf_rows = []
    for sf in SFS:
        maxpl = max([pl for pl in range(0, 256) if cp.airtime_us(sf, BW, 5, 8, pl) <= DWELL_MS * 1000] or [-1])
        toa_pos = cp.airtime_us(sf, BW, 5, 8, cp.frame_len(cp.POS)) / 1000.0
        sf_rows.append({
            "SF": sf, "taxa_bps": round(bitrate_bps(sf), 1), "sens_dBm": round(sensitivity_dbm(sf), 1),
            "snr_lim_dB": SNR_LIM[sf], "toa_POS_ms": round(toa_pos, 1),
            "max_quadro_400ms_B": maxpl, "POS_cabe_400ms": toa_pos <= DWELL_MS,
            "ocupacao_POS_5s_pct": round(100 * toa_pos / 5000.0, 2),
        })
    with open(res / "airtime_por_sf.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(sf_rows[0].keys()))
        w.writeheader()
        w.writerows(sf_rows)
    summary["por_sf"] = sf_rows

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    pls = np.arange(0, 65)
    for i, sf in enumerate(SFS):
        ax.plot(pls, [cp.airtime_us(sf, BW, 5, 8, int(p)) / 1000 for p in pls], color=ps.CAT[i], label=f"SF{sf}")
    ax.axhline(DWELL_MS, color=ps.INK, lw=1.2, ls="--")
    ax.text(63, DWELL_MS * 0.86, "limite de ocupação por quadro: 400 ms", fontsize=8, color=ps.INK, ha="right",
            va="top", bbox=dict(boxstyle="round,pad=0.2", fc=ps.SURFACE, ec="none"))
    for name, t in [("POS", cp.POS), ("ACK", cp.ACK), ("AREA_ASSIGN", cp.AREA_ASSIGN)]:
        x = cp.frame_len(t)
        ax.axvline(x, color=ps.INK2, lw=0.8, ls=":")
        ax.text(x + 0.4, 12, f"{name} ({x} B)", rotation=90, fontsize=7.5, color=ps.INK2, va="bottom")
    ax.set_yscale("log")
    ax.set_xlabel("Tamanho do quadro PHY (bytes)")
    ax.set_ylabel("Tempo no ar (ms, escala log)")
    ax.set_title("Tempo no ar LoRa — BW 125 kHz, CR 4/5, preâmbulo 8, CRC ligado")
    ax.legend(ncol=3, loc="upper left")
    ax.set_ylim(20, 6000)
    ps.save(fig, fig_dir / "fig_airtime.png")

    # ---------------- 2) orcamento de enlace e alcance
    envs = ["aberto", "suburbano", "urbano"]
    link_rows = []
    for ptx in (args.ptx, 22.0):
        for sf in SFS:
            m = mapl_db(ptx, sf)
            r = {"ptx_dBm": ptx, "SF": sf, "sens_dBm": round(sensitivity_dbm(sf), 1), "MAPL_dB": round(m, 1),
                 "d_espaco_livre_km": round(range_fspl_km(m), 1),
                 "d_logdist_n2.7_km": round(range_logdist_km(m, 2.7), 2),
                 "d_logdist_n3.5_km": round(range_logdist_km(m, 3.5), 2)}
            for env in envs:
                r[f"d_hata_{env}_km"] = round(range_hata_km(m, args.alt, LINK["hm_m"], env), 2)
            link_rows.append(r)
    with open(res / "enlace_alcance.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(link_rows[0].keys()))
        w.writeheader()
        w.writerows(link_rows)
    summary["enlace"] = link_rows
    summary["horizonte_radio_km"] = round(radio_horizon_km(args.alt, LINK["hm_m"]), 1)
    summary["fresnel_meio_m"] = {f"{d}km": round(fresnel_mid_m(d), 1) for d in (1, 2, 5, 10)}

    sel = [r for r in link_rows if r["ptx_dBm"] == args.ptx]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    series = [("Log-distância n=2,7", "d_logdist_n2.7_km"), ("Log-distância n=3,5", "d_logdist_n3.5_km"),
              ("Hata — área aberta", "d_hata_aberto_km"), ("Hata — suburbano", "d_hata_suburbano_km"),
              ("Hata — urbano", "d_hata_urbano_km")]
    for i, (lab, key) in enumerate(series):
        ys = [r[key] for r in sel]
        ax.plot(SFS, ys, marker="o", color=ps.CAT[i], label=lab)
        ax.annotate(f"{ys[-1]:.1f}", (12, ys[-1]), textcoords="offset points", xytext=(6, -3), fontsize=7.5,
                    color=ps.INK2)
    ax.set_yscale("log")
    ax.set_xlabel("Fator de espalhamento (SF)")
    ax.set_ylabel("Alcance estimado (km, escala log)")
    ax.set_title(f"Alcance estimado ar–solo: {args.ptx:.0f} dBm, drone a {args.alt:.0f} m, margem {LINK['fade_db']:.0f} dB")
    ax.axhspan(20, 200, color=ps.GRID, alpha=0.6, lw=0)
    ax.text(6.8, 45, "> 20 km: fora da faixa de validade de Okumura–Hata", fontsize=7.5, color=ps.INK2)
    ax.axvspan(10.5, 12.8, color=ps.GRID, alpha=0.35, lw=0)
    ax.text(10.6, 0.62, "POS > 400 ms no ar", fontsize=7.5, color=ps.INK2)
    ax.set_ylim(0.55, 60)
    ax.set_xticks(SFS)
    ax.set_xlim(6.7, 12.8)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3)
    ps.save(fig, fig_dir / "fig_alcance_sf.png")

    # alcance x altitude do drone (Hata suburbano), por SF
    alts = np.arange(30, 121, 5)
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    alt_rows = []
    for i, sf in enumerate([7, 9, 10, 12]):
        m = mapl_db(args.ptx, sf)
        ys = [range_hata_km(m, float(h), LINK["hm_m"], "suburbano") for h in alts]
        ax.plot(alts, ys, color=ps.CAT[i], label=f"SF{sf}")
        alt_rows += [{"SF": sf, "alt_m": int(h), "d_hata_suburbano_km": round(y, 2)} for h, y in zip(alts, ys)]
    ax.set_xlabel("Altitude do drone (m) — antena de solo a 2 m")
    ax.set_ylabel("Alcance estimado (km)")
    ax.set_title(f"Efeito da altitude no alcance (Okumura–Hata suburbano, {args.ptx:.0f} dBm)")
    ax.legend(ncol=4)
    ps.save(fig, fig_dir / "fig_alcance_altitude.png")
    summary["alcance_altitude"] = alt_rows

    # ---------------- 3) energia do radio por mensagem POS (correntes tipicas de datasheet)
    i_tx = {14: 45.0, 22: 118.0}  # mA, SX1262 (DS SX1261/2, regulador DC-DC)
    e_rows = []
    for sf in SFS:
        toa_s = cp.airtime_us(sf, BW, 5, 8, cp.frame_len(cp.POS)) / 1e6
        e_rows.append({"SF": sf, "toa_POS_ms": round(toa_s * 1000, 1),
                       "carga_POS_14dBm_uAh": round(i_tx[14] * toa_s / 3.6, 2),
                       "carga_POS_22dBm_uAh": round(i_tx[22] * toa_s / 3.6, 2)})
    summary["energia_pos"] = e_rows
    with open(res / "lora_calc.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps({"por_sf": sf_rows, "horizonte_km": summary["horizonte_radio_km"]}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
