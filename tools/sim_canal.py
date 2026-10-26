"""Simulacao de eventos discretos: N drones compartilhando um canal LoRa com um gateway.

Esquemas de acesso comparados:
  aloha      ALOHA puro: transmite assim que a telemetria e gerada, sem ACK
  aloha_ack  ALOHA + ACK: espera ACK (timeout) e retransmite ate R vezes com recuo aleatorio exponencial
             (janela = (ToA + espera do ACK) * 2^k; variante "aloha_ack_curto" usa janela = ToA * 2^k)
  csma_ack   CSMA/CAD + ACK: escuta o canal (CAD) antes de transmitir; ocupado -> espera 0,5-1 quadro
  slotted    Slotted ALOHA: transmissoes alinhadas a slots globais (sincronismo por GPS), sem ACK
  tdma_ack   TDMA + ACK: cada drone possui um slot proprio por quadro (slot = uplink + ACK + guarda);
             retransmite no proprio slot do quadro seguinte

Modelo de canal (conservador):
  - gateway com 1 demodulador, half-duplex: qualquer sobreposicao entre uplinks destroi ambos
    (efeito captura opcional, --capture-db), e uplinks que coincidem com uma transmissao do gateway sao perdidos;
  - o ACK e perdido se outro drone transmitir durante o ACK (interferencia no receptor do drone);
  - perda independente por desvanecimento com probabilidade PER (uplink e ACK);
  - fila de 1 posicao por drone: telemetria nova substitui a pendente (a antiga conta como perdida).
Tamanhos de quadro do protocolo cerise_proto: POS = 24 B (uplink), ACK = 16 B.

Uso:
  python tools/sim_canal.py                # todas as varreduras (N, SF, PER, validacao) -> resultados/ e evidencias/
  python tools/sim_canal.py --quick        # versao rapida (menos sementes/duracao)
"""
from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import random
import statistics
from dataclasses import asdict, dataclass
from multiprocessing import Pool
from pathlib import Path

import cerise_proto as cp

SCHEMES = ["aloha", "aloha_ack", "csma_ack", "slotted", "tdma_ack"]
ALL_SCHEMES = SCHEMES + ["aloha_ack_curto"]
LABELS = {"aloha": "ALOHA puro (sem ACK)", "aloha_ack": "ALOHA + ACK/retentativa (R=3)",
          "csma_ack": "CSMA/CAD + ACK (R=3)", "slotted": "Slotted ALOHA (sem ACK)",
          "tdma_ack": "TDMA + ACK (slots por GPS)", "aloha_ack_curto": "ALOHA + ACK, recuo curto (ToA·2^k)"}


@dataclass
class Cfg:
    scheme: str = "aloha"
    n: int = 10
    sf: int = 9
    period_s: float = 5.0
    jitter: float = 0.10
    dur_s: float = 1800.0
    seed: int = 0
    retries: int = 3
    per: float = 0.0
    capture_db: float | None = None
    poisson: bool = False
    guard_s: float = 0.010
    turnaround_s: float = 0.010
    margin_s: float = 0.150   # folga do timeout de ACK (igual ao firmware)
    up_len: int = cp.frame_len(cp.POS)
    ack_len: int = cp.frame_len(cp.ACK)


def toa_s(sf: int, nbytes: int) -> float:
    return cp.airtime_us(sf, 125_000, 5, 8, nbytes) / 1e6


def run(cfg: Cfg) -> dict:
    rng = random.Random(cfg.seed * 1000003 + cfg.n * 101 + cfg.sf * 7 + ALL_SCHEMES.index(cfg.scheme))
    t_up, t_ack = toa_s(cfg.sf, cfg.up_len), toa_s(cfg.sf, cfg.ack_len)
    has_ack = cfg.scheme in ("aloha_ack", "aloha_ack_curto", "csma_ack", "tdma_ack")
    tsym = (2 ** cfg.sf) / 125_000
    t_wait = cfg.turnaround_s + t_ack + cfg.margin_s  # espera maxima pelo ACK
    slot_sa = t_up + cfg.guard_s
    slot_tdma = t_up + cfg.turnaround_s + t_ack + 2 * cfg.guard_s
    frame_tdma = cfg.n * slot_tdma
    t_end_gen = cfg.dur_s
    pwr = [rng.uniform(-120.0, -80.0) for _ in range(cfg.n)]  # RSSI medio no gateway (dBm)

    ev: list = []
    cnt = [0]

    def push(t, kind, data):
        cnt[0] += 1
        heapq.heappush(ev, (t, cnt[0], kind, data))

    nodes = [{"busy": False, "cur": None, "q": None, "att": 0, "wait": False, "deadline": 0.0, "token": 0,
              "air": 0.0} for _ in range(cfg.n)]
    ups: list = []     # transmissoes de uplink recentes: [ini, fim, no]
    gws: list = []     # transmissoes do gateway (ACK): [ini, fim, destino]
    gw_busy = [0.0]
    st = {"gen": 0, "deliv": 0, "tx": 0, "coll": 0, "per_loss": 0, "superseded": 0, "drop_retry": 0,
          "retx": 0, "ack_tx": 0, "ack_lost": 0, "gw_air": 0.0, "cad_busy": 0}
    lat: list = []
    counted = []

    def next_gen(t):
        if cfg.poisson:
            return t + rng.expovariate(1.0 / cfg.period_s)
        return t + cfg.period_s * (1.0 + rng.uniform(-cfg.jitter, cfg.jitter))

    def start_time(i, t):
        if cfg.scheme == "slotted":
            return math.ceil(t / slot_sa - 1e-12) * slot_sa
        if cfg.scheme == "tdma_ack":
            off = i * slot_tdma
            k = math.ceil((t - off) / frame_tdma - 1e-12)
            return off + max(k, 0) * frame_tdma
        return t

    def start_packet(i, pkt, t):
        nd = nodes[i]
        nd["busy"], nd["cur"], nd["att"] = True, pkt, 0
        push(start_time(i, t), "tx_start", i)

    def finish(i, t):
        nd = nodes[i]
        nd["busy"], nd["cur"], nd["wait"] = False, None, False
        if nd["q"] is not None:
            p, nd["q"] = nd["q"], None
            start_packet(i, p, t)

    def overlaps(lst, a, b, exclude=None):
        return [x for x in lst if x is not exclude and x[0] < b and x[1] > a]

    for i in range(cfg.n):
        push(rng.uniform(0, cfg.period_s), "gen", i)

    while ev:
        t, _, kind, data = heapq.heappop(ev)
        if t > cfg.dur_s + 60:
            break
        if len(ups) > 400:
            ups[:] = [x for x in ups if x[1] > t - 10]
            gws[:] = [x for x in gws if x[1] > t - 10]

        if kind == "gen":
            i = data
            if t <= t_end_gen:
                push(next_gen(t), "gen", i)
                pkt = {"gen": t, "ok": False, "count": t <= t_end_gen - 20}
                st["gen"] += 1
                if pkt["count"]:
                    counted.append(pkt)
                nd = nodes[i]
                if nd["busy"]:
                    if nd["q"] is not None:
                        st["superseded"] += 1
                    nd["q"] = pkt
                else:
                    start_packet(i, pkt, t)

        elif kind == "tx_start":
            i = data
            if cfg.scheme == "csma_ack":
                ncad = nodes[i].get("cad", 0)
                busy = any(x[0] <= t - tsym and x[1] > t for x in ups) or any(x[0] <= t < x[1] for x in gws)
                if busy and ncad < 8:
                    nodes[i]["cad"] = ncad + 1
                    st["cad_busy"] += 1
                    push(t + 2 * tsym + rng.uniform(0.5 * t_up, t_up), "tx_start", i)
                    continue
                nodes[i]["cad"] = 0
                t = t + 2 * tsym  # duracao do CAD antes de comecar a transmitir
            tx = [t, t + t_up, i]
            ups.append(tx)
            nodes[i]["air"] += t_up
            st["tx"] += 1
            push(t + t_up, "tx_end", (i, tx))

        elif kind == "tx_end":
            i, tx = data
            nd = nodes[i]
            pkt = nd["cur"]
            ok = True
            if rng.random() < cfg.per:
                ok = False
                st["per_loss"] += 1
            else:
                inter = overlaps(ups, tx[0], tx[1], exclude=tx)
                gw_ovl = overlaps(gws, tx[0], tx[1])
                if gw_ovl:
                    ok = False
                elif inter:
                    if cfg.capture_db is None:
                        ok = False
                    else:
                        p_int = 10 * math.log10(sum(10 ** (pwr[x[2]] / 10) for x in inter))
                        ok = pwr[i] - p_int >= cfg.capture_db
                if not ok:
                    st["coll"] += 1
            if ok and not pkt["ok"]:
                pkt["ok"] = True
                st["deliv"] += 1
                if pkt["count"]:
                    lat.append(t - pkt["gen"])
            if has_ack:
                if ok:
                    ts = max(t + cfg.turnaround_s, gw_busy[0])
                    g = [ts, ts + t_ack, i]
                    gws.append(g)
                    gw_busy[0] = g[1]
                    st["ack_tx"] += 1
                    st["gw_air"] += t_ack
                    push(g[1], "ack_end", (i, g, nd["token"] + 1))
                nd["token"] += 1
                nd["wait"] = True
                nd["deadline"] = t + t_wait
                push(nd["deadline"], "timeout", (i, nd["token"]))
            else:
                finish(i, t)

        elif kind == "ack_end":
            i, g, token = data
            nd = nodes[i]
            if nd["wait"] and nd["token"] == token and g[1] <= nd["deadline"] + 1e-9:
                lost = rng.random() < cfg.per or bool([x for x in overlaps(ups, g[0], g[1]) if x[2] != i])
                if lost:
                    st["ack_lost"] += 1
                else:
                    finish(i, t)

        elif kind == "timeout":
            i, token = data
            nd = nodes[i]
            if nd["wait"] and nd["token"] == token:
                nd["wait"] = False
                if nd["att"] < cfg.retries:
                    nd["att"] += 1
                    st["retx"] += 1
                    if cfg.scheme == "tdma_ack":
                        tn = start_time(i, t)
                    elif cfg.scheme == "aloha_ack_curto":
                        tn = t + rng.uniform(0, t_up * (2 ** nd["att"]))
                    else:
                        tn = t + rng.uniform(0, (t_up + t_wait) * (2 ** nd["att"]))
                    push(tn, "tx_start", i)
                else:
                    st["drop_retry"] += 1
                    finish(i, t)

    n_cnt = len(counted)
    n_ok = sum(1 for p in counted if p["ok"])
    lat_sorted = sorted(lat)
    air = [nd["air"] for nd in nodes]
    return {
        **{k: v for k, v in asdict(cfg).items() if k in ("scheme", "n", "sf", "period_s", "seed", "per",
                                                          "capture_db", "poisson", "retries")},
        "toa_up_ms": round(t_up * 1000, 2), "toa_ack_ms": round(t_ack * 1000, 2),
        "pdr": n_ok / n_cnt if n_cnt else float("nan"),
        "lat_mean_ms": 1000 * statistics.fmean(lat) if lat else float("nan"),
        "lat_p95_ms": 1000 * lat_sorted[int(0.95 * (len(lat_sorted) - 1))] if lat else float("nan"),
        "coll_per_h": st["coll"] * 3600.0 / cfg.dur_s,
        "coll_frac": st["coll"] / st["tx"] if st["tx"] else 0.0,
        "tx_per_pkt": st["tx"] / st["gen"] if st["gen"] else 0.0,
        "duty_node_pct": 100 * statistics.fmean(air) / cfg.dur_s,
        "occupancy_pct": 100 * (sum(air) + st["gw_air"]) / cfg.dur_s,
        "superseded": st["superseded"], "drop_retry": st["drop_retry"], "retx": st["retx"],
        "ack_lost": st["ack_lost"], "generated": st["gen"], "cad_busy": st["cad_busy"],
        "tdma_frame_s": round(frame_tdma, 3) if cfg.scheme == "tdma_ack" else None,
    }


def aggregate(rows, keys):
    groups: dict = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    out = []
    metrics = ["pdr", "lat_mean_ms", "lat_p95_ms", "coll_per_h", "coll_frac", "tx_per_pkt", "duty_node_pct",
               "occupancy_pct"]
    for k, rs in sorted(groups.items()):
        o = dict(zip(keys, k))
        o["toa_up_ms"] = rs[0]["toa_up_ms"]
        o["seeds"] = len(rs)
        for m in metrics:
            vals = [r[m] for r in rs if not (isinstance(r[m], float) and math.isnan(r[m]))]
            o[m] = round(statistics.fmean(vals), 5) if vals else None
            o[m + "_std"] = round(statistics.pstdev(vals), 5) if len(vals) > 1 else 0.0
        out.append(o)
    return out


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main():
    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--plot-only", action="store_true", help="refaz as figuras a partir dos CSV existentes")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--dur", type=float, default=1800.0)
    ap.add_argument("--out-res", default=str(root / "resultados"))
    ap.add_argument("--out-fig", default=str(root / "evidencias"))
    args = ap.parse_args()
    if args.quick:
        args.seeds, args.dur = 2, 600.0
    res, figd = Path(args.out_res), Path(args.out_fig)
    res.mkdir(parents=True, exist_ok=True)
    figd.mkdir(parents=True, exist_ok=True)
    if args.plot_only:
        def load(name):
            with open(res / name, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            for r in rows:
                for k, v in r.items():
                    if k != "scheme":
                        try:
                            r[k] = int(v) if v.lstrip("-").isdigit() else float(v)
                        except ValueError:
                            pass
            return rows
        plot(load("sim_canal_vs_N.csv"), load("sim_canal_vs_SF.csv"), load("sim_canal_vs_PER.csv"),
             load("sim_canal_validacao_aloha.csv"), load("sim_canal_captura6dB.csv"), figd)
        return
    seeds = range(args.seeds)
    ns = list(range(2, 21))

    jobs_n = [Cfg(scheme=s, n=n, sf=9, seed=k, dur_s=args.dur) for s in ALL_SCHEMES for n in ns for k in seeds]
    jobs_sf = [Cfg(scheme=s, n=10, sf=sf, seed=k, dur_s=args.dur) for s in SCHEMES for sf in range(7, 13)
               for k in seeds]
    jobs_per = [Cfg(scheme=s, n=5, sf=9, seed=k, dur_s=args.dur, per=p) for s in SCHEMES
                for p in (0.0, 0.05, 0.10, 0.20, 0.30) for k in seeds]
    jobs_val = [Cfg(scheme="aloha", n=n, sf=9, seed=k, dur_s=args.dur, poisson=True) for n in ns for k in seeds]
    jobs_cap = [Cfg(scheme=s, n=n, sf=9, seed=k, dur_s=args.dur, capture_db=6.0) for s in ("aloha", "aloha_ack")
                for n in ns for k in seeds]

    with Pool() as pool:
        r_n = pool.map(run, jobs_n)
        r_sf = pool.map(run, jobs_sf)
        r_per = pool.map(run, jobs_per)
        r_val = pool.map(run, jobs_val)
        r_cap = pool.map(run, jobs_cap)

    a_n = aggregate(r_n, ["scheme", "n", "sf"])
    a_sf = aggregate(r_sf, ["scheme", "n", "sf"])
    a_per = aggregate(r_per, ["scheme", "n", "sf", "per"])
    a_val = aggregate(r_val, ["scheme", "n", "sf"])
    a_cap = aggregate(r_cap, ["scheme", "n", "sf"])
    toa9 = toa_s(9, cp.frame_len(cp.POS))
    for r in a_val:  # teoria ALOHA puro (chegadas de Poisson): P = exp(-2 (N-1) * toa / T)
        r["pdr_teoria"] = round(math.exp(-2 * (r["n"] - 1) * toa9 / 5.0), 5)
    write_csv(res / "sim_canal_vs_N.csv", a_n)
    write_csv(res / "sim_canal_vs_SF.csv", a_sf)
    write_csv(res / "sim_canal_vs_PER.csv", a_per)
    write_csv(res / "sim_canal_validacao_aloha.csv", a_val)
    write_csv(res / "sim_canal_captura6dB.csv", a_cap)
    meta = {"seeds": args.seeds, "dur_s": args.dur, "period_s": 5.0, "jitter": 0.10, "retries": 3,
            "up_frame_B": cp.frame_len(cp.POS), "ack_frame_B": cp.frame_len(cp.ACK), "bw_hz": 125000, "cr": "4/5",
            "runs": len(r_n) + len(r_sf) + len(r_per) + len(r_val) + len(r_cap)}
    (res / "sim_canal_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    plot(a_n, a_sf, a_per, a_val, a_cap, figd)
    # resumo no terminal
    print(json.dumps(meta))
    for s in SCHEMES:
        for n in (5, 10, 20):
            r = next(x for x in a_n if x["scheme"] == s and x["n"] == n)
            print(f"SF9 N={n:2d} {s:10s} PDR={r['pdr']:.3f} lat_med={r['lat_mean_ms']:.0f} ms "
                  f"p95={r['lat_p95_ms']:.0f} ms colis/h={r['coll_per_h']:.0f} ocup={r['occupancy_pct']:.1f}%")


def plot(a_n, a_sf, a_per, a_val, a_cap, figd):
    import plot_style as ps
    ps.apply()
    import matplotlib.pyplot as plt

    def series(rows, s, x, y):
        rs = sorted([r for r in rows if r["scheme"] == s], key=lambda r: r[x])
        return [r[x] for r in rs], [r[y] for r in rs], [r[y + "_std"] for r in rs]

    # ---- PDR, latencia e colisoes x N
    fig, axs = plt.subplots(1, 3, figsize=(12.5, 4.0))
    for s in ALL_SCHEMES:
        i = SCHEMES.index(s) if s in SCHEMES else 1
        sty = dict(color=ps.CAT[i], ls=":" if s == "aloha_ack_curto" else "-",
                   marker="o" if s in SCHEMES else None, ms=3.5)
        x, y, e = series(a_n, s, "n", "pdr")
        axs[0].errorbar(x, [100 * v for v in y], yerr=[100 * v for v in e], capsize=2, label=LABELS[s], **sty)
        x, y, e = series(a_n, s, "n", "lat_mean_ms")
        axs[1].plot(x, y, **sty)
        x, y, e = series(a_n, s, "n", "coll_per_h")
        axs[2].plot(x, y, **sty)
    axs[0].set_ylabel("PDR (%)")
    axs[0].set_ylim(0, 102)
    axs[1].set_ylabel("Latência média geração→recepção (ms)")
    axs[1].set_yscale("log")
    axs[2].set_ylabel("Transmissões colididas por hora")
    for ax, t in zip(axs, ["Taxa de entrega (PDR)", "Latência", "Colisões no gateway"]):
        ax.set_xlabel("Número de drones (N)")
        ax.set_title(t)
        ax.set_xticks([2, 5, 8, 11, 14, 17, 20])
    fig.legend(*axs[0].get_legend_handles_labels(), loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.12))
    fig.suptitle("Simulação do canal compartilhado — SF9, POS de 24 B a cada 5 s (±10 %), média de sementes ± desvio",
                 fontsize=11, fontweight="bold")
    ps.save(fig, figd / "fig_sim_vs_N.png")

    # ---- PDR e ocupacao x SF (N = 10)
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.0))
    for i, s in enumerate(SCHEMES):
        x, y, e = series(a_sf, s, "sf", "pdr")
        axs[0].errorbar(x, [100 * v for v in y], yerr=[100 * v for v in e], color=ps.CAT[i], marker="o", ms=4,
                        capsize=2, label=LABELS[s])
        x, y, e = series(a_sf, s, "sf", "occupancy_pct")
        axs[1].plot(x, y, color=ps.CAT[i], marker="o", ms=4)
    for ax in axs:
        ax.axvspan(10.5, 12.5, color=ps.GRID, alpha=0.6, lw=0)
        ax.set_xticks(range(7, 13))
        ax.set_xlabel("Fator de espalhamento (SF)")
    axs[0].text(10.6, 4, "POS > 400 ms", fontsize=8, color=ps.INK2)
    axs[0].set_ylabel("PDR (%)")
    axs[0].set_ylim(0, 102)
    axs[0].set_title("PDR x SF (N = 10 drones)")
    axs[1].set_ylabel("Soma dos tempos no ar / tempo (%)")
    axs[1].set_title("Ocupação do canal (> 100 % = sobreposição)")
    fig.legend(*axs[0].get_legend_handles_labels(), loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.12))
    ps.save(fig, figd / "fig_sim_vs_SF.png")

    # ---- validacao (Poisson) + PER + captura
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.0))
    x = [r["n"] for r in a_val]
    axs[0].plot(x, [100 * r["pdr_teoria"] for r in a_val], color=ps.INK, lw=1.4, ls="--",
                label="Teoria ALOHA puro: exp(−2(N−1)·ToA/T)")
    axs[0].errorbar(x, [100 * r["pdr"] for r in a_val], yerr=[100 * r["pdr_std"] for r in a_val], color=ps.CAT[0],
                    marker="o", ms=4, ls="none", capsize=2, label="Simulação (chegadas de Poisson)")
    for i, s in enumerate(("aloha", "aloha_ack")):
        xs, ys, _ = series(a_cap, s, "n", "pdr")
        axs[0].plot(xs, [100 * v for v in ys], color=ps.CAT[[0, 1][i]], lw=1.2, alpha=0.6, ls=":",
                    label=f"{LABELS[s]} c/ captura 6 dB")
    axs[0].set_xlabel("Número de drones (N)")
    axs[0].set_ylabel("PDR (%)")
    axs[0].set_ylim(0, 102)
    axs[0].set_title("Validação do simulador e efeito captura (SF9)")
    axs[0].set_xticks([2, 5, 8, 11, 14, 17, 20])
    axs[0].legend(fontsize=7.5, loc="lower left")
    for i, s in enumerate(SCHEMES):
        xs, ys, es = series(a_per, s, "per", "pdr")
        axs[1].errorbar([100 * v for v in xs], [100 * v for v in ys], yerr=[100 * v for v in es], color=ps.CAT[i],
                        marker="o", ms=4, capsize=2, label=LABELS[s])
    axs[1].set_xlabel("Taxa de perda por desvanecimento, PER (%)")
    axs[1].set_ylabel("PDR (%)")
    axs[1].set_ylim(0, 102)
    axs[1].set_title("Robustez a perdas do enlace (N = 5, SF9)")
    axs[1].legend(fontsize=7.5, loc="lower left")
    ps.save(fig, figd / "fig_sim_validacao_per.png")


if __name__ == "__main__":
    main()
