"""Leitor serial / dashboard do gateway LoRa CERISE.

Le as linhas JSON que o firmware gateway imprime na serial (115200 bps), grava CSV/JSONL,
calcula PDR por drone a partir das lacunas de numero de sequencia (independente das
estatisticas do proprio gateway) e gera um painel PNG (RSSI, SNR, PDR, mapa).

Exemplos:
  python tools/gateway_dashboard.py --port COM6 --duration 600 --out resultados/sessao_bancada
  python tools/gateway_dashboard.py --port socket://127.0.0.1:8195 --duration 70 --out resultados/dash_sim
  python tools/gateway_dashboard.py --replay resultados/dash_sim/brutos.jsonl --out resultados/replay
  python tools/gateway_dashboard.py --port COM6 --label "500m_SF9"     # rotula linhas de um teste de alcance
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

PKT_TYPES = {"HEARTBEAT", "POS", "AREA_ASSIGN", "COVERAGE_DONE", "COUNT_REPORT", "ACK"}


class GatewayLog:
    def __init__(self, label: str | None = None):
        self.label = label
        self.packets: list[dict] = []
        self.events: list[dict] = []
        self.raw: list[str] = []
        self.bad_lines = 0
        self.boot: dict | None = None
        self.last_stats: dict | None = None

    def feed(self, line: str) -> dict | None:
        line = line.strip()
        if not line:
            return None
        self.raw.append(line)
        if not line.startswith("{"):
            self.bad_lines += 1  # comentarios "# ..." ou lixo de boot do ESP32
            return None
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            self.bad_lines += 1
            return None
        obj["_rx_wall"] = round(time.time(), 3)
        if self.label:
            obj["label"] = self.label
        if obj.get("type") in PKT_TYPES:
            self.packets.append(obj)
        elif "evt" in obj:
            self.events.append(obj)
            if obj["evt"] == "boot":
                self.boot = obj
            elif obj["evt"] == "stats":
                self.last_stats = obj
        return obj

    # ------------------------------------------------------------------ analise
    def uplinks(self):
        return [p for p in self.packets if p["type"] != "ACK" and p.get("dst", 0) in (0, 255)]

    def summary(self) -> dict:
        per = defaultdict(list)
        for p in self.uplinks():
            per[p["src"]].append(p)
        nodes = {}
        for src, ps in sorted(per.items()):
            ps = sorted(ps, key=lambda p: p["t_ms"])
            uniq = {p["seq"] for p in ps if not p.get("dup")}
            # seq e uint16 por mensagem: "desembrulha" com deltas com sinal (tolera reordenacao e a volta 65535->0)
            seqs = [p["seq"] for p in ps if not p.get("dup")]
            expected = 0
            if seqs:
                cur, lo, hi = 0, 0, 0
                for a, b in zip(seqs, seqs[1:]):
                    cur += ((b - a + 32768) & 0xFFFF) - 32768
                    lo, hi = min(lo, cur), max(hi, cur)
                expected = hi - lo + 1
            rssi = [p["rssi"] for p in ps]
            snr = [p["snr"] for p in ps]
            gw = None
            if self.last_stats:
                gw = next((n for n in self.last_stats.get("nodes", []) if n["id"] == src), None)
            nodes[src] = {
                "rx": len(ps), "uniq": len(uniq), "dup": sum(1 for p in ps if p.get("dup")),
                "esperado": expected, "perdidos": max(0, expected - len(uniq)),
                "pdr": round(len(uniq) / expected, 4) if expected else None,
                "pdr_gateway": gw["pdr"] if gw else None,
                "rssi_med": round(statistics.fmean(rssi), 1), "rssi_min": min(rssi), "rssi_max": max(rssi),
                "snr_med": round(statistics.fmean(snr), 2), "snr_min": min(snr),
                "tipos": dict(Counter(p["type"] for p in ps)),
                "duracao_s": round((ps[-1]["t_ms"] - ps[0]["t_ms"]) / 1000, 1),
            }
        errs = Counter(e.get("err") for e in self.events if e["evt"] == "rx_err")
        return {"label": self.label, "linhas": len(self.raw), "linhas_invalidas": self.bad_lines,
                "pacotes": len(self.packets), "eventos": dict(Counter(e["evt"] for e in self.events)),
                "rx_err": dict(errs), "boot": self.boot, "nos": nodes}

    # ------------------------------------------------------------------ saidas
    def write(self, out: Path):
        out.mkdir(parents=True, exist_ok=True)
        (out / "brutos.jsonl").write_text("\n".join(self.raw) + "\n", encoding="utf-8")
        for name, rows in (("pacotes.csv", self.packets), ("eventos.csv", self.events)):
            if not rows:
                continue
            keys: list[str] = []
            for r in rows:
                for k in r:
                    if k not in keys:
                        keys.append(k)
            with open(out / name, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                w.writeheader()
                for r in rows:
                    w.writerow({k: (json.dumps(v) if isinstance(v, (list, dict)) else v) for k, v in r.items()})
        s = self.summary()
        (out / "resumo.json").write_text(json.dumps(s, ensure_ascii=False, indent=2), encoding="utf-8")
        return s

    def plot(self, path: Path, title: str = "Dashboard do gateway LoRa", gw_pos: tuple | None = None):
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import plot_style as ps
        ps.apply()
        import matplotlib.pyplot as plt

        ups = self.uplinks()
        srcs = sorted({p["src"] for p in ups})
        t0 = min(p["t_ms"] for p in ups) if ups else 0
        s = self.summary()
        fig, axs = plt.subplots(2, 2, figsize=(11.5, 7.6))
        for k, src in enumerate(srcs):
            ps_ = sorted([p for p in ups if p["src"] == src], key=lambda p: p["t_ms"])
            col = ps.CAT[k % len(ps.CAT)]
            tt = [(p["t_ms"] - t0) / 1000 for p in ps_]
            axs[0, 0].plot(tt, [p["rssi"] for p in ps_], color=col, lw=1.2, marker="o", ms=2.5,
                           label=f"drone {src}")
            axs[0, 1].plot(tt, [p["snr"] for p in ps_], color=col, lw=1.2, marker="o", ms=2.5,
                           label=f"drone {src}")
            pos = [p for p in ps_ if p["type"] in ("POS", "COUNT_REPORT")]
            axs[1, 1].plot([p["lon"] for p in pos if p["type"] == "POS"],
                           [p["lat"] for p in pos if p["type"] == "POS"], color=col, lw=1.0, marker=".", ms=3,
                           label=f"drone {src} (POS)")
            cr = [p for p in pos if p["type"] == "COUNT_REPORT"]
            if cr:
                axs[1, 1].scatter([p["lon"] for p in cr], [p["lat"] for p in cr], s=28, marker="*", color=col,
                                  edgecolor=ps.INK, linewidth=0.4, zorder=3,
                                  label=f"drone {src} COUNT_REPORT")
        errs = [e for e in self.events if e["evt"] == "rx_err" and "t_ms" in e]
        if errs:
            axs[0, 0].scatter([(e["t_ms"] - t0) / 1000 for e in errs], [e.get("rssi", 0) for e in errs],
                              marker="x", color=ps.INK2, s=18, label="erro de CRC (PHY)")
        axs[0, 0].set_title("RSSI dos pacotes recebidos")
        axs[0, 0].set_xlabel("tempo (s)")
        axs[0, 0].set_ylabel("RSSI (dBm)")
        lo, hi = axs[0, 0].get_ylim()
        axs[0, 0].set_ylim(lo, hi + 0.25 * (hi - lo))
        axs[0, 0].legend(ncol=4, loc="upper center", fontsize=7.5)
        axs[0, 1].set_title("SNR dos pacotes recebidos")
        axs[0, 1].set_xlabel("tempo (s)")
        axs[0, 1].set_ylabel("SNR (dB)")
        if self.boot and self.boot.get("sf") in (7, 8, 9, 10, 11, 12):
            lim = {7: -7.5, 8: -10, 9: -12.5, 10: -15, 11: -17.5, 12: -20}[self.boot["sf"]]
            axs[0, 1].axhline(lim, color=ps.INK, ls="--", lw=1)
            axs[0, 1].text(0, lim + 0.6, f"limite de demodulação SF{self.boot['sf']} ({lim} dB)", fontsize=8)
        lo, hi = axs[0, 1].get_ylim()
        axs[0, 1].set_ylim(lo, hi + 0.2 * (hi - lo))
        axs[0, 1].legend(ncol=3, loc="upper center", fontsize=7.5)
        # PDR por drone: calculado aqui x reportado pelo gateway
        xs = list(range(len(srcs)))
        pdr = [100 * (s["nos"][src]["pdr"] or 0) for src in srcs]
        pdr_gw = [100 * (s["nos"][src]["pdr_gateway"] or 0) for src in srcs]
        axs[1, 0].bar([x - 0.2 for x in xs], pdr, width=0.38, color=ps.INK2, label="calculado (lacunas de seq)")
        axs[1, 0].bar([x + 0.2 for x in xs], pdr_gw, width=0.38, color="#b9b8b2", label="reportado pelo gateway")
        for x, v, src in zip(xs, pdr, srcs):
            n = s["nos"][src]
            axs[1, 0].text(x - 0.2, v + 1, f"{v:.1f}%\n{n['uniq']}/{n['esperado']}", ha="center", fontsize=8)
        axs[1, 0].set_xticks(xs, [f"drone {s_}" for s_ in srcs])
        axs[1, 0].set_ylim(0, 115)
        axs[1, 0].set_ylabel("PDR (%)")
        axs[1, 0].set_title("Taxa de entrega por drone")
        axs[1, 0].legend(loc="upper center", ncol=2, fontsize=7.5, bbox_to_anchor=(0.5, -0.1))
        if gw_pos:
            axs[1, 1].scatter([gw_pos[1]], [gw_pos[0]], marker="^", s=90, color=ps.INK, label="gateway (solo)",
                              zorder=4)
        axs[1, 1].set_title("Trajetórias reportadas (POS) e contagens")
        axs[1, 1].set_xlabel("longitude (°)")
        axs[1, 1].set_ylabel("latitude (°)")
        axs[1, 1].legend(fontsize=7, loc="best")
        axs[1, 1].ticklabel_format(useOffset=False)
        fig.suptitle(title, fontsize=12, fontweight="bold")
        ps.save(fig, path)


def read_port(url: str, duration: float, log: GatewayLog, echo: bool):
    import serial  # pyserial
    ser = serial.serial_for_url(url, baudrate=115200, timeout=0.5)
    t_end = time.time() + duration if duration > 0 else float("inf")
    try:
        while time.time() < t_end:
            try:
                raw = ser.readline()
            except serial.SerialException:
                break  # porta/socket fechado
            if not raw:
                continue
            obj = log.feed(raw.decode("utf-8", errors="replace"))
            if echo and obj is not None:
                print(raw.decode("utf-8", errors="replace").rstrip())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def print_summary(s: dict):
    print(f"# linhas={s['linhas']} pacotes={s['pacotes']} invalidas={s['linhas_invalidas']} rx_err={s['rx_err']}")
    print(f"{'drone':>5} {'rx':>5} {'unicos':>6} {'dup':>4} {'esper.':>6} {'perd.':>5} {'PDR%':>6} "
          f"{'PDRgw%':>6} {'RSSI med':>8} {'RSSI min':>8} {'SNR med':>7} tipos")
    for src, n in s["nos"].items():
        pg = f"{100 * n['pdr_gateway']:.1f}" if n["pdr_gateway"] is not None else "-"
        print(f"{src:>5} {n['rx']:>5} {n['uniq']:>6} {n['dup']:>4} {n['esperado']:>6} {n['perdidos']:>5} "
              f"{100 * n['pdr']:>6.1f} {pg:>6} {n['rssi_med']:>8.1f} {n['rssi_min']:>8} {n['snr_med']:>7.2f} "
              f"{n['tipos']}")


def main():
    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="COMx, /dev/ttyUSB0 ou URL pyserial (socket://127.0.0.1:8195)")
    src.add_argument("--replay", help="arquivo .jsonl gravado anteriormente")
    ap.add_argument("--duration", type=float, default=0, help="segundos de captura (0 = ate Ctrl+C/fim)")
    ap.add_argument("--out", default=str(root / "resultados" / "sessao"))
    ap.add_argument("--label", default=None)
    ap.add_argument("--echo", action="store_true", help="ecoa as linhas recebidas")
    ap.add_argument("--title", default="Dashboard do gateway LoRa — CERISE")
    ap.add_argument("--gw-pos", default=None, help="lat,lon da estacao de solo para o mapa")
    args = ap.parse_args()
    log = GatewayLog(args.label)
    if args.replay:
        for line in Path(args.replay).read_text(encoding="utf-8").splitlines():
            log.feed(line)
    else:
        read_port(args.port, args.duration, log, args.echo)
    out = Path(args.out)
    s = log.write(out)
    print_summary(s)
    if log.uplinks():
        gw = tuple(float(v) for v in args.gw_pos.split(",")) if args.gw_pos else None
        log.plot(out / "dashboard.png", args.title, gw)


if __name__ == "__main__":
    main()
