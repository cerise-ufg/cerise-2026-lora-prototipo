"""Fluxo serial SIMULADO do gateway (para testar o dashboard sem hardware).

Emula a saida serial do firmware gateway (linhas JSON identicas as de cp_to_json) para
K drones que percorrem areas em zigue-zague. O caminho dos bytes e real: cada mensagem e
codificada com o protocolo (cerise_proto.encode), passa por um canal simulado
(perda log-distancia n = 3,0 + sombreamento, colisoes entre drones, perda de ACK) e e
decodificada (cerise_proto.decode) antes de virar JSON.

Saida: servidor TCP preso ao loopback (o dashboard le com a URL pyserial
"socket://127.0.0.1:PORTA", o mesmo codigo usado para COMx) ou stdout.

  python tools/sim_serial_gateway.py --port 8195 --speed 10 --duration 600
  python tools/sim_serial_gateway.py --stdout --duration 120 > fluxo.jsonl
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import random
import socket
import sys
import time
from dataclasses import dataclass, field

import cerise_proto as cp

BASE_LAT, BASE_LON = -16.6799, -49.2550  # posicao (sintetica) da estacao de solo
F_MHZ = 916.8
NOISE_DBM = -174 + 10 * math.log10(125e3) + 6.0
SNR_LIM = {7: -7.5, 8: -10.0, 9: -12.5, 10: -15.0, 11: -17.5, 12: -20.0}


def m2deg(dn, de, lat=BASE_LAT):
    return dn / 111320.0, de / (111320.0 * math.cos(math.radians(lat)))


@dataclass
class Drone:
    id: int
    north_m: float
    east_m: float
    w_m: float = 400.0
    h_m: float = 300.0
    spacing: float = 25.0
    speed: float = 8.0
    alt_m: float = 60.0
    area: int = 1
    task: int = 1
    s: float = 0.0
    people: int = 0
    seq: int = field(default_factory=lambda: random.randint(0, 65535))
    sent: int = 0
    cycle: int = 0
    reported: int = 0
    done: bool = False

    def path_len(self):
        rows = int(self.h_m // self.spacing) + 1
        return rows * self.w_m + (rows - 1) * self.spacing

    def pos(self):
        seg = self.w_m + self.spacing
        row = int(self.s // seg)
        off = self.s - row * seg
        rows = int(self.h_m // self.spacing) + 1
        if row >= rows:
            row, off = rows - 1, self.w_m
        if off <= self.w_m:
            x = off if row % 2 == 0 else self.w_m - off
            y = row * self.spacing
        else:
            x = self.w_m if row % 2 == 0 else 0.0
            y = row * self.spacing + (off - self.w_m)
        dn, de = self.north_m + y, self.east_m + x
        dlat, dlon = m2deg(dn, de)
        return BASE_LAT + dlat, BASE_LON + dlon, math.hypot(dn, de)


class Sim:
    def __init__(self, n_drones=2, sf=9, seed=1, speed=10.0, ptx=14.0, period=5.0):
        self.rng = random.Random(seed)
        random.seed(seed)
        self.sf, self.ptx, self.period, self.speed = sf, ptx, period, speed
        layouts = [(800, 600), (-3500, 3000), (-5000, -4000), (2500, -2000)]  # 1,0 km; 4,6 km; 6,4 km; 3,2 km
        self.drones = [Drone(i + 1, *layouts[i % len(layouts)]) for i in range(n_drones)]
        self.t = 0.0
        self.ev = []
        self.k = 0
        self.busy_until = []  # (ini, fim) de uplinks em curso
        self.dedup = {}
        self.stats = {"rx_ok": 0, "rx_err": 0, "rx_phy_err": 0, "dup": 0, "acks_tx": 0}
        self.nodes = {}
        self.gw_seq = 0
        self.truth = {d.id: {"msgs": 0, "delivered": 0} for d in self.drones}
        for d in self.drones:
            self.push(self.rng.uniform(0, period), "gen", d)
        self.push(10.0, "stats", None)

    def push(self, t, kind, data):
        self.k += 1
        heapq.heappush(self.ev, (t, self.k, kind, data))

    def rssi_snr(self, dist_m, alt_m):
        d3 = max(1.0, math.hypot(dist_m, alt_m))
        pl = 20 * math.log10(F_MHZ) - 27.55 + 30.0 * math.log10(d3)  # log-distancia, n = 3,0, d0 = 1 m
        rssi = self.ptx + 2.0 + 3.0 - 1.5 - pl + self.rng.gauss(0, 4.0)
        snr = min(rssi - NOISE_DBM, 11.5) + self.rng.gauss(0, 0.8)
        return rssi, snr

    def build(self, d: Drone):
        lat, lon, _ = d.pos()
        d.cycle += 1
        batt = max(20, 100 - int(self.t / 120))
        if d.cycle % 6 == 0:
            p = cp.Packet(cp.HEARTBEAT, src=d.id, seq=d.seq, fields={
                "batt_pct": batt, "batt_mv": 3300 + 9 * batt, "state": 2, "uptime_s": int(self.t)})
            msgs = [p]
        else:
            msgs = [cp.Packet(cp.POS, src=d.id, seq=d.seq, fields={
                "lat_e7": round(lat * 1e7), "lon_e7": round(lon * 1e7), "alt_dm": round(d.alt_m * 10),
                "batt_pct": batt, "people": d.people, "area_id": d.area})]
        d.seq = (d.seq + 1) & 0xFFFF
        if d.people >= d.reported + 5:
            msgs.append(cp.Packet(cp.COUNT_REPORT, src=d.id, seq=d.seq, fields={
                "area_id": d.area, "task_id": d.task, "people": d.people, "confidence_pct": 80,
                "lat_e7": round(lat * 1e7), "lon_e7": round(lon * 1e7)}))
            d.seq = (d.seq + 1) & 0xFFFF
            d.reported = d.people
        if d.s >= d.path_len() and not d.done:
            msgs.append(cp.Packet(cp.COVERAGE_DONE, src=d.id, seq=d.seq, fields={
                "area_id": d.area, "task_id": d.task, "coverage_pct": 100, "duration_s": int(self.t),
                "people_total": d.people}))
            d.seq = (d.seq + 1) & 0xFFFF
            d.done = True
        for m in msgs:
            m.net_id = 42
        return msgs

    def lines(self):
        """Gera (instante_sim, linha_json) indefinidamente."""
        toa = cp.airtime_us(self.sf, 125000, 5, 8, cp.frame_len(cp.POS)) / 1e6
        yield 0.0, json.dumps({"evt": "boot", "role": "gateway", "fw": "1.0.0-sim", "board": "simulado",
                               "radio": "SX1262", "rc": 0, "freq_mhz": F_MHZ, "sf": self.sf, "bw_khz": 125.0,
                               "cr": "4/5", "tx_dbm": int(self.ptx), "proto": 1, "net": 42},
                              separators=(",", ":"))
        while self.ev:
            t, _, kind, data = heapq.heappop(self.ev)
            dt = t - self.t
            self.t = t
            for d in self.drones:  # avanca as trajetorias
                d.s = min(d.s + d.speed * dt, d.path_len())
                if d.s < d.path_len() and self.rng.random() < 0.15 * dt:
                    d.people += 1
            if kind == "stats":
                self.push(t + 10.0, "stats", None)
                yield t, self.stats_line()
                continue
            d = data
            self.push(t + self.period * (1 + self.rng.uniform(-0.1, 0.1)), "gen", d)
            for idx, pkt in enumerate(self.build(d)):
                self.truth[d.id]["msgs"] += 1
                delivered = False
                for attempt in range(4):
                    pkt.flags = cp.FLAG_ACK_REQ | (cp.FLAG_RETRY if attempt else 0)
                    frame = cp.encode(pkt)
                    t_tx = self.t + idx * 0.45 + attempt * 1.0 + self.rng.uniform(0, 0.2)
                    _, _, dist = d.pos()
                    rssi, snr = self.rssi_snr(dist, d.alt_m)
                    collided = any(a < t_tx + toa and b > t_tx for a, b in self.busy_until)
                    self.busy_until = [(a, b) for a, b in self.busy_until if b > t_tx - 5] + [(t_tx, t_tx + toa)]
                    margin = snr - SNR_LIM[self.sf]
                    p_ok = 1 / (1 + math.exp(-(margin - 1.0) / 0.7))
                    if collided or self.rng.random() > p_ok:
                        if not collided and margin > -3:  # sinal fraco detectado mas com CRC de PHY invalido
                            self.stats["rx_phy_err"] += 1
                            yield t_tx, json.dumps({"evt": "rx_err", "t_ms": int(t_tx * 1000), "err": "PHY",
                                                    "rc": -7, "rssi": round(rssi), "snr": round(snr, 2)},
                                                   separators=(",", ":"))
                        continue
                    rx = cp.decode(frame)  # caminho real de decodificacao
                    dup = self.dedup.get(rx.src) == rx.seq
                    self.dedup[rx.src] = rx.seq
                    self.stats["rx_ok"] += 1
                    self.stats["dup"] += int(dup)
                    self.node_update(rx, dup, round(rssi), snr, t_tx)
                    if not dup:
                        self.truth[d.id]["delivered"] += 1
                    yield t_tx, cp.to_json(rx, int(t_tx * 1000), round(rssi), snr, dup)
                    self.stats["acks_tx"] += 1
                    if self.rng.random() < 0.9:  # ACK recebido pelo drone
                        delivered = True
                        break
                if not delivered:
                    pass

    def node_update(self, p, dup, rssi, snr, t):
        n = self.nodes.get(p.src)
        if n is None:
            n = self.nodes[p.src] = {"first": p.seq, "last": p.seq, "rx": 0, "uniq": 0, "dup": 0}
        n["rx"] += 1
        if dup:
            n["dup"] += 1
        else:
            n["uniq"] += 1
            if ((p.seq - n["last"]) & 0xFFFF) < 32768:  # so avanca (como no firmware, que envia em ordem)
                n["last"] = p.seq
        n["rssi"], n["snr"], n["t"] = rssi, snr, t

    def stats_line(self):
        nodes = []
        for i, n in sorted(self.nodes.items()):
            exp = ((n["last"] - n["first"]) & 0xFFFF) + 1
            nodes.append({"id": i, "rx": n["rx"], "uniq": n["uniq"], "dup": n["dup"],
                          "lost": max(0, exp - n["uniq"]), "pdr": round(n["uniq"] / exp, 4),
                          "rssi": n["rssi"], "snr": round(n["snr"], 2), "age_s": int(self.t - n["t"])})
        s = {"evt": "stats", "t_ms": int(self.t * 1000), **self.stats, "rx_err": 0, "foreign": 0, "dl_tx": 0,
             "dl_acked": 0, "nodes": nodes}
        return json.dumps(s, separators=(",", ":"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8195)
    ap.add_argument("--stdout", action="store_true")
    ap.add_argument("--speed", type=float, default=10.0, help="fator de aceleracao do tempo")
    ap.add_argument("--duration", type=float, default=600.0, help="tempo simulado (s)")
    ap.add_argument("--drones", type=int, default=2)
    ap.add_argument("--sf", type=int, default=9)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--truth", default=None, help="grava a verdade de campo (mensagens geradas/entregues) em JSON")
    args = ap.parse_args()
    sim = Sim(args.drones, args.sf, args.seed, args.speed)

    def stream(write):
        t0 = time.time()
        for t_sim, line in sim.lines():
            if t_sim > args.duration:
                break
            if args.speed > 0:
                wait = t0 + t_sim / args.speed - time.time()
                if wait > 0:
                    time.sleep(wait)
            write((line + "\n").encode())
        if args.truth:
            with open(args.truth, "w", encoding="utf-8") as f:
                json.dump(sim.truth, f, indent=2)

    if args.stdout:
        stream(lambda b: (sys.stdout.buffer.write(b), sys.stdout.flush()))
        return
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))  # somente loopback
    srv.listen(1)
    print(f"# gateway simulado em socket://127.0.0.1:{args.port} (aguardando conexao)", file=sys.stderr)
    conn, _ = srv.accept()
    time.sleep(0.5)  # o pyserial (socket://) descarta o buffer de entrada logo apos conectar
    with conn:
        stream(conn.sendall)
    srv.close()


if __name__ == "__main__":
    main()
