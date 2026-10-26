"""Testes do leitor serial/dashboard do gateway com um fluxo serial simulado."""
import json
import socket
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import cerise_proto as cp  # noqa: E402
import gateway_dashboard as gd  # noqa: E402
import sim_serial_gateway as sg  # noqa: E402


def _pos(src, seq):
    return cp.Packet(cp.POS, src=src, seq=seq, fields={"lat_e7": -166799000, "lon_e7": -492550000, "alt_dm": 600,
                                                       "batt_pct": 90, "people": 0, "area_id": 1})


def test_pdr_with_gaps_dups_and_wraparound():
    log = gd.GatewayLog()
    log.feed("# comentario do firmware")
    log.feed("lixo de boot \x00\x7f")
    seqs = [65533, 65534, 65534, 0, 2, 3]  # 65535 e 1 perdidos; 65534 duplicado
    dups = [False, False, True, False, False, False]
    for k, (s, d) in enumerate(zip(seqs, dups)):
        log.feed(cp.to_json(_pos(7, s), 1000 * k, -100, 5.0, d))
    log.feed(json.dumps({"evt": "rx_err", "t_ms": 10, "err": "PHY", "rc": -7, "rssi": -130, "snr": -14.0}))
    s = log.summary()
    n = s["nos"][7]
    assert n["rx"] == 6 and n["dup"] == 1 and n["uniq"] == 5
    assert n["esperado"] == 7 and n["perdidos"] == 2
    assert abs(n["pdr"] - 5 / 7) < 1e-4
    assert s["linhas_invalidas"] == 2
    assert s["rx_err"] == {"PHY": 1}


def test_reordering_does_not_inflate_expected():
    log = gd.GatewayLog()
    for k, s in enumerate([10, 12, 11, 13]):
        log.feed(cp.to_json(_pos(1, s), k, -90, 8.0, False))
    assert log.summary()["nos"][1]["esperado"] == 4


def test_socket_stream_end_to_end(tmp_path):
    sim = sg.Sim(n_drones=2, sf=9, seed=3, speed=0)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        time.sleep(0.5)
        with conn:
            for t_sim, line in sim.lines():
                if t_sim > 300:
                    break
                conn.sendall((line + "\n").encode())
        srv.close()

    th = threading.Thread(target=serve, daemon=True)
    th.start()
    log = gd.GatewayLog()
    gd.read_port(f"socket://127.0.0.1:{port}", 20, log, echo=False)
    th.join(5)
    s = log.write(tmp_path)
    assert s["boot"] is not None and s["boot"]["sf"] == 9
    assert (tmp_path / "pacotes.csv").exists() and (tmp_path / "brutos.jsonl").exists()
    for src, truth in sim.truth.items():
        n = s["nos"][src]
        # mensagens geradas no fim da janela podem nao ter sido transmitidas ainda: tolerancia de 2
        assert abs(n["uniq"] - truth["delivered"]) <= 2
        assert abs(n["esperado"] - truth["msgs"]) <= 2
        assert n["pdr"] == n["pdr_gateway"] or abs(n["pdr"] - n["pdr_gateway"]) < 0.02
