"""Valida o espelho Python do protocolo contra a implementacao em C.

Os vetores vem de tools/c/gen_vectors.c. Se houver gcc no PATH o gerador e
recompilado e executado a cada teste; senao usa tools/tests/vectors.tsv versionado.
"""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import cerise_proto as cp  # noqa: E402

N_VECTORS = 3000


def _load_vectors():
    gcc = shutil.which("gcc")
    vec_file = HERE / "vectors.tsv"
    if gcc:
        src = ROOT / "firmware" / "lib" / "cerise_proto" / "src"
        with tempfile.TemporaryDirectory() as td:
            exe = Path(td) / ("gen_vectors.exe" if os.name == "nt" else "gen_vectors")
            subprocess.run([gcc, "-O2", "-std=gnu11", "-Wall", "-Wextra", f"-I{src}", str(TOOLS / "c" / "gen_vectors.c"),
                            str(src / "cerise_proto.c"), "-o", str(exe)], check=True)
            out = subprocess.run([str(exe), str(N_VECTORS)], check=True, capture_output=True, text=True).stdout
        vec_file.write_text(out, encoding="utf-8")
    return vec_file.read_text(encoding="utf-8").splitlines()


LINES = _load_vectors()
P_LINES = [ln.split("\t") for ln in LINES if ln.startswith("P\t")]
C_LINES = [ln.split("\t") for ln in LINES if ln.startswith("C\t")]
A_LINES = [ln.split("\t") for ln in LINES if ln.startswith("A\t")]


def _packet_from(row):
    _, t, flags, net, src, dst, seq, fields = row[:8]
    t = int(t)
    names = cp.LAYOUTS[t][1]
    vals = [int(v) for v in fields.split(",")]
    f = dict(zip(names, vals))
    if t == cp.AREA_POLY:
        rest = vals[4:]
        f["verts"] = [(rest[2 * i], rest[2 * i + 1]) for i in range(f["n"])]
    return cp.Packet(type=t, flags=int(flags), net_id=int(net), src=int(src), dst=int(dst), seq=int(seq), fields=f)


def test_vector_counts():
    assert len(P_LINES) >= 1000
    assert len(C_LINES) == len(P_LINES)
    assert len(A_LINES) == 6 * 3 * 4 * 65 * 2


def test_encode_matches_c():
    for row in P_LINES:
        assert cp.encode(_packet_from(row)).hex() == row[12]


def test_decode_matches_c():
    for row in P_LINES:
        pkt = cp.decode(bytes.fromhex(row[12]))
        ref = _packet_from(row)
        assert (pkt.type, pkt.flags, pkt.net_id, pkt.src, pkt.dst, pkt.seq) == \
               (ref.type, ref.flags, ref.net_id, ref.src, ref.dst, ref.seq)
        assert pkt.fields == ref.fields


def test_json_matches_c():
    for row in P_LINES:
        t_ms, rssi, snr, dup = int(row[8]), int(row[9]), float(row[10]), row[11] == "1"
        assert cp.to_json(_packet_from(row), t_ms, rssi, snr, dup) == row[13]


def test_corruption_codes_match_c():
    for _, hx, err in C_LINES:
        try:
            cp.decode(bytes.fromhex(hx))
            got = "OK"
        except cp.ProtoError as e:
            got = e.code
        assert got == err
    # nenhum quadro corrompido com 1-4 bits passou como valido
    assert all(err != "OK" for _, _, err in C_LINES)


def test_airtime_matches_c_and_float_formula():
    for _, sf, bw, cr, pre, pl, eh, crc, us in A_LINES:
        args = dict(sf=int(sf), bw_hz=int(bw), cr_denom=int(cr), preamble=int(pre), payload=int(pl),
                    explicit_header=eh == "1", crc_on=crc == "1")
        assert cp.airtime_us(**args) == int(us)
        assert abs(cp.airtime_ms_float(**args) * 1000 - int(us)) <= 1.0


@pytest.mark.parametrize("t,expected", [(cp.HEARTBEAT, 18), (cp.POS, 24), (cp.AREA_ASSIGN, 32),
                                        (cp.COVERAGE_DONE, 17), (cp.COUNT_REPORT, 23), (cp.ACK, 16),
                                        (cp.AREA_POLY, 14), (cp.TASK_BID, 14), (cp.TASK_AWARD, 13)])
def test_frame_lengths(t, expected):
    assert cp.frame_len(t) == expected


def test_types_seen_in_vectors():
    assert {int(r[1]) for r in P_LINES} == set(range(1, 10))


def test_compat_with_drones_front():
    """Quadros de exemplo da frente de drones (cerise_drones/protocolo.py) decodificam aqui."""
    drones = ROOT.parent / "DRONES_SEGMENTACAO"
    if not (drones / "cerise_drones" / "protocolo.py").exists():
        pytest.skip("frente de drones nao encontrada neste computador")
    import importlib.util
    spec = importlib.util.spec_from_file_location("protocolo_drones", drones / "cerise_drones" / "protocolo.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for nome, quadro in mod.exemplos().items():
        pkt = cp.decode(quadro)
        assert cp.encode(pkt) == quadro, nome


def test_crc_check_value():
    assert cp.crc16(b"123456789") == 0x29B1
