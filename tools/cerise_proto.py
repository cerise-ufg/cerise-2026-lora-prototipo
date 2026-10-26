"""Espelho em Python do protocolo cerise_proto (firmware/lib/cerise_proto).

Usado pelas ferramentas de simulacao/dashboard e validado contra a implementacao
em C por tools/tests/test_proto_py.py (vetores gerados por tools/c/gen_vectors.c).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

PROTO_VERSION = 1
HEADER_LEN = 8
CRC_LEN = 2
MAX_PAYLOAD = 32
MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN
ADDR_GATEWAY = 0x00
ADDR_BROADCAST = 0xFF

FLAG_ACK_REQ = 0x01
FLAG_RETRY = 0x02
FLAG_DL_PENDING = 0x04
FLAG_TEST = 0x08

HEARTBEAT, POS, AREA_ASSIGN, COVERAGE_DONE, COUNT_REPORT, ACK = 1, 2, 3, 4, 5, 6
AREA_POLY, TASK_BID, TASK_AWARD = 7, 8, 9  # extensao proposta pela frente de drones (aditiva, versao 1)
POLY_MAX_VERTS = 7
TYPE_NAMES = {HEARTBEAT: "HEARTBEAT", POS: "POS", AREA_ASSIGN: "AREA_ASSIGN",
              COVERAGE_DONE: "COVERAGE_DONE", COUNT_REPORT: "COUNT_REPORT", ACK: "ACK",
              AREA_POLY: "AREA_POLY", TASK_BID: "TASK_BID", TASK_AWARD: "TASK_AWARD"}
STATE_NAMES = {0: "IDLE", 1: "ARMED", 2: "MISSION", 3: "RTL", 4: "LANDED", 5: "FAILSAFE"}
ACK_STATUS = {0: "OK", 1: "DUP", 2: "REJECTED"}

# layout de cada payload: (formato struct little-endian, nomes dos campos)
LAYOUTS = {
    HEARTBEAT: ("<BHBI", ("batt_pct", "batt_mv", "state", "uptime_s")),
    POS: ("<iihBHB", ("lat_e7", "lon_e7", "alt_dm", "batt_pct", "people", "area_id")),
    AREA_ASSIGN: ("<BBiiiihH", ("area_id", "task_id", "lat_min_e7", "lon_min_e7", "lat_max_e7",
                                "lon_max_e7", "alt_dm", "spacing_dm")),
    COVERAGE_DONE: ("<BBBHH", ("area_id", "task_id", "coverage_pct", "duration_s", "people_total")),
    COUNT_REPORT: ("<BBHBii", ("area_id", "task_id", "people", "confidence_pct", "lat_e7", "lon_e7")),
    ACK: ("<HBBBb", ("acked_seq", "acked_type", "status", "rssi_neg", "snr_qdb")),
    AREA_POLY: ("<BBBB", ("area_id", "task_id", "frag", "n")),  # + n x (dx, dy) int16 -> campo "verts"
    TASK_BID: ("<BBH", ("task_id", "area_id", "cost_ds")),
    TASK_AWARD: ("<BBB", ("task_id", "area_id", "winner")),
}


class ProtoError(ValueError):
    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def payload_len(msg_type: int) -> int:
    """Tamanho fixo do payload (minimo, 4 B, para AREA_POLY)."""
    lay = LAYOUTS.get(msg_type)
    return struct.calcsize(lay[0]) if lay else -1


def frame_len(msg_type: int) -> int:
    p = payload_len(msg_type)
    return -1 if p < 0 else HEADER_LEN + p + CRC_LEN


@dataclass
class Packet:
    type: int
    src: int = 1
    dst: int = ADDR_GATEWAY
    seq: int = 0
    flags: int = FLAG_ACK_REQ
    net_id: int = 42
    fields: dict = field(default_factory=dict)
    version: int = PROTO_VERSION


def encode(pkt: Packet) -> bytes:
    lay = LAYOUTS.get(pkt.type)
    if lay is None:
        raise ProtoError("TYPE")
    fmt, names = lay
    payload = struct.pack(fmt, *[pkt.fields.get(n, 0) for n in names])
    if pkt.type == AREA_POLY:
        verts = pkt.fields.get("verts", [])
        if len(verts) != pkt.fields.get("n", 0) or len(verts) > POLY_MAX_VERTS:
            raise ProtoError("LEN")
        payload += b"".join(struct.pack("<hh", dx, dy) for dx, dy in verts)
    hdr = struct.pack("<BBBBBHB", (PROTO_VERSION << 4) | (pkt.type & 0x0F), pkt.flags, pkt.net_id,
                      pkt.src, pkt.dst, pkt.seq & 0xFFFF, len(payload))
    body = hdr + payload
    return body + struct.pack("<H", crc16(body))


def decode(frame: bytes) -> Packet:
    if len(frame) < HEADER_LEN + CRC_LEN:
        raise ProtoError("SHORT")
    if len(frame) > MAX_FRAME:
        raise ProtoError("LEN")
    if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ProtoError("CRC")
    ver, mtype = frame[0] >> 4, frame[0] & 0x0F
    if ver != PROTO_VERSION:
        raise ProtoError("VERSION")
    lay = LAYOUTS.get(mtype)
    if lay is None:
        raise ProtoError("TYPE")
    plen = struct.calcsize(lay[0])
    if mtype == AREA_POLY:
        if len(frame) < HEADER_LEN + 4 + CRC_LEN or frame[HEADER_LEN + 3] > POLY_MAX_VERTS:
            raise ProtoError("LEN")
        plen = 4 + 4 * frame[HEADER_LEN + 3]
    if frame[7] != plen or len(frame) != HEADER_LEN + plen + CRC_LEN:
        raise ProtoError("LEN")
    _, flags, net, src, dst, seq, _ = struct.unpack("<BBBBBHB", frame[:HEADER_LEN])
    base = struct.calcsize(lay[0])
    fields = dict(zip(lay[1], struct.unpack(lay[0], frame[HEADER_LEN:HEADER_LEN + base])))
    if mtype == AREA_POLY:
        fields["verts"] = [struct.unpack_from("<hh", frame, HEADER_LEN + 4 + 4 * i) for i in range(fields["n"])]
    return Packet(type=mtype, src=src, dst=dst, seq=seq, flags=flags, net_id=net, fields=fields, version=ver)


def make_ack(rx: Packet, my_addr: int, my_seq: int, rssi_dbm: int, snr_db: float, status: int = 0) -> Packet:
    q = _round_q4(snr_db)
    return Packet(type=ACK, src=my_addr, dst=rx.src, seq=my_seq, flags=0, net_id=rx.net_id,
                  fields={"acked_seq": rx.seq, "acked_type": rx.type, "status": status,
                          "rssi_neg": max(0, min(255, -rssi_dbm)), "snr_qdb": max(-128, min(127, q))})


def _round_q4(snr_db: float) -> int:
    # replica o arredondamento em float32 do firmware: (int)(x*4 +/- 0.5f)
    f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]  # noqa: E731
    x = f32(f32(snr_db) * 4.0)
    return int(f32(x + 0.5)) if x >= 0 else int(f32(x - 0.5))


def airtime_us(sf: int, bw_hz: int = 125_000, cr_denom: int = 5, preamble: int = 8, payload: int = 24,
               explicit_header: bool = True, crc_on: bool = True, ldro: int = -1) -> int:
    """Tempo no ar (Semtech AN1200.13), mesmo algoritmo inteiro do firmware."""
    if not (6 <= sf <= 12) or bw_hz <= 0 or not (5 <= cr_denom <= 8):
        return 0
    tsym_num = 1 << sf
    de = ldro if ldro >= 0 else (1 if (tsym_num * 1000) // bw_hz >= 16 else 0)
    ih = 0 if explicit_header else 1
    num = 8 * payload - 4 * sf + 28 + 16 * int(crc_on) - 20 * ih
    den = 4 * (sf - 2 * de)
    c = -(-num // den) if num > 0 else 0
    payload_symb = 8 + c * cr_denom  # (CR + 4) com CR = cr_denom - 4
    sym_x4 = preamble * 4 + 17 + payload_symb * 4
    t_x = sym_x4 * tsym_num * 1_000_000
    d = bw_hz * 4
    return (t_x + d // 2) // d


def airtime_ms_float(sf: int, bw_hz: int = 125_000, cr_denom: int = 5, preamble: int = 8, payload: int = 24,
                     explicit_header: bool = True, crc_on: bool = True, ldro: int = -1) -> float:
    """Mesma formula em ponto flutuante (referencia independente)."""
    tsym = (2 ** sf) / bw_hz
    de = ldro if ldro >= 0 else (1 if tsym >= 0.016 else 0)
    ih = 0 if explicit_header else 1
    n_payload = 8 + max(math.ceil((8 * payload - 4 * sf + 28 + 16 * int(crc_on) - 20 * ih)
                                  / (4 * (sf - 2 * de))) * cr_denom, 0)
    return ((preamble + 4.25) + n_payload) * tsym * 1000.0


def au915_uplink_hz(ch: int) -> int:
    if ch < 64:
        return 915_200_000 + 200_000 * ch
    if ch < 72:
        return 915_900_000 + 1_600_000 * (ch - 64)
    return 0


def _fixed(v: int, decimals: int) -> str:
    scale = 10 ** decimals
    sign = "-" if v < 0 else ""
    a = abs(v)
    return f"{sign}{a // scale}.{a % scale:0{decimals}d}"


def to_json(pkt: Packet, t_ms: int, rssi_dbm: int, snr_db: float, dup: bool) -> str:
    """Mesma linha JSON impressa pelo gateway (cp_to_json)."""
    f = pkt.fields
    parts = [f'{{"t_ms":{t_ms},"type":"{TYPE_NAMES.get(pkt.type, "UNKNOWN")}"',
             f'"net":{pkt.net_id}', f'"src":{pkt.src}', f'"dst":{pkt.dst}', f'"seq":{pkt.seq}',
             f'"flags":{pkt.flags}', f'"rssi":{rssi_dbm}', f'"snr":{_fixed(_round_q4(snr_db) * 25, 2)}',
             f'"dup":{"true" if dup else "false"}']
    t = pkt.type
    if t == HEARTBEAT:
        parts += [f'"batt":{f["batt_pct"]}', f'"batt_mv":{f["batt_mv"]}',
                  f'"state":"{STATE_NAMES.get(f["state"], "?")}"', f'"uptime_s":{f["uptime_s"]}']
    elif t == POS:
        parts += [f'"lat":{_fixed(f["lat_e7"], 7)}', f'"lon":{_fixed(f["lon_e7"], 7)}',
                  f'"alt_m":{_fixed(f["alt_dm"], 1)}', f'"batt":{f["batt_pct"]}', f'"people":{f["people"]}',
                  f'"area":{f["area_id"]}']
    elif t == AREA_ASSIGN:
        parts += [f'"area":{f["area_id"]}', f'"task":{f["task_id"]}',
                  f'"lat_min":{_fixed(f["lat_min_e7"], 7)}', f'"lon_min":{_fixed(f["lon_min_e7"], 7)}',
                  f'"lat_max":{_fixed(f["lat_max_e7"], 7)}', f'"lon_max":{_fixed(f["lon_max_e7"], 7)}',
                  f'"alt_m":{_fixed(f["alt_dm"], 1)}', f'"spacing_m":{_fixed(f["spacing_dm"], 1)}']
    elif t == COVERAGE_DONE:
        parts += [f'"area":{f["area_id"]}', f'"task":{f["task_id"]}', f'"coverage_pct":{f["coverage_pct"]}',
                  f'"duration_s":{f["duration_s"]}', f'"people_total":{f["people_total"]}']
    elif t == COUNT_REPORT:
        parts += [f'"area":{f["area_id"]}', f'"task":{f["task_id"]}', f'"people":{f["people"]}',
                  f'"conf":{f["confidence_pct"]}', f'"lat":{_fixed(f["lat_e7"], 7)}',
                  f'"lon":{_fixed(f["lon_e7"], 7)}']
    elif t == ACK:
        parts += [f'"acked_seq":{f["acked_seq"]}', f'"acked_type":"{TYPE_NAMES.get(f["acked_type"], "UNKNOWN")}"',
                  f'"status":"{ACK_STATUS.get(f["status"], "?")}"', f'"peer_rssi":{-f["rssi_neg"]}',
                  f'"peer_snr":{_fixed(f["snr_qdb"] * 25, 2)}']
    elif t == AREA_POLY:
        verts = ",".join(f"[{dx},{dy}]" for dx, dy in f["verts"])
        parts += [f'"area":{f["area_id"]}', f'"task":{f["task_id"]}', f'"frag":{f["frag"] >> 4}',
                  f'"nfrag":{f["frag"] & 0x0F}', f'"n":{f["n"]}', f'"verts":[{verts}]']
    elif t == TASK_BID:
        parts += [f'"task":{f["task_id"]}', f'"area":{f["area_id"]}', f'"cost_s":{_fixed(f["cost_ds"], 1)}']
    elif t == TASK_AWARD:
        parts += [f'"task":{f["task_id"]}', f'"area":{f["area_id"]}', f'"winner":{f["winner"]}']
    return ",".join(parts) + "}"
