/* cerise_proto.c — implementacao do protocolo (ver cerise_proto.h) */
#include "cerise_proto.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ CRC */

uint16_t cp_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  if (data == NULL) return crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000u)
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      else
        crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* ------------------------------------------------------- tamanhos */

int cp_payload_len(uint8_t type) {
  switch (type) {
    case CP_MSG_HEARTBEAT: return 8;
    case CP_MSG_POS: return 14;
    case CP_MSG_AREA_ASSIGN: return 22;
    case CP_MSG_COVERAGE_DONE: return 7;
    case CP_MSG_COUNT_REPORT: return 13;
    case CP_MSG_ACK: return 6;
    case CP_MSG_AREA_POLY: return 4; /* minimo: 4 + 4n */
    case CP_MSG_TASK_BID: return 4;
    case CP_MSG_TASK_AWARD: return 3;
    default: return -1;
  }
}

int cp_packet_payload_len(const cp_packet_t *pkt) {
  if (pkt == NULL) return -1;
  if (pkt->type == CP_MSG_AREA_POLY) {
    if (pkt->p.poly.n > CP_POLY_MAX_VERTS) return -1;
    return 4 + 4 * (int)pkt->p.poly.n;
  }
  return cp_payload_len(pkt->type);
}

int cp_frame_len(uint8_t type) {
  int p = cp_payload_len(type);
  return p < 0 ? -1 : (int)(CP_HEADER_LEN + (unsigned)p + CP_CRC_LEN);
}

/* ------------------------------------------- serializacao LE */

static void put_u8(uint8_t **w, uint8_t v) { *(*w)++ = v; }
static void put_u16(uint8_t **w, uint16_t v) {
  *(*w)++ = (uint8_t)(v & 0xFFu);
  *(*w)++ = (uint8_t)(v >> 8);
}
static void put_u32(uint8_t **w, uint32_t v) {
  put_u16(w, (uint16_t)(v & 0xFFFFu));
  put_u16(w, (uint16_t)(v >> 16));
}
static void put_i16(uint8_t **w, int16_t v) { put_u16(w, (uint16_t)v); }
static void put_i32(uint8_t **w, int32_t v) { put_u32(w, (uint32_t)v); }

static uint8_t get_u8(const uint8_t **r) { return *(*r)++; }
static uint16_t get_u16(const uint8_t **r) {
  uint16_t v = (uint16_t)((*r)[0] | ((uint16_t)(*r)[1] << 8));
  *r += 2;
  return v;
}
static uint32_t get_u32(const uint8_t **r) {
  uint32_t lo = get_u16(r);
  uint32_t hi = get_u16(r);
  return lo | (hi << 16);
}
static int16_t get_i16(const uint8_t **r) { return (int16_t)get_u16(r); }
static int32_t get_i32(const uint8_t **r) { return (int32_t)get_u32(r); }

int cp_encode(const cp_packet_t *pkt, uint8_t *out, size_t cap) {
  if (pkt == NULL || out == NULL) return CP_ERR_NULL;
  if (cp_payload_len(pkt->type) < 0) return CP_ERR_TYPE;
  int plen = cp_packet_payload_len(pkt);
  if (plen < 0) return CP_ERR_LEN;
  size_t total = CP_HEADER_LEN + (size_t)plen + CP_CRC_LEN;
  if (cap < total) return CP_ERR_BUF;

  uint8_t *w = out;
  put_u8(&w, (uint8_t)((CP_PROTO_VERSION << 4) | (pkt->type & 0x0Fu)));
  put_u8(&w, pkt->flags);
  put_u8(&w, pkt->net_id);
  put_u8(&w, pkt->src);
  put_u8(&w, pkt->dst);
  put_u16(&w, pkt->seq);
  put_u8(&w, (uint8_t)plen);

  switch (pkt->type) {
    case CP_MSG_HEARTBEAT:
      put_u8(&w, pkt->p.hb.batt_pct);
      put_u16(&w, pkt->p.hb.batt_mv);
      put_u8(&w, pkt->p.hb.state);
      put_u32(&w, pkt->p.hb.uptime_s);
      break;
    case CP_MSG_POS:
      put_i32(&w, pkt->p.pos.lat_e7);
      put_i32(&w, pkt->p.pos.lon_e7);
      put_i16(&w, pkt->p.pos.alt_dm);
      put_u8(&w, pkt->p.pos.batt_pct);
      put_u16(&w, pkt->p.pos.people);
      put_u8(&w, pkt->p.pos.area_id);
      break;
    case CP_MSG_AREA_ASSIGN:
      put_u8(&w, pkt->p.area.area_id);
      put_u8(&w, pkt->p.area.task_id);
      put_i32(&w, pkt->p.area.lat_min_e7);
      put_i32(&w, pkt->p.area.lon_min_e7);
      put_i32(&w, pkt->p.area.lat_max_e7);
      put_i32(&w, pkt->p.area.lon_max_e7);
      put_i16(&w, pkt->p.area.alt_dm);
      put_u16(&w, pkt->p.area.spacing_dm);
      break;
    case CP_MSG_COVERAGE_DONE:
      put_u8(&w, pkt->p.done.area_id);
      put_u8(&w, pkt->p.done.task_id);
      put_u8(&w, pkt->p.done.coverage_pct);
      put_u16(&w, pkt->p.done.duration_s);
      put_u16(&w, pkt->p.done.people_total);
      break;
    case CP_MSG_COUNT_REPORT:
      put_u8(&w, pkt->p.count.area_id);
      put_u8(&w, pkt->p.count.task_id);
      put_u16(&w, pkt->p.count.people);
      put_u8(&w, pkt->p.count.confidence_pct);
      put_i32(&w, pkt->p.count.lat_e7);
      put_i32(&w, pkt->p.count.lon_e7);
      break;
    case CP_MSG_ACK:
      put_u16(&w, pkt->p.ack.acked_seq);
      put_u8(&w, pkt->p.ack.acked_type);
      put_u8(&w, pkt->p.ack.status);
      put_u8(&w, pkt->p.ack.rssi_neg);
      put_u8(&w, (uint8_t)pkt->p.ack.snr_qdb);
      break;
    case CP_MSG_AREA_POLY:
      put_u8(&w, pkt->p.poly.area_id);
      put_u8(&w, pkt->p.poly.task_id);
      put_u8(&w, pkt->p.poly.frag);
      put_u8(&w, pkt->p.poly.n);
      for (unsigned i = 0; i < pkt->p.poly.n; i++) {
        put_i16(&w, pkt->p.poly.dx[i]);
        put_i16(&w, pkt->p.poly.dy[i]);
      }
      break;
    case CP_MSG_TASK_BID:
      put_u8(&w, pkt->p.bid.task_id);
      put_u8(&w, pkt->p.bid.area_id);
      put_u16(&w, pkt->p.bid.cost_ds);
      break;
    case CP_MSG_TASK_AWARD:
      put_u8(&w, pkt->p.award.task_id);
      put_u8(&w, pkt->p.award.area_id);
      put_u8(&w, pkt->p.award.winner);
      break;
    default:
      return CP_ERR_TYPE;
  }

  uint16_t crc = cp_crc16(out, CP_HEADER_LEN + (size_t)plen);
  put_u16(&w, crc);
  return (int)(w - out);
}

int cp_decode(const uint8_t *in, size_t len, cp_packet_t *pkt) {
  if (in == NULL || pkt == NULL) return CP_ERR_NULL;
  if (len < CP_HEADER_LEN + CP_CRC_LEN) return CP_ERR_SHORT;
  if (len > CP_MAX_FRAME) return CP_ERR_LEN;

  /* CRC primeiro: qualquer corrupcao e reportada como CRC */
  uint16_t rx_crc = (uint16_t)(in[len - 2] | ((uint16_t)in[len - 1] << 8));
  if (cp_crc16(in, len - CP_CRC_LEN) != rx_crc) return CP_ERR_CRC;

  uint8_t ver = (uint8_t)(in[0] >> 4);
  uint8_t type = (uint8_t)(in[0] & 0x0Fu);
  if (ver != CP_PROTO_VERSION) return CP_ERR_VERSION;
  int plen = cp_payload_len(type);
  if (plen < 0) return CP_ERR_TYPE;
  if (type == CP_MSG_AREA_POLY) { /* tamanho variavel: 4 + 4n */
    if (len < CP_HEADER_LEN + 4u + CP_CRC_LEN || in[CP_HEADER_LEN + 3] > CP_POLY_MAX_VERTS) return CP_ERR_LEN;
    plen = 4 + 4 * (int)in[CP_HEADER_LEN + 3];
  }
  if (in[7] != (uint8_t)plen || len != CP_HEADER_LEN + (size_t)plen + CP_CRC_LEN) return CP_ERR_LEN;

  memset(pkt, 0, sizeof(*pkt));
  const uint8_t *r = in;
  (void)get_u8(&r);
  pkt->version = ver;
  pkt->type = type;
  pkt->flags = get_u8(&r);
  pkt->net_id = get_u8(&r);
  pkt->src = get_u8(&r);
  pkt->dst = get_u8(&r);
  pkt->seq = get_u16(&r);
  (void)get_u8(&r); /* len */

  switch (type) {
    case CP_MSG_HEARTBEAT:
      pkt->p.hb.batt_pct = get_u8(&r);
      pkt->p.hb.batt_mv = get_u16(&r);
      pkt->p.hb.state = get_u8(&r);
      pkt->p.hb.uptime_s = get_u32(&r);
      break;
    case CP_MSG_POS:
      pkt->p.pos.lat_e7 = get_i32(&r);
      pkt->p.pos.lon_e7 = get_i32(&r);
      pkt->p.pos.alt_dm = get_i16(&r);
      pkt->p.pos.batt_pct = get_u8(&r);
      pkt->p.pos.people = get_u16(&r);
      pkt->p.pos.area_id = get_u8(&r);
      break;
    case CP_MSG_AREA_ASSIGN:
      pkt->p.area.area_id = get_u8(&r);
      pkt->p.area.task_id = get_u8(&r);
      pkt->p.area.lat_min_e7 = get_i32(&r);
      pkt->p.area.lon_min_e7 = get_i32(&r);
      pkt->p.area.lat_max_e7 = get_i32(&r);
      pkt->p.area.lon_max_e7 = get_i32(&r);
      pkt->p.area.alt_dm = get_i16(&r);
      pkt->p.area.spacing_dm = get_u16(&r);
      break;
    case CP_MSG_COVERAGE_DONE:
      pkt->p.done.area_id = get_u8(&r);
      pkt->p.done.task_id = get_u8(&r);
      pkt->p.done.coverage_pct = get_u8(&r);
      pkt->p.done.duration_s = get_u16(&r);
      pkt->p.done.people_total = get_u16(&r);
      break;
    case CP_MSG_COUNT_REPORT:
      pkt->p.count.area_id = get_u8(&r);
      pkt->p.count.task_id = get_u8(&r);
      pkt->p.count.people = get_u16(&r);
      pkt->p.count.confidence_pct = get_u8(&r);
      pkt->p.count.lat_e7 = get_i32(&r);
      pkt->p.count.lon_e7 = get_i32(&r);
      break;
    case CP_MSG_ACK:
      pkt->p.ack.acked_seq = get_u16(&r);
      pkt->p.ack.acked_type = get_u8(&r);
      pkt->p.ack.status = get_u8(&r);
      pkt->p.ack.rssi_neg = get_u8(&r);
      pkt->p.ack.snr_qdb = (int8_t)get_u8(&r);
      break;
    case CP_MSG_AREA_POLY:
      pkt->p.poly.area_id = get_u8(&r);
      pkt->p.poly.task_id = get_u8(&r);
      pkt->p.poly.frag = get_u8(&r);
      pkt->p.poly.n = get_u8(&r);
      for (unsigned i = 0; i < pkt->p.poly.n; i++) {
        pkt->p.poly.dx[i] = get_i16(&r);
        pkt->p.poly.dy[i] = get_i16(&r);
      }
      break;
    case CP_MSG_TASK_BID:
      pkt->p.bid.task_id = get_u8(&r);
      pkt->p.bid.area_id = get_u8(&r);
      pkt->p.bid.cost_ds = get_u16(&r);
      break;
    case CP_MSG_TASK_AWARD:
      pkt->p.award.task_id = get_u8(&r);
      pkt->p.award.area_id = get_u8(&r);
      pkt->p.award.winner = get_u8(&r);
      break;
    default:
      return CP_ERR_TYPE;
  }
  return CP_OK;
}

/* --------------------------------------------------------- nomes */

const char *cp_type_name(uint8_t type) {
  switch (type) {
    case CP_MSG_HEARTBEAT: return "HEARTBEAT";
    case CP_MSG_POS: return "POS";
    case CP_MSG_AREA_ASSIGN: return "AREA_ASSIGN";
    case CP_MSG_COVERAGE_DONE: return "COVERAGE_DONE";
    case CP_MSG_COUNT_REPORT: return "COUNT_REPORT";
    case CP_MSG_ACK: return "ACK";
    case CP_MSG_AREA_POLY: return "AREA_POLY";
    case CP_MSG_TASK_BID: return "TASK_BID";
    case CP_MSG_TASK_AWARD: return "TASK_AWARD";
    default: return "UNKNOWN";
  }
}

const char *cp_err_name(int err) {
  switch (err) {
    case CP_OK: return "OK";
    case CP_ERR_NULL: return "NULL";
    case CP_ERR_SHORT: return "SHORT";
    case CP_ERR_VERSION: return "VERSION";
    case CP_ERR_TYPE: return "TYPE";
    case CP_ERR_LEN: return "LEN";
    case CP_ERR_CRC: return "CRC";
    case CP_ERR_BUF: return "BUF";
    default: return "?";
  }
}

const char *cp_state_name(uint8_t state) {
  switch (state) {
    case CP_STATE_IDLE: return "IDLE";
    case CP_STATE_ARMED: return "ARMED";
    case CP_STATE_MISSION: return "MISSION";
    case CP_STATE_RTL: return "RTL";
    case CP_STATE_LANDED: return "LANDED";
    case CP_STATE_FAILSAFE: return "FAILSAFE";
    default: return "?";
  }
}

static const char *ack_status_name(uint8_t s) {
  switch (s) {
    case CP_ACK_OK: return "OK";
    case CP_ACK_DUP: return "DUP";
    case CP_ACK_REJECTED: return "REJECTED";
    default: return "?";
  }
}

/* ------------------------------------------------------------ ACK */

static int round_q4(float snr_db) {
  float x = snr_db * 4.0f;
  return (int)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

void cp_make_ack(const cp_packet_t *rx, uint8_t my_addr, uint16_t my_seq, int rssi_dbm,
                 float snr_db, uint8_t status, cp_packet_t *ack) {
  if (rx == NULL || ack == NULL) return;
  memset(ack, 0, sizeof(*ack));
  ack->version = CP_PROTO_VERSION;
  ack->type = CP_MSG_ACK;
  ack->flags = 0;
  ack->net_id = rx->net_id;
  ack->src = my_addr;
  ack->dst = rx->src;
  ack->seq = my_seq;
  ack->p.ack.acked_seq = rx->seq;
  ack->p.ack.acked_type = rx->type;
  ack->p.ack.status = status;
  int neg = -rssi_dbm;
  if (neg < 0) neg = 0;
  if (neg > 255) neg = 255;
  ack->p.ack.rssi_neg = (uint8_t)neg;
  int q = round_q4(snr_db);
  if (q < -128) q = -128;
  if (q > 127) q = 127;
  ack->p.ack.snr_qdb = (int8_t)q;
}

/* ---------------------------------------------------------- dedup */

void cp_dedup_init(cp_dedup_t *d) {
  if (d) memset(d, 0, sizeof(*d));
}

bool cp_dedup_is_new(cp_dedup_t *d, uint8_t src, uint16_t seq) {
  if (d == NULL) return true;
  uint8_t mask = (uint8_t)(1u << (src & 7u));
  if (!(d->seen[src >> 3] & mask)) {
    d->seen[src >> 3] |= mask;
    d->last_seq[src] = seq;
    return true;
  }
  if (d->last_seq[src] == seq) return false;
  d->last_seq[src] = seq;
  return true;
}

/* ---------------------------------------------------- tempo no ar */

uint32_t cp_airtime_us(uint8_t sf, uint32_t bw_hz, uint8_t cr_denom, uint16_t preamble_len,
                       uint16_t payload_len, bool explicit_header, bool crc_on, int ldro) {
  if (sf < 6 || sf > 12 || bw_hz == 0 || cr_denom < 5 || cr_denom > 8) return 0;
  /* Tsym = 2^SF / BW ; LDRO automatico quando Tsym >= 16 ms */
  uint64_t tsym_num = (uint64_t)1u << sf; /* em unidades de 1/BW s */
  int de = ldro;
  if (de < 0) de = ((tsym_num * 1000u) / bw_hz >= 16u) ? 1 : 0;
  int ih = explicit_header ? 0 : 1;
  int crc = crc_on ? 1 : 0;
  int cr = (int)cr_denom - 4;

  int num = 8 * (int)payload_len - 4 * (int)sf + 28 + 16 * crc - 20 * ih;
  int den = 4 * ((int)sf - 2 * de);
  int c = 0;
  if (num > 0) c = (num + den - 1) / den;
  int payload_symb = 8 + c * (cr + 4);

  /* total de simbolos * 4 (preambulo + 4,25 simbolos de sincronismo) */
  uint64_t sym_x4 = (uint64_t)preamble_len * 4u + 17u + (uint64_t)payload_symb * 4u;
  uint64_t t_us_x = sym_x4 * tsym_num * 1000000u;
  uint64_t den_us = (uint64_t)bw_hz * 4u;
  return (uint32_t)((t_us_x + den_us / 2u) / den_us);
}

/* --------------------------------------------------------- AU915 */

uint32_t cp_au915_uplink_hz(uint8_t ch) {
  if (ch < 64) return 915200000u + 200000u * ch;
  if (ch < 72) return 915900000u + 1600000u * (uint32_t)(ch - 64);
  return 0;
}

uint32_t cp_au915_downlink_hz(uint8_t ch) { return ch < 8 ? 923300000u + 600000u * ch : 0; }

/* ----------------------------------------------------------- LiPo */

uint8_t cp_lipo_pct(uint16_t mv) {
  static const uint16_t v[] = {3300, 3500, 3600, 3650, 3700, 3750, 3800, 3900, 4000, 4100, 4200};
  static const uint8_t p[] = {0, 5, 10, 20, 30, 40, 50, 65, 80, 90, 100};
  const int n = (int)(sizeof(v) / sizeof(v[0]));
  if (mv <= v[0]) return 0;
  if (mv >= v[n - 1]) return 100;
  for (int i = 1; i < n; i++) {
    if (mv <= v[i]) {
      uint32_t span = (uint32_t)(v[i] - v[i - 1]);
      uint32_t frac = (uint32_t)(mv - v[i - 1]);
      return (uint8_t)(p[i - 1] + ((uint32_t)(p[i] - p[i - 1]) * frac + span / 2u) / span);
    }
  }
  return 100;
}

/* ----------------------------------------------------------- JSON */

typedef struct {
  char *buf;
  size_t cap;
  size_t n;
  int err;
} jw_t;

#if defined(__GNUC__)
static void jw_raw(jw_t *j, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#endif

static void jw_raw(jw_t *j, const char *fmt, ...) {
  if (j->err) return;
  va_list ap;
  va_start(ap, fmt);
  int k = vsnprintf(j->buf + j->n, j->cap - j->n, fmt, ap);
  va_end(ap);
  if (k < 0 || (size_t)k >= j->cap - j->n) {
    j->err = 1;
    return;
  }
  j->n += (size_t)k;
}

/* inteiro escalado -> decimal fixo (sem ponto flutuante, deterministico) */
static void jw_fixed(jw_t *j, const char *key, int64_t v, int decimals) {
  /* |v| <= 2^31 em todos os campos -> cabe em unsigned long (32 bits), sem %llu
   * (newlib-nano de alguns cores nao formata long long) */
  unsigned long scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10u;
  const char *sign = v < 0 ? "-" : "";
  unsigned long a = (unsigned long)(v < 0 ? -v : v);
  jw_raw(j, ",\"%s\":%s%lu.%0*lu", key, sign, a / scale, decimals, a % scale);
}

static void jw_uint(jw_t *j, const char *key, unsigned long v) { jw_raw(j, ",\"%s\":%lu", key, v); }
static void jw_str(jw_t *j, const char *key, const char *s) { jw_raw(j, ",\"%s\":\"%s\"", key, s); }

int cp_to_json(const cp_packet_t *pkt, uint32_t t_ms, int rssi_dbm, float snr_db, bool dup,
               char *out, size_t cap) {
  if (pkt == NULL || out == NULL || cap == 0) return -1;
  jw_t j = {out, cap, 0, 0};
  jw_raw(&j, "{\"t_ms\":%lu,\"type\":\"%s\"", (unsigned long)t_ms, cp_type_name(pkt->type));
  jw_uint(&j, "net", pkt->net_id);
  jw_uint(&j, "src", pkt->src);
  jw_uint(&j, "dst", pkt->dst);
  jw_uint(&j, "seq", pkt->seq);
  jw_uint(&j, "flags", pkt->flags);
  jw_raw(&j, ",\"rssi\":%d", rssi_dbm);
  jw_fixed(&j, "snr", (int64_t)round_q4(snr_db) * 25, 2);
  jw_raw(&j, ",\"dup\":%s", dup ? "true" : "false");

  switch (pkt->type) {
    case CP_MSG_HEARTBEAT:
      jw_uint(&j, "batt", pkt->p.hb.batt_pct);
      jw_uint(&j, "batt_mv", pkt->p.hb.batt_mv);
      jw_str(&j, "state", cp_state_name(pkt->p.hb.state));
      jw_uint(&j, "uptime_s", (unsigned long)pkt->p.hb.uptime_s);
      break;
    case CP_MSG_POS:
      jw_fixed(&j, "lat", pkt->p.pos.lat_e7, 7);
      jw_fixed(&j, "lon", pkt->p.pos.lon_e7, 7);
      jw_fixed(&j, "alt_m", pkt->p.pos.alt_dm, 1);
      jw_uint(&j, "batt", pkt->p.pos.batt_pct);
      jw_uint(&j, "people", pkt->p.pos.people);
      jw_uint(&j, "area", pkt->p.pos.area_id);
      break;
    case CP_MSG_AREA_ASSIGN:
      jw_uint(&j, "area", pkt->p.area.area_id);
      jw_uint(&j, "task", pkt->p.area.task_id);
      jw_fixed(&j, "lat_min", pkt->p.area.lat_min_e7, 7);
      jw_fixed(&j, "lon_min", pkt->p.area.lon_min_e7, 7);
      jw_fixed(&j, "lat_max", pkt->p.area.lat_max_e7, 7);
      jw_fixed(&j, "lon_max", pkt->p.area.lon_max_e7, 7);
      jw_fixed(&j, "alt_m", pkt->p.area.alt_dm, 1);
      jw_fixed(&j, "spacing_m", pkt->p.area.spacing_dm, 1);
      break;
    case CP_MSG_COVERAGE_DONE:
      jw_uint(&j, "area", pkt->p.done.area_id);
      jw_uint(&j, "task", pkt->p.done.task_id);
      jw_uint(&j, "coverage_pct", pkt->p.done.coverage_pct);
      jw_uint(&j, "duration_s", pkt->p.done.duration_s);
      jw_uint(&j, "people_total", pkt->p.done.people_total);
      break;
    case CP_MSG_COUNT_REPORT:
      jw_uint(&j, "area", pkt->p.count.area_id);
      jw_uint(&j, "task", pkt->p.count.task_id);
      jw_uint(&j, "people", pkt->p.count.people);
      jw_uint(&j, "conf", pkt->p.count.confidence_pct);
      jw_fixed(&j, "lat", pkt->p.count.lat_e7, 7);
      jw_fixed(&j, "lon", pkt->p.count.lon_e7, 7);
      break;
    case CP_MSG_ACK:
      jw_uint(&j, "acked_seq", pkt->p.ack.acked_seq);
      jw_str(&j, "acked_type", cp_type_name(pkt->p.ack.acked_type));
      jw_str(&j, "status", ack_status_name(pkt->p.ack.status));
      jw_raw(&j, ",\"peer_rssi\":%d", -(int)pkt->p.ack.rssi_neg);
      jw_fixed(&j, "peer_snr", (int64_t)pkt->p.ack.snr_qdb * 25, 2);
      break;
    case CP_MSG_AREA_POLY:
      jw_uint(&j, "area", pkt->p.poly.area_id);
      jw_uint(&j, "task", pkt->p.poly.task_id);
      jw_uint(&j, "frag", pkt->p.poly.frag >> 4);
      jw_uint(&j, "nfrag", pkt->p.poly.frag & 0x0Fu);
      jw_uint(&j, "n", pkt->p.poly.n);
      jw_raw(&j, ",\"verts\":[");
      for (unsigned i = 0; i < pkt->p.poly.n && i < CP_POLY_MAX_VERTS; i++)
        jw_raw(&j, "%s[%d,%d]", i ? "," : "", pkt->p.poly.dx[i], pkt->p.poly.dy[i]);
      jw_raw(&j, "]");
      break;
    case CP_MSG_TASK_BID:
      jw_uint(&j, "task", pkt->p.bid.task_id);
      jw_uint(&j, "area", pkt->p.bid.area_id);
      jw_fixed(&j, "cost_s", pkt->p.bid.cost_ds, 1);
      break;
    case CP_MSG_TASK_AWARD:
      jw_uint(&j, "task", pkt->p.award.task_id);
      jw_uint(&j, "area", pkt->p.award.area_id);
      jw_uint(&j, "winner", pkt->p.award.winner);
      break;
    default:
      break;
  }
  jw_raw(&j, "}");
  if (j.err) {
    out[0] = '\0';
    return -1;
  }
  return (int)j.n;
}
