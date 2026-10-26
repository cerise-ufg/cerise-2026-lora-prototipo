/*
 * Testes unitarios nativos da biblioteca cerise_proto (Unity).
 * Executar:  pio test -e native -v
 */
#include <string.h>
#include <unity.h>

#include "cerise_proto.h"

void setUp(void) {}
void tearDown(void) {}

/* quadro POS de referencia; bytes calculados de forma independente em Python
 * (struct + binascii.crc_hqx com init 0xFFFF) */
static const uint8_t GOLDEN_POS[24] = {0x12, 0x01, 0x2A, 0x01, 0x00, 0x34, 0x12, 0x0E,
                                       0x68, 0xD9, 0x0E, 0xF6, 0x90, 0x48, 0xA4, 0xE2,
                                       0xC5, 0x01, 0x57, 0x03, 0x00, 0x02, 0xCC, 0xC5};

static cp_packet_t golden_pos_packet(void) {
  cp_packet_t p;
  memset(&p, 0, sizeof(p));
  p.version = CP_PROTO_VERSION;
  p.type = CP_MSG_POS;
  p.flags = CP_FLAG_ACK_REQ;
  p.net_id = 42;
  p.src = 1;
  p.dst = CP_ADDR_GATEWAY;
  p.seq = 0x1234;
  p.p.pos.lat_e7 = -166799000;
  p.p.pos.lon_e7 = -492550000;
  p.p.pos.alt_dm = 453;
  p.p.pos.batt_pct = 87;
  p.p.pos.people = 3;
  p.p.pos.area_id = 2;
  return p;
}

static void base_header(cp_packet_t *p, uint8_t type) {
  memset(p, 0, sizeof(*p));
  p->version = CP_PROTO_VERSION;
  p->type = type;
  p->flags = CP_FLAG_ACK_REQ;
  p->net_id = 7;
  p->src = 3;
  p->dst = CP_ADDR_GATEWAY;
  p->seq = 65000;
}

static void assert_roundtrip(const cp_packet_t *in) {
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(in, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(cp_frame_len(in->type), n);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_OK, cp_decode(buf, (size_t)n, &out));
  /* ambos zerados com memset antes do preenchimento -> comparacao byte a byte */
  TEST_ASSERT_EQUAL_MEMORY(in, &out, sizeof(cp_packet_t));
}

static void assert_roundtrip_len(const cp_packet_t *in, int expected_len) {
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(in, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(expected_len, n);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_OK, cp_decode(buf, (size_t)n, &out));
  TEST_ASSERT_EQUAL_MEMORY(in, &out, sizeof(cp_packet_t));
}

static void recrc(uint8_t *buf, size_t len);

/* ------------------------------------------------------------ CRC */

void test_crc16_check_value(void) {
  const uint8_t s[] = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, cp_crc16(s, 9));
}

void test_crc16_empty(void) { TEST_ASSERT_EQUAL_HEX16(0xFFFF, cp_crc16((const uint8_t *)"", 0)); }

/* ------------------------------------------------------ tamanhos */

void test_payload_and_frame_lengths(void) {
  TEST_ASSERT_EQUAL_INT(8, cp_payload_len(CP_MSG_HEARTBEAT));
  TEST_ASSERT_EQUAL_INT(14, cp_payload_len(CP_MSG_POS));
  TEST_ASSERT_EQUAL_INT(22, cp_payload_len(CP_MSG_AREA_ASSIGN));
  TEST_ASSERT_EQUAL_INT(7, cp_payload_len(CP_MSG_COVERAGE_DONE));
  TEST_ASSERT_EQUAL_INT(13, cp_payload_len(CP_MSG_COUNT_REPORT));
  TEST_ASSERT_EQUAL_INT(6, cp_payload_len(CP_MSG_ACK));
  TEST_ASSERT_EQUAL_INT(-1, cp_payload_len(0));
  TEST_ASSERT_EQUAL_INT(4, cp_payload_len(CP_MSG_AREA_POLY)); /* minimo (4 + 4n) */
  TEST_ASSERT_EQUAL_INT(4, cp_payload_len(CP_MSG_TASK_BID));
  TEST_ASSERT_EQUAL_INT(3, cp_payload_len(CP_MSG_TASK_AWARD));
  TEST_ASSERT_EQUAL_INT(-1, cp_payload_len(10));
  TEST_ASSERT_EQUAL_INT(24, cp_frame_len(CP_MSG_POS));
  for (uint8_t t = CP_MSG_MIN; t <= CP_MSG_MAX; t++) {
    TEST_ASSERT_TRUE(cp_frame_len(t) <= (int)CP_MAX_FRAME);
    TEST_ASSERT_TRUE(cp_payload_len(t) <= (int)CP_MAX_PAYLOAD);
  }
}

/* ------------------------------------------------- vetor de ouro */

void test_pos_golden_vector_encode(void) {
  cp_packet_t p = golden_pos_packet();
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(&p, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(24, n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(GOLDEN_POS, buf, 24);
}

void test_pos_golden_vector_decode(void) {
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_OK, cp_decode(GOLDEN_POS, sizeof(GOLDEN_POS), &out));
  TEST_ASSERT_EQUAL_UINT8(CP_MSG_POS, out.type);
  TEST_ASSERT_EQUAL_UINT8(1, out.src);
  TEST_ASSERT_EQUAL_UINT16(0x1234, out.seq);
  TEST_ASSERT_EQUAL_INT32(-166799000, out.p.pos.lat_e7);
  TEST_ASSERT_EQUAL_INT32(-492550000, out.p.pos.lon_e7);
  TEST_ASSERT_EQUAL_INT16(453, out.p.pos.alt_dm);
  TEST_ASSERT_EQUAL_UINT8(87, out.p.pos.batt_pct);
  TEST_ASSERT_EQUAL_UINT16(3, out.p.pos.people);
  TEST_ASSERT_EQUAL_UINT8(2, out.p.pos.area_id);
}

/* ------------------------------------------- ida e volta por tipo */

void test_roundtrip_heartbeat(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_HEARTBEAT);
  p.p.hb.batt_pct = 64;
  p.p.hb.batt_mv = 3912;
  p.p.hb.state = CP_STATE_MISSION;
  p.p.hb.uptime_s = 4000000000u;
  assert_roundtrip(&p);
}

void test_roundtrip_pos(void) {
  cp_packet_t p = golden_pos_packet();
  assert_roundtrip(&p);
}

void test_roundtrip_area_assign(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_AREA_ASSIGN);
  p.src = CP_ADDR_GATEWAY;
  p.dst = 3;
  p.p.area.area_id = 200;
  p.p.area.task_id = 9;
  p.p.area.lat_min_e7 = -166850000;
  p.p.area.lon_min_e7 = -492600000;
  p.p.area.lat_max_e7 = -166750000;
  p.p.area.lon_max_e7 = -492500000;
  p.p.area.alt_dm = 600;
  p.p.area.spacing_dm = 250;
  assert_roundtrip(&p);
}

void test_roundtrip_coverage_done(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_COVERAGE_DONE);
  p.p.done.area_id = 5;
  p.p.done.task_id = 2;
  p.p.done.coverage_pct = 98;
  p.p.done.duration_s = 742;
  p.p.done.people_total = 1234;
  assert_roundtrip(&p);
}

void test_roundtrip_count_report(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_COUNT_REPORT);
  p.p.count.area_id = 5;
  p.p.count.task_id = 2;
  p.p.count.people = 17;
  p.p.count.confidence_pct = 83;
  p.p.count.lat_e7 = -166801234;
  p.p.count.lon_e7 = -492554321;
  assert_roundtrip(&p);
}

void test_roundtrip_ack(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_ACK);
  p.flags = CP_FLAG_DL_PENDING;
  p.p.ack.acked_seq = 0xFFFF;
  p.p.ack.acked_type = CP_MSG_POS;
  p.p.ack.status = CP_ACK_DUP;
  p.p.ack.rssi_neg = 121;
  p.p.ack.snr_qdb = -50;
  assert_roundtrip(&p);
}

void test_extreme_coordinates(void) {
  const int32_t lats[] = {900000000, -900000000, 0, 1, -1};
  const int32_t lons[] = {1800000000, -1800000000, 0, 1, -1};
  for (int i = 0; i < 5; i++) {
    cp_packet_t p = golden_pos_packet();
    p.p.pos.lat_e7 = lats[i];
    p.p.pos.lon_e7 = lons[i];
    p.p.pos.alt_dm = (int16_t)(i % 2 ? -32768 : 32767);
    p.p.pos.people = 65535;
    assert_roundtrip(&p);
  }
}

/* --------------------------------------------- deteccao de erros */

void test_detects_all_single_bit_flips(void) {
  uint8_t buf[24];
  for (int bit = 0; bit < 24 * 8; bit++) {
    memcpy(buf, GOLDEN_POS, 24);
    buf[bit / 8] ^= (uint8_t)(1u << (bit % 8));
    cp_packet_t out;
    TEST_ASSERT_EQUAL_INT(CP_ERR_CRC, cp_decode(buf, 24, &out));
  }
}

void test_detects_all_double_bit_flips(void) {
  uint8_t buf[24];
  int combos = 0;
  for (int b1 = 0; b1 < 24 * 8; b1++) {
    for (int b2 = b1 + 1; b2 < 24 * 8; b2++) {
      memcpy(buf, GOLDEN_POS, 24);
      buf[b1 / 8] ^= (uint8_t)(1u << (b1 % 8));
      buf[b2 / 8] ^= (uint8_t)(1u << (b2 % 8));
      cp_packet_t out;
      TEST_ASSERT_NOT_EQUAL(CP_OK, cp_decode(buf, 24, &out));
      combos++;
    }
  }
  TEST_ASSERT_EQUAL_INT(18336, combos); /* C(192,2) */
}

void test_detects_bursts_up_to_16_bits(void) {
  uint8_t buf[24];
  for (int len = 1; len <= 16; len++) {
    for (int start = 0; start + len <= 24 * 8; start++) {
      memcpy(buf, GOLDEN_POS, 24);
      /* rajada: primeiro e ultimo bit invertidos, internos alternados */
      for (int k = 0; k < len; k++) {
        if (k == 0 || k == len - 1 || (k % 2)) {
          int bit = start + k;
          buf[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        }
      }
      cp_packet_t out;
      TEST_ASSERT_NOT_EQUAL(CP_OK, cp_decode(buf, 24, &out));
    }
  }
}

static void recrc(uint8_t *buf, size_t len) {
  uint16_t c = cp_crc16(buf, len - 2);
  buf[len - 2] = (uint8_t)(c & 0xFF);
  buf[len - 1] = (uint8_t)(c >> 8);
}

void test_decode_rejects_short_and_long(void) {
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_ERR_SHORT, cp_decode(GOLDEN_POS, 9, &out));
  TEST_ASSERT_EQUAL_INT(CP_ERR_SHORT, cp_decode(GOLDEN_POS, 0, &out));
  uint8_t big[CP_MAX_FRAME + 1];
  memset(big, 0, sizeof(big));
  TEST_ASSERT_EQUAL_INT(CP_ERR_LEN, cp_decode(big, sizeof(big), &out));
}

void test_decode_rejects_bad_version(void) {
  uint8_t buf[24];
  memcpy(buf, GOLDEN_POS, 24);
  buf[0] = (uint8_t)((2u << 4) | CP_MSG_POS);
  recrc(buf, 24);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_ERR_VERSION, cp_decode(buf, 24, &out));
}

void test_decode_rejects_unknown_type(void) {
  uint8_t buf[24];
  memcpy(buf, GOLDEN_POS, 24);
  buf[0] = (uint8_t)((CP_PROTO_VERSION << 4) | 0x0F);
  recrc(buf, 24);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_ERR_TYPE, cp_decode(buf, 24, &out));
}

void test_decode_rejects_len_mismatch(void) {
  uint8_t buf[24];
  memcpy(buf, GOLDEN_POS, 24);
  buf[7] = 13; /* campo len inconsistente com o tipo */
  recrc(buf, 24);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_ERR_LEN, cp_decode(buf, 24, &out));
  /* quadro truncado com CRC valido */
  uint8_t t[23];
  memcpy(t, GOLDEN_POS, 21);
  recrc(t, 23);
  TEST_ASSERT_EQUAL_INT(CP_ERR_LEN, cp_decode(t, 23, &out));
}

void test_encode_rejects_invalid_args(void) {
  cp_packet_t p = golden_pos_packet();
  uint8_t buf[CP_MAX_FRAME];
  TEST_ASSERT_EQUAL_INT(CP_ERR_BUF, cp_encode(&p, buf, 23));
  TEST_ASSERT_EQUAL_INT(CP_ERR_NULL, cp_encode(NULL, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_INT(CP_ERR_NULL, cp_encode(&p, NULL, sizeof(buf)));
  p.type = 0;
  TEST_ASSERT_EQUAL_INT(CP_ERR_TYPE, cp_encode(&p, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_INT(CP_ERR_NULL, cp_decode(NULL, 24, &p));
}

/* ------------------------------------------------------------ ACK */

void test_make_ack(void) {
  cp_packet_t rx = golden_pos_packet();
  cp_packet_t ack;
  cp_make_ack(&rx, CP_ADDR_GATEWAY, 77, -87, 9.25f, CP_ACK_OK, &ack);
  TEST_ASSERT_EQUAL_UINT8(CP_MSG_ACK, ack.type);
  TEST_ASSERT_EQUAL_UINT8(1, ack.dst);
  TEST_ASSERT_EQUAL_UINT8(CP_ADDR_GATEWAY, ack.src);
  TEST_ASSERT_EQUAL_UINT8(42, ack.net_id);
  TEST_ASSERT_EQUAL_UINT16(77, ack.seq);
  TEST_ASSERT_EQUAL_UINT16(0x1234, ack.p.ack.acked_seq);
  TEST_ASSERT_EQUAL_UINT8(CP_MSG_POS, ack.p.ack.acked_type);
  TEST_ASSERT_EQUAL_UINT8(87, ack.p.ack.rssi_neg);
  TEST_ASSERT_EQUAL_INT8(37, ack.p.ack.snr_qdb);
  /* saturacao e arredondamento */
  cp_make_ack(&rx, 0, 0, -300, -7.3f, CP_ACK_DUP, &ack);
  TEST_ASSERT_EQUAL_UINT8(255, ack.p.ack.rssi_neg);
  TEST_ASSERT_EQUAL_INT8(-29, ack.p.ack.snr_qdb);
  cp_make_ack(&rx, 0, 0, 5, 100.0f, CP_ACK_OK, &ack);
  TEST_ASSERT_EQUAL_UINT8(0, ack.p.ack.rssi_neg);
  TEST_ASSERT_EQUAL_INT8(127, ack.p.ack.snr_qdb);
}

/* ---------------------------------------------------------- dedup */

void test_dedup(void) {
  static cp_dedup_t d;
  cp_dedup_init(&d);
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 1, 10));
  TEST_ASSERT_FALSE(cp_dedup_is_new(&d, 1, 10)); /* retransmissao */
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 1, 11));
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 2, 11)); /* outra origem */
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 1, 65535));
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 1, 0)); /* volta do contador */
  TEST_ASSERT_FALSE(cp_dedup_is_new(&d, 1, 0));
  TEST_ASSERT_TRUE(cp_dedup_is_new(&d, 255, 0));
  TEST_ASSERT_FALSE(cp_dedup_is_new(&d, 255, 0));
}

/* ---------------------------------------------------- tempo no ar */

void test_airtime_reference_values(void) {
  /* SF7/125k/CR4-5/preambulo 8/cabecalho explicito/CRC: 13 B -> 46,3 ms
   * (valor classico das calculadoras LoRaWAN para payload vazio) */
  TEST_ASSERT_EQUAL_UINT32(46336, cp_airtime_us(7, 125000, 5, 8, 13, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(41216, cp_airtime_us(7, 125000, 5, 8, 10, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(991232, cp_airtime_us(12, 125000, 5, 8, 10, true, true, -1));
  /* quadro POS (24 B) por SF */
  TEST_ASSERT_EQUAL_UINT32(61696, cp_airtime_us(7, 125000, 5, 8, 24, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(205824, cp_airtime_us(9, 125000, 5, 8, 24, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(370688, cp_airtime_us(10, 125000, 5, 8, 24, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(823296, cp_airtime_us(11, 125000, 5, 8, 24, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(1482752, cp_airtime_us(12, 125000, 5, 8, 24, true, true, -1));
  /* POS cabe em 400 ms ate SF10 */
  TEST_ASSERT_TRUE(cp_airtime_us(10, 125000, 5, 8, (uint16_t)cp_frame_len(CP_MSG_POS), true, true, -1) <= 400000u);
  TEST_ASSERT_TRUE(cp_airtime_us(11, 125000, 5, 8, (uint16_t)cp_frame_len(CP_MSG_POS), true, true, -1) > 400000u);
  /* parametros invalidos */
  TEST_ASSERT_EQUAL_UINT32(0, cp_airtime_us(13, 125000, 5, 8, 10, true, true, -1));
  TEST_ASSERT_EQUAL_UINT32(0, cp_airtime_us(7, 125000, 9, 8, 10, true, true, -1));
}

/* ---------------------------------------------------- AU915 e LiPo */

void test_au915_channel_plan(void) {
  TEST_ASSERT_EQUAL_UINT32(915200000u, cp_au915_uplink_hz(0));
  TEST_ASSERT_EQUAL_UINT32(916800000u, cp_au915_uplink_hz(8)); /* 1o canal da sub-banda 2 */
  TEST_ASSERT_EQUAL_UINT32(927800000u, cp_au915_uplink_hz(63));
  TEST_ASSERT_EQUAL_UINT32(915900000u, cp_au915_uplink_hz(64));
  TEST_ASSERT_EQUAL_UINT32(927100000u, cp_au915_uplink_hz(71));
  TEST_ASSERT_EQUAL_UINT32(0u, cp_au915_uplink_hz(72));
  TEST_ASSERT_EQUAL_UINT32(923300000u, cp_au915_downlink_hz(0));
  TEST_ASSERT_EQUAL_UINT32(927500000u, cp_au915_downlink_hz(7));
  for (uint8_t ch = 0; ch < 72; ch++) { /* todos dentro de 915-928 MHz */
    TEST_ASSERT_TRUE(cp_au915_uplink_hz(ch) >= 915000000u && cp_au915_uplink_hz(ch) <= 928000000u);
  }
}

void test_lipo_pct(void) {
  TEST_ASSERT_EQUAL_UINT8(0, cp_lipo_pct(2000));
  TEST_ASSERT_EQUAL_UINT8(0, cp_lipo_pct(3300));
  TEST_ASSERT_EQUAL_UINT8(50, cp_lipo_pct(3800));
  TEST_ASSERT_EQUAL_UINT8(58, cp_lipo_pct(3850));
  TEST_ASSERT_EQUAL_UINT8(100, cp_lipo_pct(4200));
  TEST_ASSERT_EQUAL_UINT8(100, cp_lipo_pct(5000));
}

/* ----------------------------------------------------------- JSON */

void test_json_pos(void) {
  cp_packet_t p = golden_pos_packet();
  char out[256];
  int n = cp_to_json(&p, 1234, -87, 9.25f, false, out, sizeof(out));
  const char *exp =
      "{\"t_ms\":1234,\"type\":\"POS\",\"net\":42,\"src\":1,\"dst\":0,\"seq\":4660,\"flags\":1,"
      "\"rssi\":-87,\"snr\":9.25,\"dup\":false,\"lat\":-16.6799000,\"lon\":-49.2550000,"
      "\"alt_m\":45.3,\"batt\":87,\"people\":3,\"area\":2}";
  TEST_ASSERT_EQUAL_STRING(exp, out);
  TEST_ASSERT_EQUAL_INT((int)strlen(exp), n);
}

void test_json_ack_negative_snr_and_overflow(void) {
  cp_packet_t rx = golden_pos_packet(), ack;
  cp_make_ack(&rx, 0, 5, -120, -7.3f, CP_ACK_OK, &ack);
  char out[256];
  TEST_ASSERT_TRUE(cp_to_json(&ack, 0, -60, -0.1f, true, out, sizeof(out)) > 0);
  TEST_ASSERT_NOT_NULL(strstr(out, "\"peer_rssi\":-120"));
  TEST_ASSERT_NOT_NULL(strstr(out, "\"peer_snr\":-7.25"));
  TEST_ASSERT_NOT_NULL(strstr(out, "\"snr\":0.00"));
  TEST_ASSERT_NOT_NULL(strstr(out, "\"dup\":true"));
  char small[20];
  TEST_ASSERT_EQUAL_INT(-1, cp_to_json(&ack, 0, -60, 0.0f, false, small, sizeof(small)));
  TEST_ASSERT_EQUAL_STRING("", small);
}

/* ------------------------------------- extensao da frente de drones (7-9) */

void test_extension_types_roundtrip(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_AREA_POLY);
  p.dst = CP_ADDR_BROADCAST;
  p.p.poly.area_id = 3;
  p.p.poly.task_id = 1;
  p.p.poly.frag = (uint8_t)((1u << 4) | 2u);
  p.p.poly.n = 7;
  for (int i = 0; i < 7; i++) {
    p.p.poly.dx[i] = (int16_t)(i * 10 - 30);
    p.p.poly.dy[i] = (int16_t)(i == 6 ? -32768 : i * 3);
  }
  uint8_t buf[CP_MAX_FRAME];
  TEST_ASSERT_EQUAL_INT(CP_HEADER_LEN + 4 + 28 + CP_CRC_LEN, cp_encode(&p, buf, sizeof(buf)));
  assert_roundtrip_len(&p, CP_HEADER_LEN + 4 + 28 + CP_CRC_LEN);
  p.p.poly.n = 0; /* fragmento vazio tambem e valido */
  for (int i = 0; i < 7; i++) p.p.poly.dx[i] = p.p.poly.dy[i] = 0;
  assert_roundtrip_len(&p, CP_HEADER_LEN + 4 + CP_CRC_LEN);

  base_header(&p, CP_MSG_TASK_BID);
  p.p.bid.task_id = 5;
  p.p.bid.area_id = 9;
  p.p.bid.cost_ds = 4200;
  assert_roundtrip_len(&p, CP_HEADER_LEN + 4 + CP_CRC_LEN);

  base_header(&p, CP_MSG_TASK_AWARD);
  p.p.award.task_id = 5;
  p.p.award.area_id = 9;
  p.p.award.winner = 2;
  assert_roundtrip_len(&p, CP_HEADER_LEN + 3 + CP_CRC_LEN);
}

void test_area_poly_rejects_too_many_vertices(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_AREA_POLY);
  p.p.poly.n = 8;
  uint8_t buf[CP_MAX_FRAME + 8];
  TEST_ASSERT_EQUAL_INT(CP_ERR_LEN, cp_encode(&p, buf, sizeof(buf)));
  /* quadro forjado com n = 8 e CRC valido */
  p.p.poly.n = 7;
  int n = cp_encode(&p, buf, sizeof(buf));
  buf[CP_HEADER_LEN + 3] = 8;
  recrc(buf, (size_t)n);
  cp_packet_t out;
  TEST_ASSERT_EQUAL_INT(CP_ERR_LEN, cp_decode(buf, (size_t)n, &out));
}

void test_json_extension(void) {
  cp_packet_t p;
  base_header(&p, CP_MSG_AREA_POLY);
  p.p.poly.area_id = 3;
  p.p.poly.task_id = 1;
  p.p.poly.frag = 0x02;
  p.p.poly.n = 2;
  p.p.poly.dx[0] = -5;
  p.p.poly.dy[0] = 7;
  p.p.poly.dx[1] = 120;
  p.p.poly.dy[1] = -32768;
  char out[320];
  TEST_ASSERT_TRUE(cp_to_json(&p, 1, -90, 1.0f, false, out, sizeof(out)) > 0);
  TEST_ASSERT_NOT_NULL(strstr(out, "\"type\":\"AREA_POLY\""));
  TEST_ASSERT_NOT_NULL(strstr(out, "\"frag\":0,\"nfrag\":2,\"n\":2,\"verts\":[[-5,7],[120,-32768]]"));
  base_header(&p, CP_MSG_TASK_BID);
  p.p.bid.cost_ds = 4205;
  cp_to_json(&p, 1, -90, 1.0f, false, out, sizeof(out));
  TEST_ASSERT_NOT_NULL(strstr(out, "\"cost_s\":420.5"));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_check_value);
  RUN_TEST(test_crc16_empty);
  RUN_TEST(test_payload_and_frame_lengths);
  RUN_TEST(test_pos_golden_vector_encode);
  RUN_TEST(test_pos_golden_vector_decode);
  RUN_TEST(test_roundtrip_heartbeat);
  RUN_TEST(test_roundtrip_pos);
  RUN_TEST(test_roundtrip_area_assign);
  RUN_TEST(test_roundtrip_coverage_done);
  RUN_TEST(test_roundtrip_count_report);
  RUN_TEST(test_roundtrip_ack);
  RUN_TEST(test_extreme_coordinates);
  RUN_TEST(test_detects_all_single_bit_flips);
  RUN_TEST(test_detects_all_double_bit_flips);
  RUN_TEST(test_detects_bursts_up_to_16_bits);
  RUN_TEST(test_decode_rejects_short_and_long);
  RUN_TEST(test_decode_rejects_bad_version);
  RUN_TEST(test_decode_rejects_unknown_type);
  RUN_TEST(test_decode_rejects_len_mismatch);
  RUN_TEST(test_encode_rejects_invalid_args);
  RUN_TEST(test_make_ack);
  RUN_TEST(test_dedup);
  RUN_TEST(test_airtime_reference_values);
  RUN_TEST(test_au915_channel_plan);
  RUN_TEST(test_lipo_pct);
  RUN_TEST(test_json_pos);
  RUN_TEST(test_json_ack_negative_snr_and_overflow);
  RUN_TEST(test_extension_types_roundtrip);
  RUN_TEST(test_area_poly_rejects_too_many_vertices);
  RUN_TEST(test_json_extension);
  return UNITY_END();
}
