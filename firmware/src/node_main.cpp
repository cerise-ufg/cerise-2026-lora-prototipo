// =====================================================================
// CERISE 2026 — firmware LoRa, papel NODE (lado drone)
//
// - telemetria periodica (POS; HEARTBEAT a cada HEARTBEAT_EVERY ciclos)
// - COUNT_REPORT quando a contagem de pessoas cresce; COVERAGE_DONE ao fim da area
// - ACK + ate MAX_RETRIES retentativas com recuo exponencial aleatorio
// - numero de sequencia por mensagem (retransmissao reutiliza o seq, flag RETRY)
// - recebe AREA_ASSIGN (downlink) na janela aberta apos um ACK com DL_PENDING
// - log em linhas JSON na serial (115200) + status no OLED
//
// Fonte da posicao: comando serial "POS" vindo do computador de bordo (prioridade),
// GPS opcional (-DUSE_GPS) ou trajetoria sintetica (mission_sim).
// =====================================================================
#include <Arduino.h>
#include <stdarg.h>

#include "cerise_proto.h"
#include "common/board.h"
#include "common/config.h"
#include "common/display.h"
#include "common/mission_sim.h"
#include "common/radio_link.h"
#include "common/serial_cmd.h"

#ifdef USE_GPS
#include <TinyGPSPlus.h>
static TinyGPSPlus gps;
static HardwareSerial GpsSerial(1);
#endif

enum NodeState { ST_IDLE, ST_WAIT_ACK, ST_BACKOFF, ST_WAIT_DL };

static struct {
  uint32_t msgs, frames, acked, dropped, retries, dl_rx, rx_err, blocked, cad_busy;
} st = {};

static NodeState g_state = ST_IDLE;
static cp_packet_t g_cur;
static uint8_t g_attempt = 0;
static uint32_t g_t_first = 0, g_t_tx_end = 0, g_deadline = 0;
static uint16_t g_seq = 0;
static uint16_t g_ack_seq = 0;  // contador separado para ACKs de downlink
static uint32_t g_next_tel = 0, g_cycle = 0;
static uint32_t g_period = TELEMETRY_PERIOD_MS;
static uint8_t g_max_retries = MAX_RETRIES;
static uint32_t g_dwell_ms = DWELL_LIMIT_MS;
static bool g_cad = USE_CAD;  // escuta antes de transmitir (Channel Activity Detection)

static cp_packet_t g_q[4];
static uint8_t g_qh = 0, g_qn = 0;

static int g_rssi = 0, g_gw_rssi = 0;
static float g_snr = 0, g_gw_snr = 0;
static uint32_t g_rtt = 0;

static bool g_host_pos = false;
static double g_hlat = 0, g_hlon = 0;
static float g_halt = 0;
static uint32_t g_host_ms = 0;

static uint32_t g_rt_left = 0, g_rt_interval = 1000, g_rt_next = 0, g_rt_sent = 0;
static uint16_t g_rt_id = 0;

static bool g_have_dl = false;
static uint16_t g_last_dl_seq = 0;
static bool g_done_sent = false;
static uint32_t g_done_ms = 0;
static uint16_t g_people_reported = 0;
static uint8_t g_fstate = CP_STATE_MISSION;

static SerialLine g_line;
#if defined(HOST_UART1) && !defined(USE_GPS)
// UART1 (GPIO47/48 na Heltec V3) para o computador de bordo do drone: mesmos comandos e mesmo log
static SerialLine g_line1;
#define HOST_PORT Serial1
#endif

// log em linhas JSON na USB (e espelhado na UART do computador de bordo, se habilitada)
static void jlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void jlog(const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  Serial.write((const uint8_t *)buf, n);
#ifdef HOST_PORT
  HOST_PORT.write((const uint8_t *)buf, n);
#endif
}

// ---------------------------------------------------------------- utilidades

static uint32_t frame_toa_ms(uint8_t type) {
  const LoraParams &p = radio_params();
  return (cp_airtime_us(p.sf, (uint32_t)(p.bw_khz * 1000), p.cr_denom, p.preamble,
                        (uint16_t)cp_frame_len(type), true, true, -1) +
          999) /
         1000;
}

static uint32_t ack_timeout_ms() { return frame_toa_ms(CP_MSG_ACK) + GW_TURNAROUND_MS + ACK_MARGIN_MS; }

static uint8_t battery(uint16_t *mv_out) {
  uint16_t mv = board_vbat_mv();
  if (mv == 0) {  // sem bateria: descarga simulada (0,5 %/min) para exercitar os campos
    uint32_t pct = millis() / 120000UL;
    pct = pct > 80 ? 20 : 100 - pct;
    *mv_out = 0;
    return (uint8_t)pct;
  }
  *mv_out = mv;
  return cp_lipo_pct(mv);
}

static void position(double *lat, double *lon, float *alt) {
  if (g_host_pos && millis() - g_host_ms < 10000UL) {
    *lat = g_hlat;
    *lon = g_hlon;
    *alt = g_halt;
    return;
  }
#ifdef USE_GPS
  if (gps.location.isValid() && gps.location.age() < 5000) {
    *lat = gps.location.lat();
    *lon = gps.location.lng();
    *alt = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
    return;
  }
#endif
  sim_position(lat, lon, alt);
}

static void header(cp_packet_t &p, uint8_t type) {
  memset(&p, 0, sizeof(p));
  p.version = CP_PROTO_VERSION;
  p.type = type;
  p.flags = CP_FLAG_ACK_REQ;
  p.net_id = CERISE_NET_ID;
  p.src = NODE_ID;
  p.dst = CP_ADDR_GATEWAY;
  p.seq = g_seq++;
}

static void enqueue(const cp_packet_t &p) {
  if (g_qn == 4) {  // fila cheia: descarta a mais antiga (telemetria velha perde valor)
    jlog("{\"evt\":\"queue_drop\",\"seq\":%u,\"type\":\"%s\"}\n", g_q[g_qh].seq,
                  cp_type_name(g_q[g_qh].type));
    g_qh = (g_qh + 1) % 4;
    g_qn--;
    st.dropped++;
  }
  g_q[(g_qh + g_qn) % 4] = p;
  g_qn++;
  st.msgs++;
}

static void make_pos(cp_packet_t &p) {
  header(p, CP_MSG_POS);
  double lat, lon;
  float alt;
  position(&lat, &lon, &alt);
  uint16_t mv;
  p.p.pos.lat_e7 = (int32_t)lround(lat * 1e7);
  p.p.pos.lon_e7 = (int32_t)lround(lon * 1e7);
  p.p.pos.alt_dm = (int16_t)lroundf(alt * 10.0f);
  p.p.pos.batt_pct = battery(&mv);
  p.p.pos.people = sim_people();
  p.p.pos.area_id = sim_area().area_id;
}

static void make_heartbeat(cp_packet_t &p) {
  header(p, CP_MSG_HEARTBEAT);
  uint16_t mv;
  p.p.hb.batt_pct = battery(&mv);
  p.p.hb.batt_mv = mv;
  p.p.hb.state = g_fstate;
  p.p.hb.uptime_s = millis() / 1000UL;
}

static void make_count(cp_packet_t &p, uint16_t people, uint8_t conf) {
  header(p, CP_MSG_COUNT_REPORT);
  double lat, lon;
  float alt;
  position(&lat, &lon, &alt);
  p.p.count.area_id = sim_area().area_id;
  p.p.count.task_id = sim_area().task_id;
  p.p.count.people = people;
  p.p.count.confidence_pct = conf;
  p.p.count.lat_e7 = (int32_t)lround(lat * 1e7);
  p.p.count.lon_e7 = (int32_t)lround(lon * 1e7);
}

static void make_done(cp_packet_t &p) {
  header(p, CP_MSG_COVERAGE_DONE);
  p.p.done.area_id = sim_area().area_id;
  p.p.done.task_id = sim_area().task_id;
  p.p.done.coverage_pct = (uint8_t)lroundf(sim_coverage_pct());
  p.p.done.duration_s = (uint16_t)sim_elapsed_s(millis());
  p.p.done.people_total = sim_people();
}

// ------------------------------------------------------------------ radio

enum TxResult { TX_OK, TX_ERR, TX_BLOCKED };

static TxResult transmit_packet(cp_packet_t &p) {
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(&p, buf, sizeof(buf));
  if (n < 0) {
    jlog("{\"evt\":\"encode_err\",\"err\":\"%s\"}\n", cp_err_name(n));
    return TX_ERR;
  }
  const LoraParams &rp = radio_params();
  uint32_t toa_us = cp_airtime_us(rp.sf, (uint32_t)(rp.bw_khz * 1000), rp.cr_denom, rp.preamble, (uint16_t)n,
                                  true, true, -1);
  if (g_dwell_ms > 0 && toa_us > g_dwell_ms * 1000UL) {
    st.blocked++;
    jlog("{\"evt\":\"dwell_block\",\"type\":\"%s\",\"toa_ms\":%.1f,\"limit_ms\":%lu,\"sf\":%u}\n",
                  cp_type_name(p.type), toa_us / 1000.0f, (unsigned long)g_dwell_ms, rp.sf);
    return TX_BLOCKED;
  }
  uint8_t cad_busy = 0;
  if (g_cad) {  // CSMA nao persistente: canal ocupado -> espera 0,5 a 1 quadro e escuta de novo
    while (cad_busy < 8 && !radio_channel_free()) {
      cad_busy++;
      delay((uint32_t)random(toa_us / 2000, toa_us / 1000 + 1));
    }
    st.cad_busy += cad_busy;
  }
  board_led(true);
  uint32_t t0 = millis();
  int16_t r = radio_send(buf, (size_t)n);
  g_t_tx_end = millis();
  board_led(false);
  radio_start_rx();
  st.frames++;
  jlog(
      "{\"evt\":\"tx\",\"t_ms\":%lu,\"type\":\"%s\",\"seq\":%u,\"attempt\":%u,\"len\":%d,\"toa_ms\":%.1f,"
      "\"tx_ms\":%lu,\"cad_busy\":%u,\"rc\":%d}\n",
      (unsigned long)t0, cp_type_name(p.type), p.seq, g_attempt + 1, n, toa_us / 1000.0f,
      (unsigned long)(g_t_tx_end - t0), cad_busy, r);
  return r == 0 ? TX_OK : TX_ERR;
}

static void start_attempt() {
  g_cur.flags = (uint8_t)(g_cur.flags & ~CP_FLAG_RETRY);
  if (g_attempt > 0) g_cur.flags |= CP_FLAG_RETRY;
  if (transmit_packet(g_cur) == TX_BLOCKED) {  // excede o limite de ocupacao: descarta
    st.dropped++;
    g_state = ST_IDLE;
    return;
  }
  if (g_cur.flags & CP_FLAG_ACK_REQ) {
    g_deadline = millis() + ack_timeout_ms();
    g_state = ST_WAIT_ACK;
  } else {
    g_state = ST_IDLE;
  }
}

static void send_dl_ack(const cp_packet_t &rx, int rssi, float snr, uint8_t status) {
  cp_packet_t ack;
  cp_make_ack(&rx, NODE_ID, g_ack_seq++, rssi, snr, status, &ack);
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(&ack, buf, sizeof(buf));
  delay(GW_TURNAROUND_MS);
  radio_send(buf, (size_t)n);
  radio_start_rx();
}

static void apply_area(const cp_packet_t &p) {
  MissionArea a;
  a.lat_min = p.p.area.lat_min_e7 / 1e7;
  a.lon_min = p.p.area.lon_min_e7 / 1e7;
  a.lat_max = p.p.area.lat_max_e7 / 1e7;
  a.lon_max = p.p.area.lon_max_e7 / 1e7;
  a.alt_m = p.p.area.alt_dm / 10.0f;
  a.spacing_m = p.p.area.spacing_dm / 10.0f;
  a.area_id = p.p.area.area_id;
  a.task_id = p.p.area.task_id;
  sim_assign(a, millis());
  g_done_sent = false;
  g_people_reported = 0;
  g_fstate = CP_STATE_MISSION;
}

static void handle_rx() {
  uint8_t buf[CP_MAX_FRAME + 8];
  size_t n = 0;
  float rssi, snr;
  int16_t rc = radio_read(buf, sizeof(buf), &n, &rssi, &snr);
  radio_start_rx();
  if (rc != 0) {
    st.rx_err++;
    jlog("{\"evt\":\"rx_err\",\"err\":\"PHY\",\"rc\":%d,\"rssi\":%.0f}\n", rc, rssi);
    return;
  }
  cp_packet_t p;
  int err = cp_decode(buf, n, &p);
  if (err != CP_OK) {
    st.rx_err++;
    jlog("{\"evt\":\"rx_err\",\"err\":\"%s\",\"len\":%u,\"rssi\":%.0f,\"snr\":%.2f}\n", cp_err_name(err),
                  (unsigned)n, rssi, snr);
    return;
  }
  if (p.net_id != CERISE_NET_ID || (p.dst != NODE_ID && p.dst != CP_ADDR_BROADCAST)) return;
  g_rssi = (int)lroundf(rssi);
  g_snr = snr;

  if (p.type == CP_MSG_ACK && g_state == ST_WAIT_ACK && p.p.ack.acked_seq == g_cur.seq) {
    st.acked++;
    g_rtt = millis() - g_t_first;
    g_gw_rssi = -(int)p.p.ack.rssi_neg;
    g_gw_snr = p.p.ack.snr_qdb / 4.0f;
    jlog(
        "{\"evt\":\"ack\",\"t_ms\":%lu,\"seq\":%u,\"type\":\"%s\",\"attempts\":%u,\"lat_ms\":%lu,\"status\":%u,"
        "\"gw_rssi\":%d,\"gw_snr\":%.2f,\"rssi\":%d,\"snr\":%.2f,\"dl\":%s}\n",
        (unsigned long)millis(), g_cur.seq, cp_type_name(g_cur.type), g_attempt + 1, (unsigned long)g_rtt,
        p.p.ack.status, g_gw_rssi, g_gw_snr, g_rssi, g_snr, (p.flags & CP_FLAG_DL_PENDING) ? "true" : "false");
    if (p.flags & CP_FLAG_DL_PENDING) {
      g_deadline = millis() + GW_TURNAROUND_MS + 40 + frame_toa_ms(CP_MSG_AREA_ASSIGN) + ACK_MARGIN_MS;
      g_state = ST_WAIT_DL;
    } else {
      g_state = ST_IDLE;
    }
    return;
  }

  if (p.type == CP_MSG_AREA_ASSIGN) {
    bool dup = g_have_dl && p.seq == g_last_dl_seq;
    char js[320];
    cp_to_json(&p, millis(), g_rssi, snr, dup, js, sizeof(js));
    jlog("{\"evt\":\"dl\",\"pkt\":%s}\n", js);
    if (!dup) {
      apply_area(p);
      st.dl_rx++;
    }
    g_have_dl = true;
    g_last_dl_seq = p.seq;
    if (p.flags & CP_FLAG_ACK_REQ) send_dl_ack(p, g_rssi, snr, dup ? CP_ACK_DUP : CP_ACK_OK);
    if (g_state == ST_WAIT_DL) g_state = ST_IDLE;
  } else if (p.type != CP_MSG_ACK) {
    // demais mensagens para este drone (ex.: AREA_POLY, TASK_AWARD da colaboracao): repassa ao
    // computador de bordo pelo log JSON e confirma se pedido
    char js[320];
    cp_to_json(&p, millis(), g_rssi, snr, false, js, sizeof(js));
    jlog("{\"evt\":\"dl\",\"pkt\":%s}\n", js);
    if ((p.flags & CP_FLAG_ACK_REQ) && p.dst == NODE_ID) send_dl_ack(p, g_rssi, snr, CP_ACK_OK);
  }
}

// ------------------------------------------------------------ comandos

static void print_status() {
  const LoraParams &p = radio_params();
  jlog(
      "{\"evt\":\"status\",\"id\":%u,\"sf\":%u,\"tx_dbm\":%d,\"period_ms\":%lu,\"retries\":%u,\"dwell_ms\":%lu,"
      "\"msgs\":%lu,\"frames\":%lu,\"acked\":%lu,\"dropped\":%lu,\"retx\":%lu,\"dl_rx\":%lu,\"rx_err\":%lu,"
      "\"blocked\":%lu,\"cad_busy\":%lu,\"cad\":%d,\"area\":%u,\"task\":%u,\"coverage\":%.1f,\"people\":%u}\n",
      NODE_ID, p.sf, p.tx_dbm, (unsigned long)g_period, g_max_retries, (unsigned long)g_dwell_ms,
      (unsigned long)st.msgs, (unsigned long)st.frames, (unsigned long)st.acked, (unsigned long)st.dropped,
      (unsigned long)st.retries, (unsigned long)st.dl_rx, (unsigned long)st.rx_err, (unsigned long)st.blocked,
      (unsigned long)st.cad_busy, g_cad ? 1 : 0,
      sim_area().area_id, sim_area().task_id, sim_coverage_pct(), sim_people());
}

static void handle_cmd(char *l) {
  int a = 0, b = 0;
  if (cmd_is(l, "HELP")) {
    jlog(
        "# comandos: STATUS | SF <7-12> | POWER <dBm> | PERIOD <ms> | RETRIES <n> | DWELL <ms|0> | CAD <0|1> | "
        "POS <lat> <lon> <alt_m> | PEOPLE <n> | COUNT <n> <conf%%> | DONE | RANGETEST <n> <intervalo_ms> | TX <hex>\n");
  } else if (cmd_is(l, "STATUS")) {
    print_status();
  } else if (cmd_is(l, "SF") && sscanf(l + 2, "%d", &a) == 1 && a >= 7 && a <= 12) {
    jlog("{\"evt\":\"cfg\",\"sf\":%d,\"rc\":%d}\n", a, radio_set_sf((uint8_t)a));
    radio_start_rx();
  } else if (cmd_is(l, "POWER") && sscanf(l + 5, "%d", &a) == 1) {
    jlog("{\"evt\":\"cfg\",\"tx_dbm\":%d,\"rc\":%d}\n", a, radio_set_power((int8_t)a));
    radio_start_rx();
  } else if (cmd_is(l, "PERIOD") && sscanf(l + 6, "%d", &a) == 1 && a >= 200) {
    g_period = (uint32_t)a;
    jlog("{\"evt\":\"cfg\",\"period_ms\":%d}\n", a);
  } else if (cmd_is(l, "RETRIES") && sscanf(l + 7, "%d", &a) == 1 && a >= 0 && a <= 8) {
    g_max_retries = (uint8_t)a;
    jlog("{\"evt\":\"cfg\",\"retries\":%d}\n", a);
  } else if (cmd_is(l, "DWELL") && sscanf(l + 5, "%d", &a) == 1 && a >= 0) {
    g_dwell_ms = (uint32_t)a;
    jlog("{\"evt\":\"cfg\",\"dwell_ms\":%d}\n", a);
  } else if (cmd_is(l, "CAD") && sscanf(l + 3, "%d", &a) == 1) {
    g_cad = a != 0;
    jlog("{\"evt\":\"cfg\",\"cad\":%d}\n", g_cad ? 1 : 0);
  } else if (cmd_is(l, "POS")) {
    double la, lo;
    float al;
    if (sscanf(l + 3, "%lf %lf %f", &la, &lo, &al) == 3) {
      g_hlat = la;
      g_hlon = lo;
      g_halt = al;
      g_host_pos = true;
      g_host_ms = millis();
    }
  } else if (cmd_is(l, "PEOPLE") && sscanf(l + 6, "%d", &a) == 1) {
    sim_set_people((uint16_t)a);
  } else if (cmd_is(l, "COUNT") && sscanf(l + 5, "%d %d", &a, &b) == 2) {
    cp_packet_t p;
    make_count(p, (uint16_t)a, (uint8_t)b);
    enqueue(p);
  } else if (cmd_is(l, "DONE")) {
    cp_packet_t p;
    make_done(p);
    enqueue(p);
  } else if (cmd_is(l, "RANGETEST") && sscanf(l + 9, "%d %d", &a, &b) == 2 && a > 0 && b >= 100) {
    g_rt_left = (uint32_t)a;
    g_rt_interval = (uint32_t)b;
    g_rt_next = millis();
    g_rt_sent = 0;
    g_rt_id++;
    jlog("{\"evt\":\"rangetest_start\",\"id\":%u,\"n\":%d,\"interval_ms\":%d,\"sf\":%u,\"tx_dbm\":%d}\n",
                  g_rt_id, a, b, radio_params().sf, radio_params().tx_dbm);
  } else if (cmd_is(l, "TX")) {
    // quadro ja codificado pelo computador de bordo (ex.: TASK_BID da colaboracao), sem ACK/retentativa
    uint8_t buf[CP_MAX_FRAME];
    int n = hex_to_bytes(l + 2, buf, sizeof(buf));
    cp_packet_t p;
    int err = n > 0 ? cp_decode(buf, (size_t)n, &p) : CP_ERR_SHORT;
    if (err != CP_OK || p.src != NODE_ID) {
      jlog("{\"evt\":\"cmd_err\",\"msg\":\"quadro invalido ou src diferente\",\"err\":\"%s\"}\n", cp_err_name(err));
      return;
    }
    if (g_cad) radio_channel_free();
    int16_t rc = radio_send(buf, (size_t)n);
    radio_start_rx();
    st.frames++;
    jlog("{\"evt\":\"raw_tx\",\"type\":\"%s\",\"seq\":%u,\"len\":%d,\"rc\":%d}\n", cp_type_name(p.type), p.seq, n, rc);
  } else {
    jlog("{\"evt\":\"cmd_err\",\"line\":\"%s\"}\n", l);
  }
}

// ---------------------------------------------------------------- display

static void update_display() {
  char l0[24], l1[24], l2[24], l3[24], l4[24], l5[24];
  double lat, lon;
  float alt;
  position(&lat, &lon, &alt);
  uint32_t fin = st.acked + st.dropped;
  uint32_t pdr = fin ? (100UL * st.acked) / fin : 0;
  snprintf(l0, sizeof(l0), "NODE %u  SF%u %ddBm", NODE_ID, radio_params().sf, radio_params().tx_dbm);
  snprintf(l1, sizeof(l1), "seq%u tx%lu ack%lu", g_seq, (unsigned long)st.frames, (unsigned long)st.acked);
  snprintf(l2, sizeof(l2), "ret%lu drop%lu PDR%lu%%", (unsigned long)st.retries, (unsigned long)st.dropped,
           (unsigned long)pdr);
  snprintf(l3, sizeof(l3), "GW %ddBm %.1fdB", g_gw_rssi, g_gw_snr);
  snprintf(l4, sizeof(l4), "%.5f %.5f", lat, lon);
  snprintf(l5, sizeof(l5), "A%u T%u P%u %.0f%%", sim_area().area_id, sim_area().task_id, sim_people(),
           sim_coverage_pct());
  display_lines(l0, l1, l2, l3, l4, l5);
  display_flush();
}

// ------------------------------------------------------------ setup/loop

void setup() {
  Serial.begin(115200);
  board_init();
  display_begin();
  randomSeed(esp_random());
  g_seq = (uint16_t)(esp_random() & 0xFFFF);  // seq inicial aleatorio (reinicio nao parece duplicata)
#ifdef USE_GPS
  GpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
#endif
#ifdef HOST_PORT
  HOST_PORT.begin(115200, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
#endif
  LoraParams p = {cp_au915_uplink_hz(LORA_AU915_CH) / 1e6f, (float)LORA_BW_KHZ, LORA_SF, LORA_CR_DENOM,
                  LORA_TX_DBM, LORA_PREAMBLE, LORA_SYNC_WORD};
  int16_t rc = radio_begin(p);
  uint16_t pos_len = (uint16_t)cp_frame_len(CP_MSG_POS);
  jlog(
      "{\"evt\":\"boot\",\"role\":\"node\",\"fw\":\"%s\",\"id\":%u,\"board\":\"%s\",\"radio\":\"%s\",\"rc\":%d,"
      "\"freq_mhz\":%.1f,\"sf\":%u,\"bw_khz\":%.1f,\"cr\":\"4/%u\",\"tx_dbm\":%d,\"proto\":%u,"
      "\"toa_pos_us\":%lu,\"toa_pos_radiolib_us\":%lu}\n",
      FW_VERSION, NODE_ID, BOARD_NAME, RADIO_NAME, rc, p.freq_mhz, p.sf, p.bw_khz, p.cr_denom, p.tx_dbm,
      CP_PROTO_VERSION,
      (unsigned long)cp_airtime_us(p.sf, (uint32_t)(p.bw_khz * 1000), p.cr_denom, p.preamble, pos_len, true, true,
                                   -1),
      (unsigned long)radio_toa_us(pos_len));
  if (rc != 0) {
    display_lines("CERISE NODE", "ERRO no radio", "codigo RadioLib:", String(rc).c_str());
    display_flush();
    while (true) delay(1000);
  }
  radio_start_rx();
  g_next_tel = millis() + random(0, g_period);  // fase aleatoria entre drones
}

void loop() {
  static uint32_t last_ms = millis(), last_disp = 0;
  uint32_t now = millis();
  sim_step((now - last_ms) / 1000.0f);
  last_ms = now;

  if (g_line.poll(Serial)) handle_cmd(g_line.buf());
#ifdef HOST_PORT
  if (g_line1.poll(HOST_PORT)) handle_cmd(g_line1.buf());
#endif
#ifdef USE_GPS
  while (GpsSerial.available()) gps.encode(GpsSerial.read());
#endif
  if (radio_rx_pending()) handle_rx();

  switch (g_state) {
    case ST_IDLE:
      if (g_rt_left > 0) {  // teste de alcance: sem ACK, intervalo fixo
        if ((int32_t)(now - g_rt_next) >= 0) {
          cp_packet_t p;
          make_pos(p);
          p.flags = CP_FLAG_TEST;
          g_attempt = 0;
          transmit_packet(p);
          g_rt_left--;
          g_rt_sent++;
          g_rt_next += g_rt_interval;
          if (g_rt_left == 0)
            jlog("{\"evt\":\"rangetest_end\",\"id\":%u,\"sent\":%lu}\n", g_rt_id, (unsigned long)g_rt_sent);
        }
        break;
      }
      if (g_qn > 0) {
        g_cur = g_q[g_qh];
        g_qh = (g_qh + 1) % 4;
        g_qn--;
        g_attempt = 0;
        g_t_first = now;
        start_attempt();
      } else if ((int32_t)(now - g_next_tel) >= 0) {
        cp_packet_t p;
        g_cycle++;
        if (g_cycle % HEARTBEAT_EVERY == 0)
          make_heartbeat(p);
        else
          make_pos(p);
        enqueue(p);
        if (sim_people() >= g_people_reported + 5) {
          make_count(p, sim_people(), 80);
          enqueue(p);
          g_people_reported = sim_people();
        }
        if (sim_done() && !g_done_sent) {
          make_done(p);
          enqueue(p);
          g_done_sent = true;
          g_done_ms = now;
          g_fstate = CP_STATE_RTL;
        }
        if (g_done_sent && now - g_done_ms > 30000UL) {  // reinicia a missao sintetica
          MissionArea a = sim_area();
          a.task_id++;
          sim_assign(a, now);
          g_done_sent = false;
          g_people_reported = 0;
          g_fstate = CP_STATE_MISSION;
        }
        // periodo com +-10 % de variacao aleatoria para desfazer sincronismo entre drones
        g_next_tel = now + g_period - g_period / 10 + (uint32_t)random(0, g_period / 5 + 1);
      }
      break;

    case ST_WAIT_ACK:
      if ((int32_t)(now - g_deadline) >= 0) {
        if (g_attempt < g_max_retries) {
          g_attempt++;
          st.retries++;
          // janela de recuo = (quadro + espera do ACK) * 2^tentativa; janelas da ordem de 1 quadro
          // fazem as retransmissoes de nos que colidiram colidirem de novo (ver tools/sim_canal.py)
          uint32_t win = (frame_toa_ms(g_cur.type) + ack_timeout_ms()) << g_attempt;
          uint32_t backoff = 20 + (uint32_t)random(0, win + 1);
          jlog("{\"evt\":\"timeout\",\"seq\":%u,\"attempt\":%u,\"backoff_ms\":%lu}\n", g_cur.seq, g_attempt,
                        (unsigned long)backoff);
          g_deadline = now + backoff;
          g_state = ST_BACKOFF;
        } else {
          st.dropped++;
          jlog("{\"evt\":\"drop\",\"seq\":%u,\"type\":\"%s\",\"attempts\":%u}\n", g_cur.seq,
                        cp_type_name(g_cur.type), g_attempt + 1);
          g_state = ST_IDLE;
        }
      }
      break;

    case ST_BACKOFF:
      if ((int32_t)(now - g_deadline) >= 0) start_attempt();
      break;

    case ST_WAIT_DL:
      if ((int32_t)(now - g_deadline) >= 0) g_state = ST_IDLE;
      break;
  }

  if (now - last_disp > 500) {
    last_disp = now;
    update_display();
  }
}
