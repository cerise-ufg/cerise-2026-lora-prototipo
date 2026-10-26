// =====================================================================
// CERISE 2026 — firmware LoRa, papel GATEWAY (estacao de solo)
//
// - recepcao continua; valida CRC de camada fisica + quadro cerise_proto (CRC16)
// - responde ACK (com RSSI/SNR medidos) quando o no pede (CP_FLAG_ACK_REQ)
// - detecta duplicatas (retransmissoes) e estima PDR por no a partir das lacunas de seq
// - fila de downlink (AREA_ASSIGN) entregue logo apos o ACK (flag DL_PENDING)
// - imprime uma linha JSON por evento na serial (lida por tools/gateway_dashboard.py)
// - status no OLED
// =====================================================================
#include <Arduino.h>

#include "cerise_proto.h"
#include "common/board.h"
#include "common/config.h"
#include "common/display.h"
#include "common/radio_link.h"
#include "common/serial_cmd.h"

struct NodeStats {
  bool seen;
  uint16_t first_seq, last_seq;
  uint32_t rx, uniq, dups;
  int last_rssi;
  float last_snr;
  uint32_t last_ms;
};

struct Downlink {
  bool used;
  uint8_t node;
  uint8_t tries;
  cp_packet_t pkt;
};

static NodeStats g_nodes[256];
static cp_dedup_t g_dedup;
static Downlink g_dl[8];
static uint16_t g_gw_seq = 0;
static struct {
  uint32_t rx_ok, rx_err, rx_phy_err, dup, foreign, acks_tx, dl_tx, dl_acked, dl_dropped;
} gs = {};
static int g_last_rssi = 0;
static float g_last_snr = 0;
static uint8_t g_last_src = 0;
static uint16_t g_last_seq = 0;
static const char *g_last_type = "-";
static SerialLine g_line;

// ------------------------------------------------------------ estatisticas

static void node_update(const cp_packet_t &p, bool dup, int rssi, float snr) {
  NodeStats &n = g_nodes[p.src];
  if (n.seen && !dup) {
    uint16_t delta = (uint16_t)(p.seq - n.last_seq);
    if (delta == 0 || delta > 1000) {  // seq recuou ou saltou: no reiniciado
      Serial.printf("{\"evt\":\"node_reset\",\"src\":%u,\"old_seq\":%u,\"new_seq\":%u}\n", p.src, n.last_seq, p.seq);
      n.seen = false;
    }
  }
  if (!n.seen) {
    memset(&n, 0, sizeof(n));
    n.seen = true;
    n.first_seq = p.seq;
    n.last_seq = p.seq;
  }
  n.rx++;
  if (dup) {
    n.dups++;
  } else {
    n.uniq++;
    n.last_seq = p.seq;
  }
  n.last_rssi = rssi;
  n.last_snr = snr;
  n.last_ms = millis();
}

static float node_pdr(const NodeStats &n, uint32_t *lost) {
  uint32_t expected = (uint32_t)(uint16_t)(n.last_seq - n.first_seq) + 1;
  uint32_t l = expected > n.uniq ? expected - n.uniq : 0;
  if (lost) *lost = l;
  return expected ? (float)n.uniq / (float)expected : 0.0f;
}

static void print_stats() {
  Serial.printf(
      "{\"evt\":\"stats\",\"t_ms\":%lu,\"rx_ok\":%lu,\"rx_err\":%lu,\"rx_phy_err\":%lu,\"dup\":%lu,\"foreign\":%lu,"
      "\"acks_tx\":%lu,\"dl_tx\":%lu,\"dl_acked\":%lu,\"nodes\":[",
      (unsigned long)millis(), (unsigned long)gs.rx_ok, (unsigned long)gs.rx_err, (unsigned long)gs.rx_phy_err,
      (unsigned long)gs.dup, (unsigned long)gs.foreign, (unsigned long)gs.acks_tx, (unsigned long)gs.dl_tx,
      (unsigned long)gs.dl_acked);
  bool first = true;
  for (int i = 1; i < 255; i++) {
    const NodeStats &n = g_nodes[i];
    if (!n.seen) continue;
    uint32_t lost;
    float pdr = node_pdr(n, &lost);
    Serial.printf("%s{\"id\":%d,\"rx\":%lu,\"uniq\":%lu,\"dup\":%lu,\"lost\":%lu,\"pdr\":%.4f,\"rssi\":%d,\"snr\":%.2f,"
                  "\"age_s\":%lu}",
                  first ? "" : ",", i, (unsigned long)n.rx, (unsigned long)n.uniq, (unsigned long)n.dups,
                  (unsigned long)lost, pdr, n.last_rssi, n.last_snr, (unsigned long)((millis() - n.last_ms) / 1000));
    first = false;
  }
  Serial.println("]}");
}

// --------------------------------------------------------------- downlink

static Downlink *dl_find(uint8_t node) {
  for (auto &d : g_dl)
    if (d.used && d.node == node) return &d;
  return nullptr;
}

static bool send_packet(const cp_packet_t &p) {
  uint8_t buf[CP_MAX_FRAME];
  int n = cp_encode(&p, buf, sizeof(buf));
  if (n < 0) return false;
  board_led(true);
  int16_t rc = radio_send(buf, (size_t)n);
  board_led(false);
  return rc == 0;
}

// --------------------------------------------------------------- recepcao

static void handle_rx() {
  uint8_t buf[CP_MAX_FRAME + 8];
  size_t n = 0;
  float rssi_f, snr;
  int16_t rc = radio_read(buf, sizeof(buf), &n, &rssi_f, &snr);
  int rssi = (int)lroundf(rssi_f);
  uint32_t now = millis();
  if (rc != 0) {  // ex.: RADIOLIB_ERR_CRC_MISMATCH (CRC da camada fisica)
    gs.rx_phy_err++;
    Serial.printf("{\"evt\":\"rx_err\",\"t_ms\":%lu,\"err\":\"PHY\",\"rc\":%d,\"rssi\":%d,\"snr\":%.2f}\n",
                  (unsigned long)now, rc, rssi, snr);
    radio_start_rx();
    return;
  }
  cp_packet_t p;
  int err = cp_decode(buf, n, &p);
  if (err != CP_OK) {
    gs.rx_err++;
    Serial.printf("{\"evt\":\"rx_err\",\"t_ms\":%lu,\"err\":\"%s\",\"len\":%u,\"rssi\":%d,\"snr\":%.2f}\n",
                  (unsigned long)now, cp_err_name(err), (unsigned)n, rssi, snr);
    radio_start_rx();
    return;
  }
  if (p.net_id != CERISE_NET_ID) {
    gs.foreign++;
    Serial.printf("{\"evt\":\"rx_foreign\",\"t_ms\":%lu,\"net\":%u,\"src\":%u}\n", (unsigned long)now, p.net_id,
                  p.src);
    radio_start_rx();
    return;
  }
  if (p.dst != CP_ADDR_GATEWAY && p.dst != CP_ADDR_BROADCAST) {
    radio_start_rx();
    return;
  }

  char js[320];
  if (p.type == CP_MSG_ACK) {  // confirmacao de um downlink
    Downlink *d = dl_find(p.src);
    if (d && p.p.ack.acked_type == CP_MSG_AREA_ASSIGN && p.p.ack.acked_seq == d->pkt.seq) {
      d->used = false;
      gs.dl_acked++;
    }
    cp_to_json(&p, now, rssi, snr, false, js, sizeof(js));
    Serial.println(js);
    radio_start_rx();
    return;
  }

  gs.rx_ok++;
  bool dup = !cp_dedup_is_new(&g_dedup, p.src, p.seq);
  if (dup) gs.dup++;
  node_update(p, dup, rssi, snr);
  g_last_rssi = rssi;
  g_last_snr = snr;
  g_last_src = p.src;
  g_last_seq = p.seq;
  g_last_type = cp_type_name(p.type);
  cp_to_json(&p, now, rssi, snr, dup, js, sizeof(js));
  Serial.println(js);

  if (p.flags & CP_FLAG_ACK_REQ) {
    cp_packet_t ack;
    cp_make_ack(&p, CP_ADDR_GATEWAY, g_gw_seq++, rssi, snr, dup ? CP_ACK_DUP : CP_ACK_OK, &ack);
    Downlink *d = dl_find(p.src);
    if (d) ack.flags |= CP_FLAG_DL_PENDING;
    delay(GW_TURNAROUND_MS);
    if (send_packet(ack)) gs.acks_tx++;
    if (d) {
      delay(GW_TURNAROUND_MS + 20);
      d->tries++;
      if (send_packet(d->pkt)) gs.dl_tx++;
      Serial.printf("{\"evt\":\"dl_tx\",\"t_ms\":%lu,\"dst\":%u,\"seq\":%u,\"try\":%u}\n", (unsigned long)millis(),
                    d->node, d->pkt.seq, d->tries);
      if (d->tries >= 5) {
        d->used = false;
        gs.dl_dropped++;
        Serial.printf("{\"evt\":\"dl_drop\",\"dst\":%u,\"seq\":%u}\n", d->node, d->pkt.seq);
      }
    }
  }
  radio_start_rx();
}

// --------------------------------------------------------------- comandos

static void handle_cmd(char *l) {
  int a = 0;
  if (cmd_is(l, "HELP")) {
    Serial.println(
        "# comandos: STATS | SF <7-12> | POWER <dBm> | "
        "ASSIGN <no> <area> <tarefa> <lat_min> <lon_min> <lat_max> <lon_max> <alt_m> <espac_m> | TX <hex>");
  } else if (cmd_is(l, "STATS")) {
    print_stats();
  } else if (cmd_is(l, "SF") && sscanf(l + 2, "%d", &a) == 1 && a >= 7 && a <= 12) {
    Serial.printf("{\"evt\":\"cfg\",\"sf\":%d,\"rc\":%d}\n", a, radio_set_sf((uint8_t)a));
    radio_start_rx();
  } else if (cmd_is(l, "POWER") && sscanf(l + 5, "%d", &a) == 1) {
    Serial.printf("{\"evt\":\"cfg\",\"tx_dbm\":%d,\"rc\":%d}\n", a, radio_set_power((int8_t)a));
    radio_start_rx();
  } else if (cmd_is(l, "ASSIGN")) {
    int node, area, task;
    double la0, lo0, la1, lo1;
    float alt, sp;
    if (sscanf(l + 6, "%d %d %d %lf %lf %lf %lf %f %f", &node, &area, &task, &la0, &lo0, &la1, &lo1, &alt, &sp) ==
        9) {
      Downlink *d = dl_find((uint8_t)node);
      if (!d)
        for (auto &x : g_dl)
          if (!x.used) {
            d = &x;
            break;
          }
      if (!d) {
        Serial.println("{\"evt\":\"cmd_err\",\"msg\":\"fila de downlink cheia\"}");
        return;
      }
      memset(d, 0, sizeof(*d));
      d->used = true;
      d->node = (uint8_t)node;
      cp_packet_t &p = d->pkt;
      p.version = CP_PROTO_VERSION;
      p.type = CP_MSG_AREA_ASSIGN;
      p.flags = CP_FLAG_ACK_REQ;
      p.net_id = CERISE_NET_ID;
      p.src = CP_ADDR_GATEWAY;
      p.dst = (uint8_t)node;
      p.seq = g_gw_seq++;
      p.p.area.area_id = (uint8_t)area;
      p.p.area.task_id = (uint8_t)task;
      p.p.area.lat_min_e7 = (int32_t)lround(la0 * 1e7);
      p.p.area.lon_min_e7 = (int32_t)lround(lo0 * 1e7);
      p.p.area.lat_max_e7 = (int32_t)lround(la1 * 1e7);
      p.p.area.lon_max_e7 = (int32_t)lround(lo1 * 1e7);
      p.p.area.alt_dm = (int16_t)lroundf(alt * 10.0f);
      p.p.area.spacing_dm = (uint16_t)lroundf(sp * 10.0f);
      Serial.printf("{\"evt\":\"dl_queued\",\"dst\":%d,\"seq\":%u,\"area\":%d,\"task\":%d}\n", node, p.seq, area, task);
    } else {
      Serial.println("{\"evt\":\"cmd_err\",\"msg\":\"uso: ASSIGN no area tarefa lat_min lon_min lat_max lon_max alt espac\"}");
    }
  } else if (cmd_is(l, "TX")) {
    // quadro ja codificado pelo software da estacao de solo (ex.: cerise_drones/protocolo.py:
    // AREA_POLY, TASK_AWARD); validado antes de ir ao ar
    uint8_t buf[CP_MAX_FRAME];
    int n = hex_to_bytes(l + 2, buf, sizeof(buf));
    cp_packet_t p;
    int err = n > 0 ? cp_decode(buf, (size_t)n, &p) : CP_ERR_SHORT;
    if (err != CP_OK) {
      Serial.printf("{\"evt\":\"cmd_err\",\"msg\":\"quadro invalido\",\"err\":\"%s\"}\n", cp_err_name(err));
      return;
    }
    int16_t rc = radio_send(buf, (size_t)n);
    radio_start_rx();
    Serial.printf("{\"evt\":\"raw_tx\",\"type\":\"%s\",\"dst\":%u,\"seq\":%u,\"len\":%d,\"rc\":%d}\n",
                  cp_type_name(p.type), p.dst, p.seq, n, rc);
  } else {
    Serial.printf("{\"evt\":\"cmd_err\",\"line\":\"%s\"}\n", l);
  }
}

// ---------------------------------------------------------------- display

static void update_display() {
  char l0[24], l1[24], l2[24], l3[24], l4[24], l5[24];
  const LoraParams &rp = radio_params();
  snprintf(l0, sizeof(l0), "GATEWAY SF%u %.1f", rp.sf, rp.freq_mhz);
  snprintf(l1, sizeof(l1), "rx%lu err%lu dup%lu", (unsigned long)gs.rx_ok, (unsigned long)(gs.rx_err + gs.rx_phy_err),
           (unsigned long)gs.dup);
  snprintf(l2, sizeof(l2), "#%u %s s%u", g_last_src, g_last_type, g_last_seq);
  snprintf(l3, sizeof(l3), "RSSI%d SNR%.1f", g_last_rssi, g_last_snr);
  l4[0] = '\0';
  int k = 0;
  for (int i = 1; i < 255 && k < 2; i++) {
    if (!g_nodes[i].seen) continue;
    size_t used = strlen(l4);
    snprintf(l4 + used, sizeof(l4) - used, "n%d:%.0f%% ", i, 100.0f * node_pdr(g_nodes[i], nullptr));
    k++;
  }
  snprintf(l5, sizeof(l5), "ack%lu dl%lu/%lu", (unsigned long)gs.acks_tx, (unsigned long)gs.dl_acked,
           (unsigned long)gs.dl_tx);
  display_lines(l0, l1, l2, l3, l4, l5);
  if (gs.rx_ok) display_rssi_bar(g_last_rssi);
  display_flush();
}

// ------------------------------------------------------------ setup/loop

void setup() {
  Serial.begin(115200);
  board_init();
  display_begin();
  cp_dedup_init(&g_dedup);
  LoraParams p = {cp_au915_uplink_hz(LORA_AU915_CH) / 1e6f, (float)LORA_BW_KHZ, LORA_SF, LORA_CR_DENOM,
                  LORA_TX_DBM, LORA_PREAMBLE, LORA_SYNC_WORD};
  int16_t rc = radio_begin(p);
  Serial.printf(
      "{\"evt\":\"boot\",\"role\":\"gateway\",\"fw\":\"%s\",\"board\":\"%s\",\"radio\":\"%s\",\"rc\":%d,"
      "\"freq_mhz\":%.1f,\"sf\":%u,\"bw_khz\":%.1f,\"cr\":\"4/%u\",\"tx_dbm\":%d,\"proto\":%u,\"net\":%u}\n",
      FW_VERSION, BOARD_NAME, RADIO_NAME, rc, p.freq_mhz, p.sf, p.bw_khz, p.cr_denom, p.tx_dbm, CP_PROTO_VERSION,
      CERISE_NET_ID);
  if (rc != 0) {
    display_lines("CERISE GATEWAY", "ERRO no radio", "codigo RadioLib:", String(rc).c_str());
    display_flush();
    while (true) delay(1000);
  }
  radio_start_rx();
  update_display();
}

void loop() {
  static uint32_t last_stats = 0, last_disp = 0;
  if (g_line.poll(Serial)) handle_cmd(g_line.buf());
  if (radio_rx_pending()) handle_rx();
  uint32_t now = millis();
  if (now - last_stats >= GW_STATS_PERIOD_MS) {
    last_stats = now;
    print_stats();
  }
  if (now - last_disp >= 500) {
    last_disp = now;
    update_display();
  }
}
