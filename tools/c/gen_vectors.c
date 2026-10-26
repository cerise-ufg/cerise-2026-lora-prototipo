/*
 * Gera vetores de teste com a implementacao em C do cerise_proto para validar
 * o espelho Python (tools/cerise_proto.py). Compilar no PC:
 *
 *   gcc -O2 -std=gnu11 -I../../firmware/lib/cerise_proto/src \
 *       gen_vectors.c ../../firmware/lib/cerise_proto/src/cerise_proto.c -o gen_vectors
 *   ./gen_vectors 2000 > vectors.tsv
 *
 * Linhas (separadas por TAB):
 *   P type flags net src dst seq campos(csv) t_ms rssi snr dup hex json
 *   A sf bw cr preamble payload explicit crc tempo_us
 *   C hex erro          (quadro corrompido aleatoriamente -> codigo de erro do decode)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cerise_proto.h"

static uint32_t rng_state = 0xC0FFEEu;
static uint32_t rnd(void) { /* xorshift32 deterministico */
  uint32_t x = rng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return rng_state = x;
}
static int32_t rnd_range(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }

static void hex(const uint8_t *b, int n) {
  for (int i = 0; i < n; i++) printf("%02x", b[i]);
}

int main(int argc, char **argv) {
  int n = argc > 1 ? atoi(argv[1]) : 1000;
  for (int i = 0; i < n; i++) {
    cp_packet_t p;
    memset(&p, 0, sizeof(p));
    p.version = CP_PROTO_VERSION;
    p.type = (uint8_t)rnd_range(CP_MSG_MIN, CP_MSG_MAX);
    p.flags = (uint8_t)(rnd() & 0x0F);
    p.net_id = (uint8_t)rnd();
    p.src = (uint8_t)rnd();
    p.dst = (uint8_t)rnd();
    p.seq = (uint16_t)rnd();
    char fields[256];
    switch (p.type) {
      case CP_MSG_HEARTBEAT:
        p.p.hb.batt_pct = (uint8_t)rnd_range(0, 100);
        p.p.hb.batt_mv = (uint16_t)rnd();
        p.p.hb.state = (uint8_t)rnd_range(0, 5);
        p.p.hb.uptime_s = rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%u,%lu", p.p.hb.batt_pct, p.p.hb.batt_mv, p.p.hb.state,
                 (unsigned long)p.p.hb.uptime_s);
        break;
      case CP_MSG_POS:
        p.p.pos.lat_e7 = rnd_range(-900000000, 900000000);
        p.p.pos.lon_e7 = rnd_range(-1800000000, 1800000000);
        p.p.pos.alt_dm = (int16_t)rnd();
        p.p.pos.batt_pct = (uint8_t)rnd_range(0, 100);
        p.p.pos.people = (uint16_t)rnd();
        p.p.pos.area_id = (uint8_t)rnd();
        snprintf(fields, sizeof(fields), "%ld,%ld,%d,%u,%u,%u", (long)p.p.pos.lat_e7, (long)p.p.pos.lon_e7,
                 p.p.pos.alt_dm, p.p.pos.batt_pct, p.p.pos.people, p.p.pos.area_id);
        break;
      case CP_MSG_AREA_ASSIGN:
        p.p.area.area_id = (uint8_t)rnd();
        p.p.area.task_id = (uint8_t)rnd();
        p.p.area.lat_min_e7 = rnd_range(-900000000, 900000000);
        p.p.area.lon_min_e7 = rnd_range(-1800000000, 1800000000);
        p.p.area.lat_max_e7 = rnd_range(-900000000, 900000000);
        p.p.area.lon_max_e7 = rnd_range(-1800000000, 1800000000);
        p.p.area.alt_dm = (int16_t)rnd();
        p.p.area.spacing_dm = (uint16_t)rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%ld,%ld,%ld,%ld,%d,%u", p.p.area.area_id, p.p.area.task_id,
                 (long)p.p.area.lat_min_e7, (long)p.p.area.lon_min_e7, (long)p.p.area.lat_max_e7,
                 (long)p.p.area.lon_max_e7, p.p.area.alt_dm, p.p.area.spacing_dm);
        break;
      case CP_MSG_COVERAGE_DONE:
        p.p.done.area_id = (uint8_t)rnd();
        p.p.done.task_id = (uint8_t)rnd();
        p.p.done.coverage_pct = (uint8_t)rnd_range(0, 100);
        p.p.done.duration_s = (uint16_t)rnd();
        p.p.done.people_total = (uint16_t)rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%u,%u,%u", p.p.done.area_id, p.p.done.task_id,
                 p.p.done.coverage_pct, p.p.done.duration_s, p.p.done.people_total);
        break;
      case CP_MSG_COUNT_REPORT:
        p.p.count.area_id = (uint8_t)rnd();
        p.p.count.task_id = (uint8_t)rnd();
        p.p.count.people = (uint16_t)rnd();
        p.p.count.confidence_pct = (uint8_t)rnd_range(0, 100);
        p.p.count.lat_e7 = rnd_range(-900000000, 900000000);
        p.p.count.lon_e7 = rnd_range(-1800000000, 1800000000);
        snprintf(fields, sizeof(fields), "%u,%u,%u,%u,%ld,%ld", p.p.count.area_id, p.p.count.task_id,
                 p.p.count.people, p.p.count.confidence_pct, (long)p.p.count.lat_e7, (long)p.p.count.lon_e7);
        break;
      case CP_MSG_AREA_POLY: {
        p.p.poly.area_id = (uint8_t)rnd();
        p.p.poly.task_id = (uint8_t)rnd();
        p.p.poly.frag = (uint8_t)rnd();
        p.p.poly.n = (uint8_t)rnd_range(0, CP_POLY_MAX_VERTS);
        int k = snprintf(fields, sizeof(fields), "%u,%u,%u,%u", p.p.poly.area_id, p.p.poly.task_id, p.p.poly.frag,
                         p.p.poly.n);
        for (unsigned v = 0; v < p.p.poly.n; v++) {
          p.p.poly.dx[v] = (int16_t)rnd();
          p.p.poly.dy[v] = (int16_t)rnd();
          k += snprintf(fields + k, sizeof(fields) - (size_t)k, ",%d,%d", p.p.poly.dx[v], p.p.poly.dy[v]);
        }
        break;
      }
      case CP_MSG_TASK_BID:
        p.p.bid.task_id = (uint8_t)rnd();
        p.p.bid.area_id = (uint8_t)rnd();
        p.p.bid.cost_ds = (uint16_t)rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%u", p.p.bid.task_id, p.p.bid.area_id, p.p.bid.cost_ds);
        break;
      case CP_MSG_TASK_AWARD:
        p.p.award.task_id = (uint8_t)rnd();
        p.p.award.area_id = (uint8_t)rnd();
        p.p.award.winner = (uint8_t)rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%u", p.p.award.task_id, p.p.award.area_id, p.p.award.winner);
        break;
      default: /* ACK */
        p.p.ack.acked_seq = (uint16_t)rnd();
        p.p.ack.acked_type = (uint8_t)rnd_range(1, 6);
        p.p.ack.status = (uint8_t)rnd_range(0, 2);
        p.p.ack.rssi_neg = (uint8_t)rnd();
        p.p.ack.snr_qdb = (int8_t)rnd();
        snprintf(fields, sizeof(fields), "%u,%u,%u,%u,%d", p.p.ack.acked_seq, p.p.ack.acked_type,
                 p.p.ack.status, p.p.ack.rssi_neg, p.p.ack.snr_qdb);
        break;
    }
    uint8_t buf[CP_MAX_FRAME];
    int len = cp_encode(&p, buf, sizeof(buf));
    uint32_t t_ms = rnd();
    int rssi = rnd_range(-140, -20);
    float snr = (float)rnd_range(-2500, 1500) / 97.0f;
    int dup = (int)(rnd() & 1u);
    char js[400];
    cp_to_json(&p, t_ms, rssi, snr, dup != 0, js, sizeof(js));
    printf("P\t%u\t%u\t%u\t%u\t%u\t%u\t%s\t%lu\t%d\t%.9g\t%d\t", p.type, p.flags, p.net_id, p.src, p.dst, p.seq,
           fields, (unsigned long)t_ms, rssi, (double)snr, dup);
    hex(buf, len);
    printf("\t%s\n", js);

    /* corrupcao aleatoria de 1 a 4 bits DISTINTOS */
    int nflip = rnd_range(1, 4), bits[4];
    for (int k = 0; k < nflip; k++) {
      int bit, again;
      do {
        bit = rnd_range(0, len * 8 - 1);
        again = 0;
        for (int q = 0; q < k; q++) again |= (bits[q] == bit);
      } while (again);
      bits[k] = bit;
      buf[bit / 8] ^= (uint8_t)(1u << (bit % 8));
    }
    cp_packet_t out;
    int err = cp_decode(buf, (size_t)len, &out);
    printf("C\t");
    hex(buf, len);
    printf("\t%s\n", cp_err_name(err));
  }

  /* grade de tempo no ar */
  const uint32_t bws[] = {125000, 250000, 500000};
  for (int sf = 7; sf <= 12; sf++)
    for (int b = 0; b < 3; b++)
      for (int cr = 5; cr <= 8; cr++)
        for (int pl = 0; pl <= 64; pl += 1)
          for (int eh = 0; eh <= 1; eh++) {
            uint32_t us = cp_airtime_us((uint8_t)sf, bws[b], (uint8_t)cr, 8, (uint16_t)pl, eh != 0, true, -1);
            printf("A\t%d\t%lu\t%d\t8\t%d\t%d\t1\t%lu\n", sf, (unsigned long)bws[b], cr, pl, eh, (unsigned long)us);
          }
  return 0;
}
