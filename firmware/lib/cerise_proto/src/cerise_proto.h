/*
 * cerise_proto — protocolo binario versionado de telemetria/comando LoRa
 * entre os drones do CERISE (no "node") e a estacao de solo ("gateway").
 *
 * Biblioteca em C99 puro (sem dependencia de Arduino) para que o mesmo
 * codigo rode no ESP32 e nos testes unitarios nativos (pio test -e native).
 *
 * Formato do quadro (todos os campos multibyte em little-endian):
 *
 *   off  tam  campo
 *   0    1    ver_type  = (versao << 4) | tipo        (versao atual = 1)
 *   1    1    flags     (CP_FLAG_*)
 *   2    1    net_id    (identificador da rede/missao)
 *   3    1    src       (0x00 = gateway, 0x01..0xFE = drones)
 *   4    1    dst       (0x00 = gateway, 0xFF = broadcast)
 *   5    2    seq       (numero de sequencia, uint16, por origem)
 *   7    1    len       (tamanho do payload em bytes)
 *   8    len  payload   (layout fixo por tipo, ver cp_payload_len())
 *   8+len 2   crc16     CRC-16/CCITT-FALSE sobre bytes [0, 8+len)
 *
 * Nomes de mensagens alinhados com a frente de drones (planejamento de
 * cobertura + contagem de pessoas): HEARTBEAT, POS, AREA_ASSIGN,
 * COVERAGE_DONE, COUNT_REPORT e ACK. Os tipos 7-9 (AREA_POLY, TASK_BID,
 * TASK_AWARD) incorporam a extensao proposta pela frente de drones para a
 * colaboracao (cerise_drones/protocolo.py); sao aditivos e nao mudam a versao.
 */
#ifndef CERISE_PROTO_H
#define CERISE_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CP_PROTO_VERSION 1u
#define CP_HEADER_LEN 8u
#define CP_CRC_LEN 2u
#define CP_MAX_PAYLOAD 32u
#define CP_MAX_FRAME (CP_HEADER_LEN + CP_MAX_PAYLOAD + CP_CRC_LEN)

#define CP_ADDR_GATEWAY 0x00u
#define CP_ADDR_BROADCAST 0xFFu

/* flags do cabecalho */
#define CP_FLAG_ACK_REQ 0x01u    /* remetente quer ACK */
#define CP_FLAG_RETRY 0x02u      /* retransmissao (mesmo seq) */
#define CP_FLAG_DL_PENDING 0x04u /* (no ACK) gateway tem downlink para o no */
#define CP_FLAG_TEST 0x08u       /* pacote de teste de alcance (sem retentativa) */

typedef enum {
  CP_MSG_HEARTBEAT = 1,
  CP_MSG_POS = 2,
  CP_MSG_AREA_ASSIGN = 3,
  CP_MSG_COVERAGE_DONE = 4,
  CP_MSG_COUNT_REPORT = 5,
  CP_MSG_ACK = 6,
  CP_MSG_AREA_POLY = 7, /* extensao: vertices do poligono da area (fragmentado) */
  CP_MSG_TASK_BID = 8,  /* extensao: lance no leilao de tarefas */
  CP_MSG_TASK_AWARD = 9 /* extensao: resultado do leilao */
} cp_msg_type_t;

#define CP_MSG_MIN 1
#define CP_MSG_MAX 9
#define CP_POLY_MAX_VERTS 7 /* (32 - 4) / 4 */

/* estados de voo reportados no HEARTBEAT */
typedef enum {
  CP_STATE_IDLE = 0,
  CP_STATE_ARMED = 1,
  CP_STATE_MISSION = 2,
  CP_STATE_RTL = 3,
  CP_STATE_LANDED = 4,
  CP_STATE_FAILSAFE = 5
} cp_state_t;

/* status do ACK */
typedef enum { CP_ACK_OK = 0, CP_ACK_DUP = 1, CP_ACK_REJECTED = 2 } cp_ack_status_t;

typedef enum {
  CP_OK = 0,
  CP_ERR_NULL = -1,
  CP_ERR_SHORT = -2,
  CP_ERR_VERSION = -3,
  CP_ERR_TYPE = -4,
  CP_ERR_LEN = -5,
  CP_ERR_CRC = -6,
  CP_ERR_BUF = -7
} cp_err_t;

/* ---- payloads (tamanho serializado entre parenteses) ---- */

typedef struct { /* HEARTBEAT (8 B) */
  uint8_t batt_pct;
  uint16_t batt_mv;
  uint8_t state; /* cp_state_t */
  uint32_t uptime_s;
} cp_heartbeat_t;

/* POS (14 B) — telemetria periodica do drone. Dimensionado para que o quadro
 * completo (24 B) caiba no limite de 400 ms de ocupacao do canal ate SF10/125 kHz. */
typedef struct {
  int32_t lat_e7; /* graus * 1e7 */
  int32_t lon_e7; /* graus * 1e7 */
  int16_t alt_dm; /* altitude relativa ao ponto de decolagem, decimetros */
  uint8_t batt_pct;
  uint16_t people; /* contagem de pessoas acumulada na tarefa */
  uint8_t area_id; /* area/tarefa em execucao (0 = nenhuma) */
} cp_pos_t;

typedef struct { /* AREA_ASSIGN (22 B) — gateway -> drone */
  uint8_t area_id;
  uint8_t task_id;
  int32_t lat_min_e7, lon_min_e7, lat_max_e7, lon_max_e7; /* retangulo */
  int16_t alt_dm;      /* altitude de voo */
  uint16_t spacing_dm; /* espacamento entre faixas (boustrophedon) */
} cp_area_assign_t;

typedef struct { /* COVERAGE_DONE (7 B) */
  uint8_t area_id;
  uint8_t task_id;
  uint8_t coverage_pct;
  uint16_t duration_s;
  uint16_t people_total;
} cp_coverage_done_t;

typedef struct { /* COUNT_REPORT (13 B) */
  uint8_t area_id;
  uint8_t task_id;
  uint16_t people;
  uint8_t confidence_pct;
  int32_t lat_e7;
  int32_t lon_e7;
} cp_count_report_t;

typedef struct { /* ACK (6 B) */
  uint16_t acked_seq;
  uint8_t acked_type;
  uint8_t status;   /* cp_ack_status_t */
  uint8_t rssi_neg; /* RSSI medido no receptor = -rssi_neg dBm */
  int8_t snr_qdb;   /* SNR medido no receptor, quartos de dB */
} cp_ack_t;

typedef struct { /* AREA_POLY (4 + 4n B, n <= 7) — vertices em metros relativos a (lon_min, lat_min) */
  uint8_t area_id;
  uint8_t task_id;
  uint8_t frag; /* (indice << 4) | total de fragmentos */
  uint8_t n;    /* vertices neste quadro; (-32768,-32768) separa partes */
  int16_t dx[CP_POLY_MAX_VERTS];
  int16_t dy[CP_POLY_MAX_VERTS];
} cp_area_poly_t;

typedef struct { /* TASK_BID (4 B) */
  uint8_t task_id;
  uint8_t area_id;
  uint16_t cost_ds; /* custo = tempo estimado de conclusao, decimos de segundo */
} cp_task_bid_t;

typedef struct { /* TASK_AWARD (3 B) */
  uint8_t task_id;
  uint8_t area_id;
  uint8_t winner; /* endereco do drone vencedor */
} cp_task_award_t;

typedef struct {
  uint8_t version; /* preenchido pelo decode; o encode usa CP_PROTO_VERSION */
  uint8_t type;    /* cp_msg_type_t */
  uint8_t flags;
  uint8_t net_id;
  uint8_t src;
  uint8_t dst;
  uint16_t seq;
  union {
    cp_heartbeat_t hb;
    cp_pos_t pos;
    cp_area_assign_t area;
    cp_coverage_done_t done;
    cp_count_report_t count;
    cp_ack_t ack;
    cp_area_poly_t poly;
    cp_task_bid_t bid;
    cp_task_award_t award;
  } p;
} cp_packet_t;

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, sem reflexao, xorout 0).
 * Valor de verificacao: cp_crc16("123456789") == 0x29B1. */
uint16_t cp_crc16(const uint8_t *data, size_t len);

/* tamanho do payload do tipo (minimo, para AREA_POLY), ou -1 se o tipo e desconhecido */
int cp_payload_len(uint8_t type);

/* tamanho do payload deste pacote (considera n em AREA_POLY); -1 se invalido */
int cp_packet_payload_len(const cp_packet_t *pkt);

/* tamanho total do quadro (cabecalho + payload + CRC), ou -1 */
int cp_frame_len(uint8_t type);

/* Serializa pkt em out. Retorna o numero de bytes (>0) ou cp_err_t (<0). */
int cp_encode(const cp_packet_t *pkt, uint8_t *out, size_t cap);

/* Valida (tamanho, versao, tipo, len, CRC) e desserializa. Retorna CP_OK ou cp_err_t. */
int cp_decode(const uint8_t *in, size_t len, cp_packet_t *pkt);

const char *cp_type_name(uint8_t type);
const char *cp_err_name(int err);
const char *cp_state_name(uint8_t state);

/* Monta o ACK de resposta a rx (src/dst invertidos, mesmo net_id). */
void cp_make_ack(const cp_packet_t *rx, uint8_t my_addr, uint16_t my_seq, int rssi_dbm,
                 float snr_db, uint8_t status, cp_packet_t *ack);

/* ---- deteccao de duplicatas (retransmissoes) no receptor ---- */
typedef struct {
  uint16_t last_seq[256];
  uint8_t seen[32]; /* bitmap: origem ja vista */
} cp_dedup_t;

void cp_dedup_init(cp_dedup_t *d);
/* true se (src, seq) e novo; false se e repeticao do ultimo seq da origem */
bool cp_dedup_is_new(cp_dedup_t *d, uint8_t src, uint16_t seq);

/* ---- utilitarios ---- */

/* Tempo no ar LoRa (Semtech AN1200.13 / datasheet SX127x/SX126x), em microssegundos.
 * cr_denom: 5..8 (4/5..4/8). ldro: 1 liga, 0 desliga, -1 automatico (Tsym >= 16 ms). */
uint32_t cp_airtime_us(uint8_t sf, uint32_t bw_hz, uint8_t cr_denom, uint16_t preamble_len,
                       uint16_t payload_len, bool explicit_header, bool crc_on, int ldro);

/* Plano AU915-928 (adotado no Brasil): canal de uplink 0..63 (125 kHz) e 64..71 (500 kHz). */
uint32_t cp_au915_uplink_hz(uint8_t ch);
/* Canal de downlink 0..7 (500 kHz): 923,3 + 0,6*ch MHz */
uint32_t cp_au915_downlink_hz(uint8_t ch);

/* Estimativa de carga de bateria LiPo 1S a partir da tensao (curva linear por partes). */
uint8_t cp_lipo_pct(uint16_t mv);

/* Formata o pacote como uma linha JSON (sem '\n'), igual a saida serial do gateway.
 * Retorna o numero de caracteres escritos ou -1 se nao coube. */
int cp_to_json(const cp_packet_t *pkt, uint32_t t_ms, int rssi_dbm, float snr_db, bool dup,
               char *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* CERISE_PROTO_H */
