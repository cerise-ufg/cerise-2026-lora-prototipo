// Parametros padrao do enlace (sobrescreviveis por -D no platformio.ini)
#pragma once

#ifndef LORA_AU915_CH
#define LORA_AU915_CH 8  // 916,8 MHz
#endif
#ifndef LORA_SF
#define LORA_SF 9
#endif
#ifndef LORA_BW_KHZ
#define LORA_BW_KHZ 125.0
#endif
#ifndef LORA_CR_DENOM
#define LORA_CR_DENOM 5
#endif
#ifndef LORA_TX_DBM
#define LORA_TX_DBM 14
#endif
#ifndef LORA_PREAMBLE
#define LORA_PREAMBLE 8
#endif
#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x12
#endif
#ifndef CERISE_NET_ID
#define CERISE_NET_ID 42
#endif

// ---- no (drone) ----
#ifndef NODE_ID
#define NODE_ID 1
#endif
#ifndef TELEMETRY_PERIOD_MS
#define TELEMETRY_PERIOD_MS 5000UL
#endif
#ifndef MAX_RETRIES
#define MAX_RETRIES 3
#endif
#ifndef ACK_MARGIN_MS
#define ACK_MARGIN_MS 150UL  // folga alem do tempo no ar do ACK
#endif
#ifndef USE_CAD
#define USE_CAD 1  // CSMA com CAD antes de cada transmissao (desligavel: "CAD 0")
#endif
#ifndef HEARTBEAT_EVERY
#define HEARTBEAT_EVERY 6  // 1 HEARTBEAT a cada 6 ciclos de telemetria
#endif

// Criterio de projeto: nenhum quadro com tempo no ar > 400 ms (limite de ocupacao
// do AU915/ANATEL 10.2.5.2 para canais < 250 kHz). Desligavel na serial ("DWELL 0")
// apenas para ensaios de bancada.
#ifndef DWELL_LIMIT_MS
#define DWELL_LIMIT_MS 400UL
#endif

// ---- gateway ----
#ifndef GW_TURNAROUND_MS
#define GW_TURNAROUND_MS 10UL  // espera antes de responder (no comuta TX->RX)
#endif
#ifndef GW_STATS_PERIOD_MS
#define GW_STATS_PERIOD_MS 10000UL
#endif

#define FW_VERSION "1.0.0"
