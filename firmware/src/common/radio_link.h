// Camada fina sobre o RadioLib, comum a SX1262 (Heltec V3) e SX1276 (TTGO v2.1)
#pragma once
#include <Arduino.h>

struct LoraParams {
  float freq_mhz;
  float bw_khz;
  uint8_t sf;
  uint8_t cr_denom;
  int8_t tx_dbm;
  uint16_t preamble;
  uint8_t sync_word;
};

// inicializa SPI + radio; retorna codigo RadioLib (0 = RADIOLIB_ERR_NONE)
int16_t radio_begin(const LoraParams &p);
// transmissao bloqueante; retorna codigo RadioLib
int16_t radio_send(const uint8_t *data, size_t len);
// coloca em recepcao continua com interrupcao de pacote recebido
int16_t radio_start_rx();
// true quando ha pacote pendente (flag setada na ISR)
bool radio_rx_pending();
// le o pacote pendente; retorna codigo RadioLib (CRC de camada fisica incluso)
int16_t radio_read(uint8_t *buf, size_t cap, size_t *len, float *rssi_dbm, float *snr_db);
// deteccao de atividade no canal (CAD); true = livre. Deixa o radio em standby.
bool radio_channel_free();
int16_t radio_set_sf(uint8_t sf);
int16_t radio_set_power(int8_t dbm);
// tempo no ar calculado pelo RadioLib (us), para conferencia com cp_airtime_us()
uint32_t radio_toa_us(size_t len);
const LoraParams &radio_params();
