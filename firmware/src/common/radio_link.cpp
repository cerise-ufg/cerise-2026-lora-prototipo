#include "radio_link.h"

#include <RadioLib.h>
#include <SPI.h>

#include "board.h"

#if defined(RADIO_SX1262)
static SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, SPI);
#elif defined(RADIO_SX1276)
static SX1276 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1, SPI);
#else
#error "Defina RADIO_SX1262 ou RADIO_SX1276"
#endif

static LoraParams g_params;
static volatile bool g_rx_flag = false;

static void IRAM_ATTR on_dio_irq() { g_rx_flag = true; }

int16_t radio_begin(const LoraParams &p) {
  g_params = p;
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
#if defined(RADIO_SX1262)
  int16_t st = radio.begin(p.freq_mhz, p.bw_khz, p.sf, p.cr_denom, p.sync_word, p.tx_dbm, p.preamble,
                           LORA_TCXO_V, false);
  if (st != RADIOLIB_ERR_NONE) return st;
  st = radio.setDio2AsRfSwitch(true);  // DIO2 controla a chave de RF na Heltec V3
  if (st != RADIOLIB_ERR_NONE) return st;
  st = radio.setCRC(2);
#else
  int16_t st = radio.begin(p.freq_mhz, p.bw_khz, p.sf, p.cr_denom, p.sync_word, p.tx_dbm, p.preamble, 0);
  if (st != RADIOLIB_ERR_NONE) return st;
  st = radio.setCRC(true);
#endif
  if (st != RADIOLIB_ERR_NONE) return st;
  radio.setPacketReceivedAction(on_dio_irq);
  return RADIOLIB_ERR_NONE;
}

int16_t radio_send(const uint8_t *data, size_t len) {
  int16_t st = radio.transmit(data, len);
  // a mesma linha de IRQ sinaliza TxDone: descarta a flag antes de voltar a RX
  g_rx_flag = false;
  return st;
}

int16_t radio_start_rx() {
  g_rx_flag = false;
  return radio.startReceive();
}

bool radio_rx_pending() { return g_rx_flag; }

int16_t radio_read(uint8_t *buf, size_t cap, size_t *len, float *rssi_dbm, float *snr_db) {
  g_rx_flag = false;
  size_t n = radio.getPacketLength();
  if (n > cap) n = cap;
  int16_t st = radio.readData(buf, n);
  if (len) *len = n;
  if (rssi_dbm) *rssi_dbm = radio.getRSSI();
  if (snr_db) *snr_db = radio.getSNR();
  return st;
}

bool radio_channel_free() {
  int16_t st = radio.scanChannel();  // CAD bloqueante (alguns simbolos)
  g_rx_flag = false;                 // a IRQ de CadDone tambem usa a linha DIO
  return st == RADIOLIB_CHANNEL_FREE;
}

int16_t radio_set_sf(uint8_t sf) {
  int16_t st = radio.setSpreadingFactor(sf);  // LDRO ajustado automaticamente pelo RadioLib
  if (st == RADIOLIB_ERR_NONE) g_params.sf = sf;
  return st;
}

int16_t radio_set_power(int8_t dbm) {
  int16_t st = radio.setOutputPower(dbm);
  if (st == RADIOLIB_ERR_NONE) g_params.tx_dbm = dbm;
  return st;
}

uint32_t radio_toa_us(size_t len) { return (uint32_t)radio.getTimeOnAir(len); }

const LoraParams &radio_params() { return g_params; }
