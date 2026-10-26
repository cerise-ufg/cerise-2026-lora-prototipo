// Mapeamento de pinos e funcoes de placa (Heltec V3 / TTGO LoRa32 v2.1)
#pragma once
#include <Arduino.h>

#if defined(BOARD_HELTEC_V3)
// Heltec WiFi LoRa 32 V3: ESP32-S3FN8 + SX1262 (TCXO 1,8 V) + OLED SSD1306 0,96"
#define BOARD_NAME "Heltec WiFi LoRa 32 V3"
#define RADIO_NAME "SX1262"
#define PIN_LORA_NSS 8
#define PIN_LORA_SCK 9
#define PIN_LORA_MOSI 10
#define PIN_LORA_MISO 11
#define PIN_LORA_RST 12
#define PIN_LORA_BUSY 13
#define PIN_LORA_DIO1 14
#define LORA_TCXO_V 1.8f
#define PIN_OLED_SDA 17
#define PIN_OLED_SCL 18
#define PIN_OLED_RST 21
#define PIN_VEXT 36  // LOW liga a alimentacao do OLED (Vext)
#define PIN_LED 35
#define PIN_VBAT_ADC 1
#define PIN_VBAT_CTRL 37  // habilita o divisor de tensao da bateria
#define VBAT_CTRL_ACTIVE LOW  // V3.0/V3.1 (na V3.2 o nivel ativo e HIGH)
#define VBAT_DIVIDER 4.9f     // 390k / 100k
#define PIN_GPS_RX 47  // opcional: TX do modulo GPS -> GPIO47
#define PIN_GPS_TX 48

#elif defined(BOARD_TTGO_V21)
// LILYGO TTGO LoRa32 v2.1 (T3 v1.6.1): ESP32-PICO-D4 + SX1276 + OLED SSD1306
#define BOARD_NAME "TTGO LoRa32 v2.1"
#define RADIO_NAME "SX1276"
#define PIN_LORA_NSS 18
#define PIN_LORA_SCK 5
#define PIN_LORA_MOSI 27
#define PIN_LORA_MISO 19
#define PIN_LORA_RST 23
#define PIN_LORA_DIO0 26
#define PIN_LORA_DIO1 33
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22
#define PIN_OLED_RST -1
#define PIN_LED 25
#define PIN_VBAT_ADC 35
#define VBAT_DIVIDER 2.0f  // 100k / 100k
#define PIN_GPS_RX 34      // entrada apenas
#define PIN_GPS_TX 13

#else
#error "Defina BOARD_HELTEC_V3 ou BOARD_TTGO_V21"
#endif

void board_init();
void board_led(bool on);
// tensao da bateria em mV (0 se a leitura indicar ausencia de bateria)
uint16_t board_vbat_mv();
