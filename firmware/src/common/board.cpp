#include "board.h"

void board_init() {
#ifdef PIN_VEXT
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);  // liga OLED
  delay(50);
#endif
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#ifdef PIN_VBAT_CTRL
  pinMode(PIN_VBAT_CTRL, OUTPUT);
  digitalWrite(PIN_VBAT_CTRL, !VBAT_CTRL_ACTIVE);
#endif
  analogReadResolution(12);
}

void board_led(bool on) { digitalWrite(PIN_LED, on ? HIGH : LOW); }

uint16_t board_vbat_mv() {
#ifdef PIN_VBAT_CTRL
  digitalWrite(PIN_VBAT_CTRL, VBAT_CTRL_ACTIVE);
  delay(5);
#endif
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) acc += analogReadMilliVolts(PIN_VBAT_ADC);
#ifdef PIN_VBAT_CTRL
  digitalWrite(PIN_VBAT_CTRL, !VBAT_CTRL_ACTIVE);
#endif
  float mv = (acc / 8.0f) * VBAT_DIVIDER;
  if (mv < 2500.0f || mv > 4500.0f) return 0;  // sem bateria (so USB) ou leitura invalida
  return (uint16_t)mv;
}
