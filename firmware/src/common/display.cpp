#include "display.h"

#include <U8g2lib.h>
#include <Wire.h>

#include "board.h"

#if PIN_OLED_RST >= 0
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);
#endif

void display_begin() {
  u8g2.setBusClock(400000);  // I2C rapido: um quadro completo leva ~25 ms (nao atrasa o ACK)
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void display_lines(const char *l0, const char *l1, const char *l2, const char *l3, const char *l4,
                   const char *l5) {
  const char *ls[6] = {l0, l1, l2, l3, l4, l5};
  u8g2.clearBuffer();
  for (int i = 0; i < 6; i++) {
    if (ls[i]) u8g2.drawStr(0, i * 10 + 1, ls[i]);
  }
}

void display_rssi_bar(int rssi_dbm) {
  int v = rssi_dbm;
  if (v < -130) v = -130;
  if (v > -30) v = -30;
  int w = (v + 130) * 127 / 100;
  u8g2.drawFrame(0, 61, 128, 3);
  u8g2.drawBox(0, 61, w, 3);
}

void display_flush() { u8g2.sendBuffer(); }
