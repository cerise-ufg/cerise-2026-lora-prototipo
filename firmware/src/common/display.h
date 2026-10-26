// OLED SSD1306 128x64 (U8g2) — status local do no/gateway
#pragma once
#include <Arduino.h>

void display_begin();
// desenha ate 6 linhas de texto (fonte 6x10); NULL = linha vazia
void display_lines(const char *l0, const char *l1 = nullptr, const char *l2 = nullptr,
                   const char *l3 = nullptr, const char *l4 = nullptr, const char *l5 = nullptr);
// barra de RSSI (-130..-30 dBm) na ultima linha
void display_rssi_bar(int rssi_dbm);
void display_flush();
