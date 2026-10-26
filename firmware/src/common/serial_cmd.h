// Leitura nao bloqueante de linhas de comando da serial (USB)
#pragma once
#include <Arduino.h>

class SerialLine {
 public:
  // retorna true quando uma linha completa (sem \r\n) esta em buf()
  bool poll(Stream &s) {
    while (s.available()) {
      char c = (char)s.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf_[n_] = '\0';
        n_ = 0;
        return buf_[0] != '\0';
      }
      if (n_ < sizeof(buf_) - 1) buf_[n_++] = c;
    }
    return false;
  }
  char *buf() { return buf_; }

 private:
  char buf_[160] = {0};
  size_t n_ = 0;
};

// compara o primeiro token sem diferenciar maiusculas
inline bool cmd_is(const char *line, const char *cmd) {
  size_t n = strlen(cmd);
  return strncasecmp(line, cmd, n) == 0 && (line[n] == '\0' || line[n] == ' ');
}

// converte "1201ab..." em bytes; retorna o numero de bytes ou -1 se invalido
inline int hex_to_bytes(const char *s, uint8_t *out, size_t cap) {
  while (*s == ' ') s++;
  size_t n = 0;
  auto nib = [](char ch) -> int {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    return -1;
  };
  while (s[0] && s[0] != ' ') {
    int hi = nib(s[0]), lo = s[1] ? nib(s[1]) : -1;
    if (hi < 0 || lo < 0 || n >= cap) return -1;
    out[n++] = (uint8_t)((hi << 4) | lo);
    s += 2;
  }
  return (int)n;
}