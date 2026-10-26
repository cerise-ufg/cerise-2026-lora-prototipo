#include "mission_sim.h"

#include <math.h>

static MissionArea g_area = {-16.6810, -49.2565, -16.6790, -49.2535, 40.0f, 25.0f, 1, 1};
static double g_s = 0.0;  // distancia percorrida (m)
static const float kSpeed = 8.0f;  // m/s
static uint16_t g_people = 0;
static uint32_t g_start_ms = 0;

static double m_per_deg_lat() { return 111320.0; }
static double m_per_deg_lon() { return 111320.0 * cos((g_area.lat_min + g_area.lat_max) * 0.5 * M_PI / 180.0); }
static double width_m() { return (g_area.lon_max - g_area.lon_min) * m_per_deg_lon(); }
static double height_m() { return (g_area.lat_max - g_area.lat_min) * m_per_deg_lat(); }
static int n_rows() {
  int n = (int)floor(height_m() / g_area.spacing_m) + 1;
  return n < 1 ? 1 : n;
}
static double total_len() { return n_rows() * width_m() + (n_rows() - 1) * (double)g_area.spacing_m; }

void sim_assign(const MissionArea &a, uint32_t now_ms) {
  g_area = a;
  if (g_area.spacing_m < 1.0f) g_area.spacing_m = 1.0f;
  g_s = 0.0;
  g_people = 0;
  g_start_ms = now_ms;
}

void sim_step(float dt_s) {
  double L = total_len();
  g_s += kSpeed * dt_s;
  if (g_s > L) g_s = L;
  // ~0,15 pessoa nova por segundo de voo (apenas para exercitar os campos)
  if (g_s < L && (float)random(0, 1000) < 150.0f * dt_s) g_people++;
}

void sim_position(double *lat, double *lon, float *alt_m) {
  double w = width_m(), seg = w + g_area.spacing_m;
  int row = (int)floor(g_s / seg);
  double off = g_s - row * seg;
  double x, y;  // metros a partir do canto (lon_min, lat_min)
  if (row >= n_rows()) {
    row = n_rows() - 1;
    off = w;
  }
  if (off <= w) {  // percorrendo a faixa
    x = (row % 2 == 0) ? off : (w - off);
    y = row * (double)g_area.spacing_m;
  } else {  // transicao para a proxima faixa
    x = (row % 2 == 0) ? w : 0.0;
    y = row * (double)g_area.spacing_m + (off - w);
  }
  if (y > height_m()) y = height_m();
  *lat = g_area.lat_min + y / m_per_deg_lat();
  *lon = g_area.lon_min + x / m_per_deg_lon();
  *alt_m = g_area.alt_m;
}

float sim_coverage_pct() {
  double L = total_len();
  return L > 0 ? (float)(100.0 * g_s / L) : 100.0f;
}
bool sim_done() { return g_s >= total_len(); }
uint16_t sim_people() { return g_people; }
void sim_set_people(uint16_t n) { g_people = n; }
uint32_t sim_elapsed_s(uint32_t now_ms) { return (now_ms - g_start_ms) / 1000UL; }
const MissionArea &sim_area() { return g_area; }
