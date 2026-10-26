// Gerador de telemetria sintetica para o no (sem GPS / sem computador de bordo):
// percorre um retangulo em zigue-zague (boustrophedon), como no planejamento de
// cobertura da frente de drones, e incrementa a contagem de pessoas aleatoriamente.
#pragma once
#include <Arduino.h>

struct MissionArea {
  double lat_min, lon_min, lat_max, lon_max;
  float alt_m;
  float spacing_m;
  uint8_t area_id;
  uint8_t task_id;
};

void sim_assign(const MissionArea &a, uint32_t now_ms);
void sim_step(float dt_s);
void sim_position(double *lat, double *lon, float *alt_m);
float sim_coverage_pct();
bool sim_done();
uint16_t sim_people();
void sim_set_people(uint16_t n);
uint32_t sim_elapsed_s(uint32_t now_ms);
const MissionArea &sim_area();
