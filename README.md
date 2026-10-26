# CERISE 2026 — Protótipo de comunicação LoRa drone ↔ solo

Repositório da **Atividade 10 do Objetivo 3** do Plano de Trabalho CERISE/EMC/UFG 2026
(*desenvolvimento de protótipo de comunicação LoRa: envio e recebimento de dados entre dois dispositivos*)
e do **projeto dos módulos de comunicação definitivos para os drones** (Objetivo 5, itens 3–4, parte de projeto).

Bolsista: Eduardo Martins de Carvalho Caixeta · Coordenação: Prof. Dr. Alisson Assis Cardoso.

> **Estado:** firmware compilado para 5 ambientes, protocolo com 30 testes unitários nativos e 21 testes
> Python (validação cruzada C × Python e compatibilidade com a frente de drones), simulação de canal com 1 130 execuções, dashboard testado com
> fluxo serial simulado. **O ensaio físico depende da aquisição dos módulos** (2 × Heltec V3, ver
> `hardware/BOM.csv`); os comandos de gravação e o protocolo de teste estão prontos.

## Estrutura

```
firmware/                    PlatformIO (Arduino + RadioLib 7.8.1 + U8g2)
  lib/cerise_proto/          protocolo binário versionado em C99 (encode/decode, CRC16, ACK, dedup, JSON, airtime)
  src/node_main.cpp          papel NODE (drone): telemetria, ACK + retentativas, CAD, downlink, teste de alcance
  src/gateway_main.cpp       papel GATEWAY (solo): validação, ACK c/ RSSI/SNR, PDR por nó, JSON na serial, OLED
  src/common/                pinos das placas, camada RadioLib, OLED, gerador de trajetória sintética
  test/test_proto/           testes unitários Unity (pio test -e native)
tools/
  cerise_proto.py            espelho Python do protocolo (validado contra o C)
  lora_calc.py               tempo no ar (Semtech) + orçamento de enlace / alcance (Friis, log-distância, Okumura-Hata)
  sim_canal.py               simulação de eventos discretos: N drones, ALOHA × ALOHA+ACK × CSMA/CAD × slotted × TDMA
  gateway_dashboard.py       leitor serial do gateway: JSON → CSV/JSONL + PDR + painel PNG
  sim_serial_gateway.py      fluxo serial simulado do gateway (TCP no loopback / stdout)
  fig_protocolo.py           figuras do formato do quadro e da sequência de mensagens
  c/gen_vectors.c            gerador de vetores de teste com a implementação C
  tests/                     pytest (validação cruzada e dashboard)
hardware/
  BOM.csv                    lista de materiais
  diagrama_ligacao.py        diagrama de ligação (bancada e módulo no drone)
  gabinete/gabinete.py       gabinete paramétrico (trimesh + manifold3d) → STL
  orcamento_energia.py       estimativa de consumo do módulo
  protocolo_teste_alcance.md procedimento do ensaio de alcance/PDR
  requisitos_modulo_drone.md requisitos e projeto do módulo definitivo
evidencias/                  figuras e prints usados no relatório
resultados/                  CSV/JSON/logs medidos ou calculados pelas ferramentas
RELATORIO_SECAO.md           seção do relatório técnico (A10 + projeto dos módulos)
```

## Parâmetros do enlace

| Parâmetro | Valor padrão |
|---|---|
| Frequência | 916,8 MHz (canal 8 do plano AU915-928, adotado no Brasil) |
| Modulação | LoRa, SF9 (7–12 via serial), BW 125 kHz, CR 4/5, preâmbulo 8, CRC de PHY ligado, sync word 0x12 |
| Potência | 14 dBm (até 22 dBm no SX1262) |
| Ocupação | nenhum quadro > 400 ms (POS de 24 B: 61,7 ms SF7 · 205,8 ms SF9 · 370,7 ms SF10) |
| Telemetria | POS a cada 5 s ± 10 %, HEARTBEAT a cada 6 ciclos; ACK + até 3 retentativas |

## Protocolo `cerise_proto` v1

Cabeçalho de 8 B (`ver|tipo`, `flags`, `net`, `src`, `dst`, `seq` u16, `len`) + payload de layout fixo +
CRC-16/CCITT-FALSE. Tipos: `HEARTBEAT`(8 B), `POS`(14 B), `AREA_ASSIGN`(22 B), `COVERAGE_DONE`(7 B),
`COUNT_REPORT`(13 B), `ACK`(6 B) — nomes alinhados com a frente de drones — e a extensão proposta por ela para
a colaboração: `AREA_POLY`(4 + 4n B), `TASK_BID`(4 B), `TASK_AWARD`(3 B). Detalhes em
`firmware/lib/cerise_proto/src/cerise_proto.h` e `evidencias/fig_formato_quadro.png`.

## Como rodar

Pré-requisitos (Windows): PlatformIO Core 6.1 (`%USERPROFILE%\.platformio\penv\Scripts\pio.exe`), um GCC
no PATH para os testes nativos (usado: WinLibs GCC 16.2 UCRT) e Python 3.11 com `tools/requirements.txt`.

```powershell
# firmware: compilar todos os ambientes e rodar os testes unitários do protocolo no PC
cd firmware
pio run                       # heltec_v3_node, heltec_v3_gateway, ttgo_v21_node, ttgo_v21_gateway, heltec_v3_node_gps
pio test -e native -v

# ferramentas
cd ..
python -m pip install -r tools/requirements.txt
python -m pytest -v tools/tests
python tools/lora_calc.py                       # tabelas + fig_airtime / fig_alcance_*
python tools/sim_canal.py                       # ~5 min; --quick para versão curta; --plot-only refaz as figuras
python tools/fig_protocolo.py
python hardware/diagrama_ligacao.py
python hardware/gabinete/gabinete.py            # STL em hardware/gabinete/
python hardware/orcamento_energia.py

# dashboard com fluxo serial simulado (dois terminais)
python tools/sim_serial_gateway.py --port 8195 --speed 10 --duration 600 --drones 3
python tools/gateway_dashboard.py --port socket://127.0.0.1:8195 --duration 75 --out resultados/dash_sim "--gw-pos=-16.6799,-49.2550"
```

### Com as placas (quando adquiridas)

```powershell
cd firmware
pio run -e heltec_v3_gateway -t upload --upload-port COM6
pio run -e heltec_v3_node    -t upload --upload-port COM7
python ..\tools\gateway_dashboard.py --port COM6 --echo --out ..\resultados\bancada
```

Comandos seriais do **nó**: `STATUS`, `SF n`, `POWER dBm`, `PERIOD ms`, `RETRIES n`, `DWELL ms|0`, `CAD 0|1`,
`POS lat lon alt`, `PEOPLE n`, `COUNT n conf`, `DONE`, `RANGETEST n intervalo_ms`, `TX <hex>`.
Comandos do **gateway**: `STATS`, `SF n`, `POWER dBm`,
`ASSIGN nó área tarefa lat_min lon_min lat_max lon_max alt_m espaç_m`, `TX <hex>` (quadro já codificado, p. ex. pelo
`cerise_drones/protocolo.py`; é validado antes de ir ao ar).

## Resultados principais (ver `RELATORIO_SECAO.md`)

- 30/30 testes unitários nativos e 21/21 testes Python; 3 000 quadros aleatórios idênticos byte a byte entre C e Python.
- Firmware: RAM 6,8–10,2 % e Flash 10,6–28,7 % conforme o ambiente (`resultados/build_all.log`).
- Simulação (SF9, POS a cada 5 s): com 10 drones, ALOHA puro entrega 46,0 %, ALOHA+ACK 15,1 %,
  CSMA/CAD+ACK 62,6 %, slotted ALOHA 67,2 % e TDMA+ACK 100 % (TDMA comporta até 12 drones em SF9).
- Alcance estimado (14 dBm, drone a 60 m, margem 10 dB, SF9): 6,0 km (Hata suburbano) / 3,0 km (urbano).

## Licença

MIT (código desenvolvido no âmbito do CERISE/EMC/UFG).
