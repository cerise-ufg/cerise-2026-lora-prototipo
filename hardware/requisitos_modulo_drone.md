# Requisitos e projeto do módulo de comunicação definitivo (drone)

Objetivo 5, itens 3–4 do Plano de Trabalho — **apenas a parte de projeto**. Os valores marcados como
*estimativa* vêm de cálculo/hipótese de datasheet e devem ser confirmados com o protótipo.

## 1. Arquitetura

O módulo é um **modem LoRa transparente para o computador de bordo** do drone (o mesmo que executa o
planejamento de cobertura e a contagem de pessoas da frente de drones). O computador de bordo envia
comandos de linha pela UART (`POS lat lon alt`, `COUNT n conf`, `DONE`, `PEOPLE n`) e recebe o log em
linhas JSON (incluindo os downlinks `AREA_ASSIGN`). Assim o rádio não depende da controladora de voo,
e a integração com ROS 2/MAVLink fica no computador de bordo.

Hardware de referência: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 com TCXO) em gabinete PETG,
alimentado por 5 V do BEC, UART1 em GPIO47 (RX) / GPIO48 (TX), antena dipolo 915 MHz em SMA.

## 2. Requisitos

| ID | Requisito | Valor de projeto | Base |
|---|---|---|---|
| R1 | Mensagens | HEARTBEAT, POS, AREA_ASSIGN, COVERAGE_DONE, COUNT_REPORT, ACK (cerise_proto v1) | alinhado à frente de drones |
| R2 | Taxa de telemetria | POS a cada 5 s (±10 %) por drone; HEARTBEAT a cada 30 s | sim_canal |
| R3 | Ocupação por quadro | ≤ 400 ms → POS até SF10 (370,7 ms); AREA_ASSIGN até SF9 (246,8 ms) | AU915 / ANATEL 10.2.5.2 |
| R4 | Alcance de projeto | ≥ 3 km com margem de 10 dB, drone a 60 m (SF9: 6,0 km Hata suburbano; 3,0 km urbano) | lora_calc (*estimativa*) |
| R5 | Entrega | PDR ≥ 95 % na área de operação, com ACK para mensagens críticas | sim_canal / teste de campo |
| R6 | Acesso múltiplo | até 12 drones/canal em SF9 (TDMA, slot 401 ms) ou ~34 em SF7; CSMA/CAD + ACK até ~8 drones em SF9 | sim_canal |
| R7 | Massa | ≤ 40 g (placa + gabinete + antena + cabo); gabinete PETG ≈ 12,9 g (sólido) | gabinete.json (*estimativa*) |
| R8 | Dimensões | 55 × 44,3 × 17,2 mm com orelhas (interno 51,4 × 26,7 × 14,0 mm) | gabinete.json |
| R9 | Alimentação | 5 V ± 5 %, ≤ 0,35 W médio (≈ 0,1–0,2 % da potência de voo de ~200 W) | orcamento_energia (*estimativa*) |
| R10 | Interface | UART 3,3 V TTL, 115200 8N1, conector JST-GH 4 vias (5V, GND, TX, RX) | firmware HOST_UART1 |
| R11 | Fixação | 4 orelhas M3 (padrão 30,5 mm) ou abraçadeiras; amortecimento com fita dupla-face de espuma | gabinete.py |
| R12 | Antena | dipolo λ/2 vertical, apontada para baixo, ≥ 10 cm do GPS/bússola e de cabos de ESC, fora da sombra do frame de carbono; ganho ≤ 6 dBi | boas práticas RF / ANATEL 10.5 |
| R13 | Regulatório | equipamento homologado (ou módulo certificado) para 915–928 MHz; operação final com salto de frequência ≥ 35 canais (10.2.5) **ou** enquadramento como modulação digital (10.3: ≤ 1 W, densidade ≤ 8 dBm/3 kHz) | Ato ANATEL 14.448/2017 |
| R14 | Robustez | watchdog, detecção de reinício (seq inicial aleatório), OLED desligável em voo, proteção contra inversão de polaridade | firmware / hardware |
| R15 | Segurança da informação | autenticação + cifragem (AES-128-CCM, MIC de 4 B) antes do uso operacional — **não implementado no protótipo** | próximo passo |

## 3. Posicionamento da antena e gabinete

- Antena **vertical para baixo**, sob o trem de pouso ou na extremidade de um braço, para manter a visada
  com a estação de solo durante inclinações de até ~30° (o diagrama do dipolo tem nulo apenas no eixo).
- Afastar ≥ 10 cm do receptor GNSS (o 2º harmônico de 915 MHz não cai na banda L1, mas o acoplamento
  degrada a sensibilidade) e dos cabos de potência dos ESCs (ruído de chaveamento).
- Gabinete com janela para o OLED (diagnóstico em solo), furos de ventilação sobre o PA, SMA de painel
  no lado oposto à USB-C e passa-cabo para o chicote JST-GH.
- Estação de solo: antena 3 dBi em mastro de 2–3 m, longe de estruturas metálicas; a altura de solo e a
  altitude do drone dominam o alcance (fig_alcance_altitude.png).

## 4. Evolução prevista do firmware para o módulo definitivo

1. **TDMA sincronizado por GPS** (PPS da controladora ou do computador de bordo): slot por drone; resultado
   da simulação: PDR 100 % até 12 drones em SF9 (vs. 46 % no ALOHA puro com 10 drones).
2. ACK apenas para mensagens críticas (AREA_ASSIGN, COVERAGE_DONE, COUNT_REPORT); POS sem confirmação
   (valor mais recente substitui o anterior) — reduz a ocupação em ~45 % (ACK de 165 ms por POS de 206 ms em SF9).
3. Salto de frequência sobre os canais AU915 (64 × 125 kHz em 915,2–927,8 MHz) para conformidade 10.2.5.
4. ADR simplificado: o ACK já devolve RSSI/SNR medidos no gateway → o nó pode reduzir SF/potência.
5. Cifragem/autenticação (R15) e contador anti-replay (o seq de 16 bits passa a compor o nonce).
