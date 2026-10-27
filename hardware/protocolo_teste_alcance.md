# Protocolo de teste de alcance e PDR (enlace LoRa drone–solo)

> **Status:** procedimento pronto, **ensaio físico pendente** — depende da aquisição dos módulos
> (2 × Heltec WiFi LoRa 32 V3 + antenas, ver `BOM.csv`), prevista no Plano de Trabalho.

## 1. Objetivo

Medir, em campo, a taxa de entrega de pacotes (PDR), o RSSI e o SNR do enlace entre o nó (lado drone)
e o gateway (estação de solo) em função da distância, do fator de espalhamento (SF) e do tamanho do
quadro, e comparar com o orçamento de enlace calculado (`tools/lora_calc.py`, `resultados/enlace_alcance.csv`).

## 2. Material

- 2 × Heltec V3 com antenas 915 MHz (gateway: 3 dBi em mastro de 2 m; nó: dipolo 2 dBi), cabos USB-C;
- notebook com o repositório, PlatformIO e `tools/gateway_dashboard.py`;
- bateria LiPo 1S ou power bank para o nó; módulo GPS opcional (firmware `heltec_v3_node_gps`);
- trena a laser / GPS de celular para conferir distâncias; planilha de campo (seção 7).

## 3. Gravação do firmware (quando as placas chegarem)

```powershell
cd firmware
pio run -e heltec_v3_gateway -t upload --upload-port COM6     # placa 1 (solo)
pio run -e heltec_v3_node    -t upload --upload-port COM7     # placa 2 (drone)  [ou heltec_v3_node_gps]
pio device monitor -e heltec_v3_gateway --port COM6           # conferir a linha {"evt":"boot",...}
```

Conferências de bancada (antes do campo, placas a ~2 m, potência reduzida `POWER 2`):
1. boot sem erro de rádio (`"rc":0`) nas duas placas e mesmo `freq_mhz`/`sf`;
2. o gateway imprime `POS`/`HEARTBEAT` com `"dup":false` e o nó imprime `{"evt":"ack",...}`;
3. comando no gateway `ASSIGN 1 3 7 -16.681 -49.2565 -16.679 -49.2535 40 25` → o nó imprime
   `{"evt":"dl",...}` e passa a reportar `area=3`;
4. desconectar a antena do **gateway** por 20 s (nunca a do transmissor ligado) → o nó registra
   `timeout`/`drop`; ao reconectar, a entrega volta.

## 4. Matriz de ensaio

| Variável | Valores |
|---|---|
| Distância (visada direta, solo plano) | 50, 100, 200, 500, 1000, 2000, 3000 m e o máximo alcançável |
| SF | 7, 9, 10 (SF11/12 apenas como referência de sensibilidade, com `DWELL 0`: excedem 400 ms) |
| Potência | 14 dBm (padrão); 20 dBm só no ponto de maior distância |
| Quadro | POS (24 B) — padrão; repetir no ponto mais distante com AREA_ASSIGN (32 B) via `ASSIGN`, só com SF ≤ 9 (452,6 ms em SF10; o gateway não aplica o limite de 400 ms aos downlinks) |
| Altura do nó | 1,5 m (tripé) e, quando autorizado, drone pairando a 30 / 60 / 100 m AGL |
| Repetições | 200 quadros por ponto (intervalo 1 s) |

Com n = 200 e PDR ≈ 0,9, a meia-largura do intervalo de 95 % é ≈ ±4,2 p.p.; para PDR ≈ 0,99, ≈ ±1,4 p.p.

## 5. Procedimento por ponto

1. Posicionar o nó, anotar coordenadas (GPS) e distância; ajustar o SF **nas duas placas** (`SF 9`).
2. No notebook: `python tools/gateway_dashboard.py --port COM6 --label "d500_sf9_h1.5" --out resultados/campo/d500_sf9`.
3. No nó: `RANGETEST 200 1000` (200 quadros POS com flag TESTE, sem ACK/retentativa → PDR bruta do enlace).
4. Após `{"evt":"rangetest_end"}` encerrar o dashboard (Ctrl+C). O `resumo.json` traz PDR (lacunas de seq),
   RSSI e SNR médios/mínimos; repetir com ACK habilitado (telemetria normal por 5 min) para medir a PDR
   após retentativas, a latência (`lat_ms` nos eventos `ack` do nó) e o número de retransmissões.
5. Registrar condições: clima, obstáculos, orientação das antenas, altura.

## 6. Medição de consumo (substitui a estimativa)

Medidor USB entre a fonte e o nó: corrente média em (a) RX contínuo, (b) telemetria a cada 5 s em SF7/9/10,
(c) `POWER 22`; preencher `resultados/consumo_medido.csv` e comparar com `resultados/orcamento_energia.json`.

## 7. Planilha de resultados (modelo)

`distancia_m, altura_no_m, sf, tx_dbm, quadro_B, enviados, recebidos, pdr, rssi_med_dBm, rssi_min_dBm, snr_med_dB, snr_min_dB, pdr_com_ack, latencia_med_ms, obs`

## 8. Segurança e regulamentação

- Voo apenas com drone cadastrado/autorizado, em linha de visada, até 120 m AGL, seguindo RBAC-E nº 94 (ANAC),
  ICA 100-40 (DECEA) e as regras do campus; nunca energizar o rádio sem antena.
- Rádio na faixa de 915–928 MHz (radiação restrita, Resolução ANATEL nº 680/2017 e Ato nº 14.448/2017):
  potência ≤ 1 W, antena ≤ 6 dBi; protótipo usado apenas em ensaios de curta duração.
