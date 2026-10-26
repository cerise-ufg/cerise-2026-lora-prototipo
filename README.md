# CERISE 2026 — Protótipo de comunicação LoRa drone ↔ solo

Repositório da **Atividade 10 do Objetivo 3** do Plano de Trabalho CERISE 2026 — *Pesquisa, Desenvolvimento e
Inovação em Gêmeos Digitais, Tecnologias Imersivas, Robótica e Navegação Autônoma no escopo do CERISE/EMC/UFG*
(*desenvolvimento de protótipo de comunicação LoRa: envio e recebimento de dados entre dois dispositivos*) e do
**projeto dos módulos de comunicação definitivos para os drones** (Objetivo 5, itens 3–4, parte de projeto).

- **Autor:** Eduardo Martins de Carvalho Caixeta (bolsista, CERISE/EMC/UFG)
- **Coordenação:** Prof. Dr. Alisson Assis Cardoso (Navegação Autônoma)
- **Texto técnico completo:** [`RELATORIO_SECAO.md`](RELATORIO_SECAO.md) (16 figuras em `evidencias/`)

> **Estado (26/10/2026):** firmware compilado para 5 ambientes; protocolo com 30 testes unitários nativos e 21 testes
> Python (validação cruzada C × Python e compatibilidade com a frente de drones); simulação de canal com 1 130
> execuções; dashboard testado com fluxo serial simulado; projeto do módulo (BOM, ligação, gabinete STL, energia,
> requisitos, protocolo de ensaio) concluído. **O ensaio físico entre duas placas depende da aquisição dos módulos**
> (2 × Heltec V3, ver `hardware/BOM.csv`); os comandos de gravação e o protocolo de teste estão prontos.
> Nenhuma medição de rádio foi feita: alcance e consumo são estimativas, rotuladas como tais.

## Estrutura

```
firmware/                    PlatformIO (Arduino + RadioLib 7.8.1 + U8g2 2.36.18)
  platformio.ini             5 ambientes ESP32 + ambiente "native" (testes no PC)
  lib/cerise_proto/          protocolo binário versionado em C99 (encode/decode, CRC16, ACK, dedup, JSON, airtime, AU915)
  src/node_main.cpp          papel NODE (drone): telemetria, ACK + retentativas, CAD, downlink, teste de alcance
  src/gateway_main.cpp       papel GATEWAY (solo): validação, ACK c/ RSSI/SNR, PDR por nó, JSON na serial, OLED
  src/common/                pinos das placas, camada RadioLib, OLED, comandos seriais, trajetória sintética
  test/test_proto/           30 testes unitários Unity (pio test -e native)
  binarios/<ambiente>/       bootloader.bin, partitions.bin e firmware.bin compilados (+ SHA256SUMS.txt, README)
tools/
  cerise_proto.py            espelho Python do protocolo (validado contra o C)
  lora_calc.py               tempo no ar (Semtech) + orçamento de enlace / alcance (Friis, log-distância, Okumura-Hata)
  sim_canal.py               simulação de eventos discretos: N drones, ALOHA × ALOHA+ACK × CSMA/CAD × slotted × TDMA
  gateway_dashboard.py       leitor serial do gateway: JSON → CSV/JSONL + PDR + painel PNG
  sim_serial_gateway.py      fluxo serial simulado do gateway (TCP no loopback / stdout)
  fig_protocolo.py           figuras do formato do quadro e da sequência de mensagens
  plot_style.py              estilo comum das figuras (matplotlib, 150 dpi)
  c/gen_vectors.c            gerador de vetores de teste com a implementação C
  tests/                     pytest (validação cruzada, compatibilidade com drones, dashboard) + vectors.tsv
  requirements.txt           dependências Python
hardware/
  BOM.csv                    lista de materiais (bancada e módulo definitivo)
  diagrama_ligacao.py        diagrama de ligação (bancada e módulo no drone)
  gabinete/gabinete.py       gabinete paramétrico (trimesh + manifold3d) → gabinete_base.stl / gabinete_tampa.stl
  orcamento_energia.py       estimativa de consumo do módulo
  protocolo_teste_alcance.md procedimento do ensaio de alcance/PDR
  requisitos_modulo_drone.md requisitos R1–R15 e projeto do módulo definitivo
evidencias/                  figuras e prints de terminal usados no relatório (16 PNG)
resultados/                  CSV/JSON/logs medidos ou calculados pelas ferramentas
  dash_sim/                  captura do dashboard com fluxo simulado (JSONL bruto, CSV, resumo, painel)
RELATORIO_SECAO.md           seção do relatório técnico (A10 + projeto dos módulos)
RELEASES.md                  artefatos anexados ao Release (ELF/MAP de depuração) e somas SHA-256
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

## Como reproduzir

Pré-requisitos (Windows, como usado): PlatformIO Core 6.1.19 (`%USERPROFILE%\.platformio\penv\Scripts\pio.exe`),
um GCC no PATH para os testes nativos (usado: WinLibs GCC 16.2 UCRT) e Python 3.11 com `tools/requirements.txt`.
As bibliotecas do firmware (RadioLib, U8g2, TinyGPSPlus, Unity) e a plataforma `espressif32@6.12.0` são baixadas
automaticamente pelo PlatformIO na primeira compilação, nas versões fixadas em `firmware/platformio.ini`.

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

O teste `test_compat_with_drones_front` decodifica os quadros de exemplo do pacote `cerise_drones` (repositório da
frente de drones/segmentação). Ele procura `cerise_drones/protocolo.py` na pasta indicada pela variável de ambiente
`CERISE_DRONES_DIR` ou, na falta dela, na pasta irmã `DRONES_SEGMENTACAO`; se não encontrar, o teste é pulado
(*skip*) e os demais 20 continuam valendo. Os testes de validação cruzada recompilam `tools/c/gen_vectors.c` se houver
`gcc` no PATH; caso contrário, usam `tools/tests/vectors.tsv` versionado.

### Com as placas (quando adquiridas)

```powershell
cd firmware
pio run -e heltec_v3_gateway -t upload --upload-port COM6
pio run -e heltec_v3_node    -t upload --upload-port COM7
python ..\tools\gateway_dashboard.py --port COM6 --echo --out ..\resultados\bancada
```

Os binários já compilados podem ser gravados sem recompilar — ver `firmware/binarios/README.md`.

Comandos seriais do **nó**: `STATUS`, `SF n`, `POWER dBm`, `PERIOD ms`, `RETRIES n`, `DWELL ms|0`, `CAD 0|1`,
`POS lat lon alt`, `PEOPLE n`, `COUNT n conf`, `DONE`, `RANGETEST n intervalo_ms`, `TX <hex>`.
Comandos do **gateway**: `STATS`, `SF n`, `POWER dBm`,
`ASSIGN nó área tarefa lat_min lon_min lat_max lon_max alt_m espaç_m`, `TX <hex>` (quadro já codificado, p. ex. pelo
`cerise_drones/protocolo.py`; é validado antes de ir ao ar).

O ensaio de alcance/PDR em campo segue `hardware/protocolo_teste_alcance.md` (200 quadros por ponto, de 50 m a
3 km, SF7/9/10).

## Resultados principais

Todos os números vêm de arquivos em `resultados/` (detalhes e discussão em `RELATORIO_SECAO.md`).

| Resultado | Valor | Fonte |
|---|---|---|
| Testes unitários nativos (Unity) | 30/30 aprovados | `resultados/pio_test_native.txt` |
| Testes pytest | 21/21 aprovados | `resultados/pytest_tools.txt` |
| Vetores idênticos C × Python (quadros / JSON / corrompidos / tempo no ar) | 3 000 / 3 000 / 3 000 / 9 360 | `tools/tests/vectors.tsv` |
| RAM / Flash `heltec_v3_node` | 22 400 B (6,8 %) / 376 241 B (11,3 %) | `resultados/build_all.log` |
| RAM / Flash `heltec_v3_gateway` | 30 552 B (9,3 %) / 355 821 B (10,6 %) | `resultados/build_all.log` |
| RAM / Flash `heltec_v3_node_gps` | 22 552 B (6,9 %) / 378 489 B (11,3 %) | `resultados/build_all.log` |
| RAM / Flash `ttgo_v21_node` | 25 228 B (7,7 %) / 376 277 B (28,7 %) | `resultados/build_all.log` |
| RAM / Flash `ttgo_v21_gateway` | 33 424 B (10,2 %) / 358 045 B (27,3 %) | `resultados/build_all.log` |
| Tempo no ar do POS (24 B) em SF7 / SF9 / SF10 | 61,7 / 205,8 / 370,7 ms | `resultados/airtime_mensagens.csv` |
| Alcance estimado SF9, 14 dBm, drone a 60 m (Hata suburbano / urbano) | 5,99 / 3,00 km | `resultados/enlace_alcance.csv` |
| PDR simulada N = 10, SF9: ALOHA / ALOHA+ACK / CSMA+ACK / slotted / TDMA+ACK | 46,0 / 15,1 / 62,6 / 67,2 / 100,0 % | `resultados/sim_canal_vs_N.csv` |
| Capacidade TDMA+ACK com PDR de 100 % (SF9, POS a cada 5 s) | 12 drones | `resultados/sim_canal_vs_N.csv` |
| Erro do simulador × teoria do ALOHA puro (médio / máximo) | 0,56 / 1,47 p.p. | `resultados/sim_canal_validacao_aloha.csv` |
| PDR do dashboard com fluxo serial simulado (drones 1 / 2 / 3) | 98,6 / 97,8 / 94,8 % | `resultados/dash_sim/resumo.json` |
| Gabinete: dimensões externas / massa estimada (PETG sólido) | 55,0 × 44,3 × 17,2 mm / 12,9 g | `resultados/gabinete.json` |
| Consumo estimado do módulo (entrada de 5 V) | 0,23–0,34 W | `resultados/orcamento_energia.json` |

Conclusão da simulação que alterou o firmware: ACK com retransmissão no mesmo canal piora a entrega sob carga; o
recuo de retransmissão passou a ser U(0, (ToA + espera do ACK)·2ᵏ), e a recomendação final é TDMA sincronizado por
GPS com ACK seletivo.

## Artefatos em Releases

Os arquivos de depuração da compilação (`firmware.elf` e `firmware.map` dos 5 ambientes, ~111 MB descompactados) e o
executável dos testes nativos estão no pacote `lora-firmware-v1-elf-map.zip` (20,7 MiB) anexado ao Release
`v1.0-prototipo`. Tamanho, SHA-256 e instruções de download/extração em [`RELEASES.md`](RELEASES.md). Os `.bin`
graváveis estão versionados em `firmware/binarios/`.

## O que não está aqui e por quê

- **Saídas de compilação e caches** (`firmware/.pio/`, `.pytest_cache/`, `__pycache__/`, ambientes virtuais Python):
  são regeneradas por `pio run`, `pio test` e `pip install`. As exceções úteis estão em `firmware/binarios/` e no Release.
- **Bibliotecas e ferramentas de terceiros** — não são redistribuídas; o PlatformIO baixa as versões fixadas:
  RadioLib 7.8.1 (`jgromes/RadioLib`), U8g2 2.36.18 (`olikraus/U8g2`), TinyGPSPlus 1.1.0 (`mikalhart/TinyGPSPlus`),
  Unity 2.6.1 (ThrowTheSwitch), plataforma `espressif32@6.12.0` com framework Arduino-ESP32 2.0.17
  (`framework-arduinoespressif32 3.20017.241212`) e o `boot_app0.bin` desse framework. O GCC 16.2 dos testes nativos
  é o WinLibs (UCRT, r3), distribuído por Brecht Sanders (winlibs.com / GitHub `brechtsanders/winlibs_mingw`).
- **Medições de rádio em campo**: ainda não existem, porque as placas não foram adquiridas. O `.gitignore` já reserva
  `resultados/campo/**/brutos.jsonl` para capturas brutas grandes; quando houver, os resumos entram no git e os brutos
  vão para o Release.
- **Segredos**: o projeto não usa chaves, tokens nem senhas; nada foi omitido por esse motivo. O protocolo atual não tem
  cifragem nem autenticação (ver limitações no relatório).

## Pendências

1. Aquisição de 2 × Heltec V3 e antenas de 915 MHz (`hardware/BOM.csv`), gravação do firmware e ensaio de alcance/PDR
   conforme `hardware/protocolo_teste_alcance.md`.
2. Medição de consumo e massa reais; conferência das dimensões da Heltec V3, impressão e ajuste do gabinete.
3. TDMA sincronizado por GPS e ACK seletivo, conforme a simulação.
4. Salto de frequência sobre os canais AU915 (≥ 35 canais) ou enquadramento como modulação digital, e homologação
   na ANATEL.
5. Cifragem e autenticação (AES-128-CCM).
6. Integração com o computador de bordo da frente de drones pela UART (GPIO47/48, JSON).

## Licença

Este repositório não inclui arquivo `LICENSE`, por decisão do autor. Para uso ou redistribuição do código, consultar
o autor.
