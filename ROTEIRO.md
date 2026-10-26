# Roteiro de implementação — Protótipo de comunicação LoRa

## O que será reproduzido

Este roteiro reproduz o protótipo de comunicação LoRa entre um nó no drone (papel *node*) e uma estação de solo (papel *gateway*), da Atividade 10 do Objetivo 3, e o projeto do módulo de comunicação dos drones (Objetivo 5, itens 3 e 4, só a parte de projeto). O software foi verificado sem rádio, por compilação, testes, validação cruzada C × Python e simulação.

**O ensaio físico entre duas placas ainda não foi realizado**, porque as placas não foram adquiridas. As seções de montagem, gravação e ensaio descrevem o que deve ser feito quando elas chegarem; quem as executar fará a primeira medição real do enlace.

Ao final, você terá:

- o firmware compilado para os 5 ambientes (Heltec WiFi LoRa 32 V3 e TTGO LoRa32 v2.1, nó e gateway, e nó com GPS);
- os 30 testes nativos do protocolo `cerise_proto` e os 21 testes Python aprovados;
- as tabelas de tempo no ar e de alcance, a simulação de vários drones no canal e o dashboard alimentado por um fluxo serial simulado;
- os STL do gabinete do módulo e as estimativas de massa e consumo;
- a bancada de duas placas gravada e o ensaio de alcance e de taxa de entrega de pacotes (PDR), ainda inédito.

## Requisitos

- **Computador:** PC x86_64 com Windows 10 ou 11 (usado: notebook com Windows 11). Não há dependência de GPU, de aarch64 nem da estação DGX Spark. Linux e macOS devem funcionar, mas não foram testados.
- **Software:** Git; GitHub CLI (`gh`), opcional; PlatformIO Core 6.1.19; Python 3.11 (usado 3.11.9); GCC para os testes nativos (usado WinLibs GCC 16.2.0, UCRT, r3).
- **Bibliotecas do firmware**, baixadas pelo PlatformIO nas versões de `firmware/platformio.ini`: `espressif32@6.12.0` (Arduino-ESP32 2.0.17), RadioLib 7.8.1, U8g2 2.36.18, TinyGPSPlus 1.1.0 e Unity 2.6.1.
- **Acesso:** conta no GitHub com acesso à organização `cerise-ufg` (repositório privado; solicite ao bolsista). O projeto não usa chaves nem senhas.
- **Disco:** cerca de 5 GB (pacotes do PlatformIO de 1,5 a 2,4 GB, `firmware/.pio` com 0,85 GB, WinLibs com 0,95 GB, ambiente Python com 0,4 GB).
- **Tempo:** 1 h para preparar e compilar pela primeira vez; 15 min para testes, ferramentas e dashboard; 1 a 2 h para a bancada; um dia de campo para o ensaio (estimativa).

## Obtenção do código e dos dados

1. Clone o repositório e confira os binários versionados em `firmware/binarios/` (no Git Bash ou no Linux); as 15 linhas devem terminar em `OK`:

```bash
git clone https://github.com/cerise-ufg/cerise-2026-lora-prototipo.git
cd cerise-2026-lora-prototipo/firmware/binarios
sha256sum -c SHA256SUMS.txt
```

2. Opcionalmente, numa pasta fora do repositório, baixe o Release `v1.0-prototipo` (pelo `gh`, após `gh auth login`, ou pela página Releases). Ele traz só arquivos de depuração (`firmware.elf` e `firmware.map` dos 5 ambientes e o executável dos testes nativos), sem os quais tudo funciona:

```powershell
gh release download v1.0-prototipo -R cerise-ufg/cerise-2026-lora-prototipo -p "lora-firmware-v1-elf-map.zip"
(Get-FileHash .\lora-firmware-v1-elf-map.zip -Algorithm SHA256).Hash.ToLower()
# deve ser 2148810e3fe4d4a3e2e1458f68d98f5c1ba44dbb7bc376208c413578c45a7e17
Expand-Archive .\lora-firmware-v1-elf-map.zip -DestinationPath .\lora-firmware-v1
```

No Linux, use `sha256sum` e `unzip lora-firmware-v1-elf-map.zip -d lora-firmware-v1`; o `SHA256SUMS.txt` interno confere cada arquivo extraído.

## Preparação do ambiente

1. Instale o PlatformIO Core 6.1.19 (usado: extensão PlatformIO IDE do VS Code, que cria `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`; também há o pacote `platformio` do pip) e ponha-o no PATH:

```powershell
$env:Path = "$env:USERPROFILE\.platformio\penv\Scripts;$env:Path"
pio --version      # PlatformIO Core, version 6.1.19
```

2. No Windows, instale o WinLibs (winlibs.com ou GitHub `brechtsanders/winlibs_mingw`) e ponha a pasta `mingw64\bin` no PATH; no Linux, use o GCC da distribuição (não testado):

```powershell
$env:Path = "C:\caminho\para\mingw64\bin;$env:Path"   # ajuste ao local da extração
gcc --version      # ... built by Brecht Sanders, r3) 16.2.0
```

3. Na raiz do repositório, crie o ambiente Python (a pasta `.venv/` já está no `.gitignore`):

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r tools/requirements.txt
```

## Compilação do firmware

1. Compile os 5 ambientes; a primeira execução baixa plataforma, compiladores e bibliotecas:

```powershell
cd firmware
pio run
```

*Tabela 1. Ambientes do firmware definidos em firmware/platformio.ini*

| Ambiente | Placa | Papel | Particularidade |
|---|---|---|---|
| `heltec_v3_node` | Heltec V3 (ESP32-S3 + SX1262) | nó | `NODE_ID=1`, UART1 para o computador de bordo |
| `heltec_v3_gateway` | Heltec V3 | gateway | JSON na serial e OLED |
| `heltec_v3_node_gps` | Heltec V3 + GPS | nó | `-DUSE_GPS` |
| `ttgo_v21_node` | TTGO LoRa32 v2.1 (ESP32 + SX1276) | nó | `NODE_ID=2` |
| `ttgo_v21_gateway` | TTGO LoRa32 v2.1 | gateway | placa alternativa |

2. Para um só ambiente, use `pio run -e heltec_v3_gateway`. Os padrões de rádio ficam nos `build_flags` de `[esp32_base]`: `-DLORA_AU915_CH=8` (916,8 MHz), `-DLORA_SF=9`, `-DLORA_BW_KHZ=125.0`, `-DLORA_TX_DBM=14` e `-DCERISE_NET_ID=42`.

Cada ambiente deve terminar em `SUCCESS`, com as imagens em `firmware/.pio/build/<ambiente>/`. Com outras versões de compilador ou bibliotecas, os binários não serão idênticos bit a bit aos de `firmware/binarios/`.

## Testes do protocolo

1. Com o GCC no PATH, rode em `firmware/` os 30 testes Unity da biblioteca `cerise_proto` (CRC-16, codificação dos nove tipos, erros de bit, ACK, duplicatas, tempo no ar, AU915 e JSON):

```powershell
pio test -e native -v
```

2. Na raiz, rode os 21 testes Python (validação cruzada C × Python, compatibilidade com a frente de drones e dashboard). O teste `test_compat_with_drones_front` precisa do repositório `cerise-2026-drones-segmentacao` clonado ao lado; sem ele, é pulado e os outros 20 continuam válidos:

```powershell
cd ..
$env:CERISE_DRONES_DIR = "..\cerise-2026-drones-segmentacao"
python -m pytest -v tools/tests
```

Com `gcc` no PATH, os testes recompilam `tools/c/gen_vectors.c` e regravam `tools/tests/vectors.tsv`; como o gerador é determinístico, `git status` não deve acusar mudança.

![Saída esperada dos testes nativos do protocolo (30 testes, 0 falhas)](evidencias/term_pio_test_native.png)

*Figura 1. Saída esperada dos testes nativos do protocolo (30 testes, 0 falhas)*

## Ferramentas de análise

Por padrão, os scripts regravam `resultados/` e `evidencias/` (na máquina usada, com conteúdo idêntico); confira com `git status` e desfaça com `git restore`. `lora_calc.py` e `sim_canal.py` aceitam `--out-res` e `--out-fig`.

1. Calcule tempo no ar por mensagem e SF, maior quadro dentro de 400 ms, sensibilidade e alcance (Friis, log-distância e Okumura–Hata). O padrão é 14 dBm com o drone a 60 m; a segunda linha simula outro cenário fora do repositório:

```powershell
python tools/lora_calc.py
python tools/lora_calc.py --ptx 20 --alt 120 --out-res ..\lora_saida --out-fig ..\lora_saida
```

2. Rode a simulação de N drones no mesmo canal (ALOHA, ALOHA + ACK, CSMA/CAD + ACK, slotted ALOHA e TDMA + ACK). A versão completa (1 130 execuções) leva cerca de 5 min; `--quick` levou 15 s, com números um pouco diferentes dos CSV versionados; `--plot-only` só refaz as figuras:

```powershell
python tools/sim_canal.py --quick --out-res ..\lora_saida --out-fig ..\lora_saida
python tools/sim_canal.py
python tools/sim_canal.py --plot-only
python tools/fig_protocolo.py
```

## Dashboard com fluxo serial simulado

O `tools/sim_serial_gateway.py` emula a saída serial do gateway (as mesmas linhas JSON do firmware, com codificação real dos quadros e canal sintético) num servidor TCP preso ao loopback, e o `tools/gateway_dashboard.py` lê esse fluxo com o mesmo código que lê uma porta COM.

1. No primeiro terminal, inicie o gateway simulado (3 drones, 600 s simulados, 10 vezes acelerados); ele imprime `aguardando conexao`:

```powershell
python tools/sim_serial_gateway.py --port 8195 --speed 10 --duration 600 --drones 3
```

2. No segundo terminal, com o mesmo ambiente Python, capture por 75 s. A pasta recebe `brutos.jsonl`, `pacotes.csv`, `eventos.csv`, `resumo.json` e `dashboard.png`; a segunda linha reprocessa a captura versionada, para conferência imediata:

```powershell
python tools/gateway_dashboard.py --port socket://127.0.0.1:8195 --duration 75 --out ..\lora_saida\dash_sim "--gw-pos=-16.6799,-49.2550"
python tools/gateway_dashboard.py --replay resultados/dash_sim/brutos.jsonl --out ..\lora_saida\replay
```

![Painel esperado do dashboard com o fluxo simulado: RSSI, SNR, PDR por drone e trajetórias](evidencias/fig_dashboard_sim.png)

*Figura 2. Painel esperado do dashboard com o fluxo simulado: RSSI, SNR, PDR por drone e trajetórias*

## Gabinete e documentos de hardware

1. Meça a placa com paquímetro: o gabinete supõe 50,2 × 25,5 mm (`--pcb-l`, `--pcb-w`) e 8,6 mm de componentes (`--comp-h`), valores não conferidos numa placa real. Gere base e tampa com as medidas obtidas:

```powershell
python hardware/gabinete/gabinete.py --pcb-l 50.2 --pcb-w 25.5 --comp-h 8.6
```

As saídas são `hardware/gabinete/gabinete_base.stl`, `hardware/gabinete/gabinete_tampa.stl`, `resultados/gabinete.json` e `evidencias/fig_gabinete.png`. Imprima em PETG (cerca de 15 g de filamento) e fixe com parafusos M3x8 nas quatro orelhas (padrão de 30,5 mm). O repositório não define parâmetros de fatiamento; os 12,9 g estimados supõem preenchimento de 100 %.

2. Regere o diagrama de ligação e a estimativa de consumo (valores de datasheet, a substituir por medição):

```powershell
python hardware/diagrama_ligacao.py
python hardware/orcamento_energia.py
```

## Montagem da bancada

Esta etapa e as duas seguintes nunca foram executadas. Adquira os itens de `hardware/BOM.csv`, nas versões de 915 MHz.

*Tabela 2. Lista de materiais da bancada (extrato de hardware/BOM.csv)*

| Qtd. | Item | Uso |
|---|---|---|
| 2 (+1 reserva) | Heltec WiFi LoRa 32 V3, versão 863–928 MHz | nó e gateway |
| 1 | Antena omnidirecional 915 MHz, 3 dBi, SMA | gateway, em mastro de 2 a 3 m |
| 1 | Antena dipolo 915 MHz, 2 dBi, SMA | nó |
| 2 | Pigtail U.FL para SMA fêmea, 10 a 15 cm | placa até a antena |
| 2 | LiPo 1S 3,7 V, 1000 a 2000 mAh, JST 1,25 mm | nó sem USB |
| 2 | Cabo USB-C de dados | gravação e serial |
| 1 | GPS NEO-6M ou NEO-M8N, 3,3 V (opcional) | posição dos pacotes |
| 1 | Medidor de energia USB (opcional) | consumo real |

Ligue conforme a parte (a) da figura:

1. Gateway: pigtail no U.FL, antena de 3 dBi vertical e USB-C ao notebook (alimentação e serial a 115200 bps).

2. Nó: dipolo de 2 dBi vertical e alimentação por USB-C ou LiPo 1S; confira a polaridade do JST antes de ligar.

3. GPS, só com `heltec_v3_node_gps`: TX do GPS no GPIO47, RX no GPIO48, 3V3 e GND. No `heltec_v3_node`, esses pinos são a UART1 (115200 8N1) do computador de bordo.

4. Nunca energize uma placa sem antena: o amplificador do SX1262 pode ser danificado.

![Diagrama de ligação: (a) bancada com duas placas Heltec V3 e notebook de solo; (b) módulo definitivo no drone](evidencias/fig_diagrama_ligacao.png)

*Figura 3. Diagrama de ligação: (a) bancada com duas placas Heltec V3 e notebook de solo; (b) módulo definitivo no drone*

## Gravação do firmware

1. Identifique as portas seriais (Gerenciador de Dispositivos; no Linux, `/dev/ttyUSB0` ou semelhante; as portas abaixo são exemplos) e, em `firmware/`, grave o gateway na placa 1 e o nó na placa 2:

```powershell
pio run -e heltec_v3_gateway -t upload --upload-port COM6
pio run -e heltec_v3_node    -t upload --upload-port COM7
```

2. Abra o monitor do gateway: a linha `{"evt":"boot",...}` deve ter `"rc":0`, `"freq_mhz":916.8` e `"sf":9`. Digite `HELP` e `STATS` e saia com Ctrl+C; repita no nó (`-e heltec_v3_node --port COM7`) com `STATUS`:

```powershell
pio device monitor -e heltec_v3_gateway --port COM6
```

3. Para gravar sem recompilar, siga `firmware/binarios/README.md`: use o esptool (pacote `esptool` do pip) com os endereços 0x0 (Heltec V3) ou 0x1000 (TTGO) para o bootloader, 0x8000, 0xe000 e 0x10000. O `boot_app0.bin` não é versionado; copie-o de `%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\` para `firmware/binarios/` e execute ali:

```powershell
esptool.py --chip esp32s3 --port COM6 --baud 921600 write_flash -z `
  0x0 heltec_v3_gateway\bootloader.bin 0x8000 heltec_v3_gateway\partitions.bin `
  0xe000 boot_app0.bin 0x10000 heltec_v3_gateway\firmware.bin
```

## Ensaio de alcance e PDR

O procedimento completo está em `hardware/protocolo_teste_alcance.md`. Faça primeiro as conferências de bancada, com as placas a cerca de 2 m e `POWER 2` nas duas:

1. Ambas iniciam com `"rc":0` e com as mesmas `freq_mhz` e `sf`.

2. O gateway imprime `POS` e `HEARTBEAT` com `"dup":false`, e o nó imprime `{"evt":"ack",...}`.

3. No gateway, `ASSIGN 1 3 7 -16.681 -49.2565 -16.679 -49.2535 40 25` faz o nó imprimir `{"evt":"dl",...}` e reportar a área 3.

4. Com a antena do gateway desconectada por 20 s (nunca a do transmissor ligado), o nó registra `timeout` e `drop`; ao reconectar, a entrega volta.

*Tabela 3. Matriz do ensaio (hardware/protocolo_teste_alcance.md, seção 4)*

| Variável | Valores |
|---|---|
| Distância (visada direta) | 50, 100, 200, 500, 1000, 2000 e 3000 m e o máximo alcançável |
| SF | 7, 9 e 10 (SF11 e SF12 só como referência, com `DWELL 0`) |
| Potência | 14 dBm; 20 dBm só no ponto mais distante |
| Quadro | POS (24 B); no ponto mais distante, também AREA_ASSIGN (32 B) |
| Altura do nó | 1,5 m em tripé; com autorização, drone a 30, 60 e 100 m |
| Repetições | 200 quadros por ponto, a cada 1 s |

Em cada ponto:

1. Posicione o nó, anote coordenadas e distância e ajuste o mesmo SF nas duas placas pelo monitor (`SF 9`). Feche o monitor do gateway, pois a porta não pode ser aberta por dois programas.

2. Inicie a captura rotulada no notebook:

```powershell
python tools/gateway_dashboard.py --port COM6 --label "d500_sf9_h1.5" --out resultados/campo/d500_sf9
```

3. Num terminal serial ligado ao nó (por exemplo, outro notebook com `pio device monitor`), envie `RANGETEST 200 1000`: 200 quadros POS sem ACK nem retentativa, que medem a PDR bruta do enlace.

4. Após `{"evt":"rangetest_end",...}`, encerre o dashboard com Ctrl+C; o `resumo.json` traz PDR, RSSI e SNR médios e mínimos.

5. Repita por 5 min com a telemetria normal, com ACK, para medir a PDR após retentativas e a latência (`lat_ms` nos eventos `ack`).

6. Registre clima, obstáculos, orientação das antenas e altura na planilha da seção 7 do protocolo; para o consumo, siga a seção 6.

Regulamentação: o rádio opera em 915–928 MHz (Resolução ANATEL nº 680/2017, Ato nº 14.448/2017), plano AU915, canal 8, com no máximo 400 ms no ar por quadro: o POS cabe até SF10 (370,7 ms) e o AREA_ASSIGN até SF9 (246,8 ms). Use até 1 W e antenas de até 6 dBi, só em ensaios curtos; a operação definitiva exige salto de frequência e homologação. Voe só com drone cadastrado, em linha de visada, até 120 m (RBAC-E nº 94 e ICA 100-40).

## Verificação do resultado

As etapas de software devem reproduzir a tabela abaixo; o dashboard simulado foi reexecutado e reproduziu exatamente os números de `resultados/dash_sim/resumo.json`.

*Tabela 4. Valores de referência e arquivos de origem*

| Verificação | Valor esperado | Arquivo |
|---|---|---|
| Compilação | 5 `SUCCESS`; `heltec_v3_node` com 22 400 B de RAM e 376 241 B de Flash | `resultados/build_all.log` |
| Testes nativos | 30 testes, 0 falhas | `resultados/pio_test_native.txt` |
| Testes Python | 21 aprovados (sem o repositório de drones, 20 e 1 pulado) | `resultados/pytest_tools.txt` |
| Tempo no ar do POS | 61,7 / 205,8 / 370,7 ms em SF7 / SF9 / SF10 | `resultados/airtime_mensagens.csv` |
| Alcance SF9, drone a 60 m | 5,99 km (Hata suburbano), 3,00 km (urbano) | `resultados/enlace_alcance.csv` |
| PDR simulada, 10 drones, SF9 | ALOHA 46,0 %; CSMA/CAD + ACK 62,6 %; TDMA + ACK 100 % | `resultados/sim_canal_vs_N.csv` |
| Simulador contra teoria | erro médio de 0,56 p.p. | `resultados/sim_canal_validacao_aloha.csv` |
| Dashboard simulado | 593 linhas; PDR de 98,6 / 97,8 / 94,8 % | `resultados/dash_sim/resumo.json` |
| Gabinete | 55,0 × 44,3 × 17,2 mm; 12,9 g | `resultados/gabinete.json` |
| Consumo estimado | 0,23 a 0,34 W | `resultados/orcamento_energia.json` |

Para o ensaio físico não há valores medidos: compare com `resultados/enlace_alcance.csv` e com os requisitos R4 (alcance de 3 km ou mais com margem de 10 dB) e R5 (PDR de 95 % ou mais) de `hardware/requisitos_modulo_drone.md`. Com 200 quadros, o intervalo de 95 % tem meia-largura de cerca de 4,2 p.p. para PDR de 0,9.

## Problemas comuns e soluções

- **Testes nativos sem compilador no Windows:** instale o WinLibs, ponha `mingw64\bin` no PATH e abra um terminal novo.
- **Teste de compatibilidade pulado (SKIPPED):** defina `CERISE_DRONES_DIR` com a pasta do `cerise-2026-drones-segmentacao`.
- **Erro na posição do gateway:** o valor de `--gw-pos` começa com sinal de menos; use `"--gw-pos=-16.6799,-49.2550"`.
- **Dashboard simulado sem linhas:** inicie antes o `sim_serial_gateway.py`; ele aceita uma única conexão e termina ao fim da duração.
- **Porta serial ocupada:** monitor e dashboard não abrem a mesma porta ao mesmo tempo.
- **SF ou potência voltam ao padrão:** `SF` e `POWER` não são persistentes; se a linha `boot` reaparecer, reenvie-os nas duas placas ou altere `-DLORA_SF` e regrave.
- **Quadro bloqueado pelo limite de 400 ms:** o evento `dwell_block` é esperado em SF11 e SF12 e para AREA_ASSIGN em SF10 (452,6 ms).
- **OLED com "ERRO no radio":** o ambiente gravado não corresponde à placa (Heltec V3 com SX1262, TTGO com SX1276).
- **Tensão de bateria errada na Heltec V3.2:** `firmware/src/common/board.h` usa `VBAT_CTRL_ACTIVE LOW` (V3.0 e V3.1); na V3.2 é HIGH.
- **Ativação do Python bloqueada no PowerShell:** chame `.\.venv\Scripts\python.exe` diretamente, sem alterar a política de execução.

## Referências do repositório

- [`README.md`](README.md), [`RELATORIO_SECAO.md`](RELATORIO_SECAO.md) e [`RELEASES.md`](RELEASES.md): visão geral, texto técnico e Release.
- [`firmware/platformio.ini`](firmware/platformio.ini), [`firmware/lib/cerise_proto/src/cerise_proto.h`](firmware/lib/cerise_proto/src/cerise_proto.h), [`firmware/src/node_main.cpp`](firmware/src/node_main.cpp) e [`firmware/src/gateway_main.cpp`](firmware/src/gateway_main.cpp): ambientes, protocolo e comandos seriais.
- [`firmware/test/test_proto/test_proto.c`](firmware/test/test_proto/test_proto.c), [`tools/tests/`](tools/tests) e [`firmware/binarios/README.md`](firmware/binarios/README.md): testes e gravação.
- [`hardware/BOM.csv`](hardware/BOM.csv), [`hardware/protocolo_teste_alcance.md`](hardware/protocolo_teste_alcance.md), [`hardware/requisitos_modulo_drone.md`](hardware/requisitos_modulo_drone.md) e [`hardware/gabinete/gabinete.py`](hardware/gabinete/gabinete.py): hardware, ensaio e gabinete.
- [`resultados/`](resultados) e [`evidencias/`](evidencias): números de referência e figuras.
