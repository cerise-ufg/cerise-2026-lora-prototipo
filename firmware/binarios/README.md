# Binários compilados do firmware (cerise_proto v1)

Imagens geradas por `pio run` (PlatformIO Core 6.1.19, plataforma `espressif32@6.12.0`, framework Arduino-ESP32
2.0.17 — pacote `framework-arduinoespressif32 3.20017.241212`, RadioLib 7.8.1, U8g2 2.36.18, TinyGPSPlus 1.1.0 no
ambiente `heltec_v3_node_gps`) em 26/10/2026, a partir do código deste repositório. O uso de RAM/Flash de cada
ambiente está em `resultados/build_all.log` e `resultados/build_all_resumo.txt`.

| Pasta | Placa / papel | MCU | Endereço do bootloader |
|---|---|---|---|
| `heltec_v3_node/` | Heltec WiFi LoRa 32 V3, nó (drone), `NODE_ID=1` | ESP32-S3 | `0x0` |
| `heltec_v3_gateway/` | Heltec WiFi LoRa 32 V3, gateway (solo) | ESP32-S3 | `0x0` |
| `heltec_v3_node_gps/` | Heltec V3, nó com GPS serial (`-DUSE_GPS`) | ESP32-S3 | `0x0` |
| `ttgo_v21_node/` | TTGO LoRa32 v2.1, nó (drone), `NODE_ID=2` | ESP32 | `0x1000` |
| `ttgo_v21_gateway/` | TTGO LoRa32 v2.1, gateway (solo) | ESP32 | `0x1000` |

Cada pasta contém `bootloader.bin`, `partitions.bin` e `firmware.bin` (aplicação). As somas SHA-256 estão em
`SHA256SUMS.txt`. Os arquivos de depuração (`firmware.elf`, `firmware.map`) e o executável dos testes nativos
ficam no Release (ver `RELEASES.md` na raiz).

## Gravação

Forma recomendada (recompila e grava, a partir de `firmware/`):

```powershell
pio run -e heltec_v3_gateway -t upload --upload-port COM6
pio run -e heltec_v3_node    -t upload --upload-port COM7
```

Gravação direta destes binários com o `esptool` (sem recompilar). O arquivo `boot_app0.bin` faz parte do
framework Arduino-ESP32 (Espressif) e não é versionado aqui; ele fica em
`%USERPROFILE%\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin` após a primeira
compilação, ou em `tools/partitions/boot_app0.bin` do repositório `espressif/arduino-esp32` (tag `2.0.17`).

```powershell
# Heltec V3 (ESP32-S3)
esptool.py --chip esp32s3 --port COM6 --baud 921600 write_flash -z `
  0x0 heltec_v3_gateway\bootloader.bin 0x8000 heltec_v3_gateway\partitions.bin `
  0xe000 boot_app0.bin 0x10000 heltec_v3_gateway\firmware.bin

# TTGO LoRa32 v2.1 (ESP32)
esptool.py --chip esp32 --port COM6 --baud 921600 write_flash -z `
  0x1000 ttgo_v21_gateway\bootloader.bin 0x8000 ttgo_v21_gateway\partitions.bin `
  0xe000 boot_app0.bin 0x10000 ttgo_v21_gateway\firmware.bin
```

Os endereços (`0x0`/`0x1000`, `0x8000`, `0xe000`, `0x10000`) são os usados pelo PlatformIO para o framework
Arduino nessas placas (`FLASH_EXTRA_IMAGES` em `framework-arduinoespressif32/tools/platformio-build.py`).

Observação: os binários não foram gravados em placa física até o momento (os módulos ainda não foram adquiridos);
eles foram verificados apenas por compilação.
