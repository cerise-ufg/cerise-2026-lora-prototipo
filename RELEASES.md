# Artefatos publicados como Release

Os arquivos abaixo não são versionados no git (são saídas de compilação grandes, regeneráveis com `pio run`),
mas ficam anexados ao Release **`v1.0-prototipo`** deste repositório no GitHub para referência e depuração.

| Arquivo | Tamanho | SHA-256 | Conteúdo |
|---|---:|---|---|
| `lora-firmware-v1-elf-map.zip` | 21 677 374 B (20,7 MiB) | `2148810e3fe4d4a3e2e1458f68d98f5c1ba44dbb7bc376208c413578c45a7e17` | Saídas completas da compilação de 26/10/2026 para os 5 ambientes (`heltec_v3_node`, `heltec_v3_gateway`, `heltec_v3_node_gps`, `ttgo_v21_node`, `ttgo_v21_gateway`): `firmware.elf` (com símbolos de depuração, ~9,2–10,6 MB cada), `firmware.map` (~11,9–12,6 MB cada), `firmware.bin`, `bootloader.bin`, `partitions.bin`; e `native/program.exe` (executável Windows dos 30 testes Unity nativos). Inclui `SHA256SUMS.txt` interno com a soma de cada arquivo. |

Os `.bin` graváveis (bootloader, tabela de partições e aplicação) também estão no próprio repositório, em
`firmware/binarios/`, com as mesmas somas SHA-256.

## Como baixar e verificar

```powershell
# com o GitHub CLI (substitua cerise-ufg pela organização do repositório)
gh release download v1.0-prototipo -R cerise-ufg/cerise-2026-lora-prototipo -p "lora-firmware-v1-elf-map.zip"

# conferir a integridade
(Get-FileHash .\lora-firmware-v1-elf-map.zip -Algorithm SHA256).Hash.ToLower()
# deve ser 2148810e3fe4d4a3e2e1458f68d98f5c1ba44dbb7bc376208c413578c45a7e17

# extrair
Expand-Archive .\lora-firmware-v1-elf-map.zip -DestinationPath .\lora-firmware-v1
```

No Linux/macOS: `sha256sum lora-firmware-v1-elf-map.zip` e `unzip lora-firmware-v1-elf-map.zip -d lora-firmware-v1`.
Não há partes divididas (o arquivo tem menos de 1,9 GB).

## Uso dos arquivos `.elf`

O `.elf` permite decodificar *backtraces* de exceção do ESP32 (por exemplo com o filtro `esp32_exception_decoder`
do monitor do PlatformIO ou com `xtensa-esp32s3-elf-addr2line -pfiaC -e firmware.elf <endereços>`) sem precisar
recompilar exatamente a mesma versão. O `.map` mostra a ocupação de RAM/Flash por símbolo.

## Como regenerar

```powershell
cd firmware
pio run      # gera .pio/build/<ambiente>/{firmware.elf,firmware.map,firmware.bin,bootloader.bin,partitions.bin}
pio test -e native -v      # gera .pio/build/native/program.exe e executa os 30 testes
```

Versões usadas: PlatformIO Core 6.1.19, `espressif32@6.12.0` (Arduino-ESP32 2.0.17), RadioLib 7.8.1,
U8g2 2.36.18, TinyGPSPlus 1.1.0, Unity 2.6.1 e GCC 16.2 (WinLibs, UCRT) para os testes nativos.
Compilações com outras versões de compilador/biblioteca não reproduzem bit a bit os mesmos binários.
