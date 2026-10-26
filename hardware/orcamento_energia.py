"""Orcamento de energia (ESTIMATIVA de projeto) do modulo LoRa embarcado.

As correntes sao valores tipicos/hipoteses de projeto (datasheet SX1261/2 e ordem de grandeza do
ESP32-S3 com Wi-Fi/BLE desligados) e DEVEM ser substituidas por medicoes com o prototipo
(medidor USB ou shunt + osciloscopio) — ver hardware/protocolo_teste_alcance.md, secao 6.

Saida: resultados/orcamento_energia.json
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import cerise_proto as cp  # noqa: E402

V_SYS = 3.3
HIP = {  # mA
    "mcu_ativo_rf_off": 40.0,      # hipotese de projeto (ESP32-S3, CPU ativa, Wi-Fi/BLE desligados)
    "sx1262_rx": 4.6,              # datasheet SX1262, DC-DC, BW 125 kHz
    "sx1262_tx_14dBm": 45.0,       # datasheet SX1262 (+14 dBm, ajuste otimo do PA)
    "sx1262_tx_22dBm": 118.0,      # datasheet SX1262 (+22 dBm)
    "oled_ligado": 15.0,           # SSD1306 128x64, ~metade dos pixels (ordem de grandeza)
}
EFF_LDO_5V = 3.3 / 5.0  # regulador linear da placa alimentada por 5 V
P_DRONE_W = 200.0       # potencia media de voo de um multirrotor de pequeno porte (referencia)


def budget(sf, ptx_key, period_s=5.0, oled=False, ack=True):
    toa_up = cp.airtime_us(sf, payload=cp.frame_len(cp.POS)) / 1e6
    toa_ack = cp.airtime_us(sf, payload=cp.frame_len(cp.ACK)) / 1e6
    d_tx = toa_up / period_s
    d_rx = (toa_ack + 0.16) / period_s if ack else 0.0  # janela de espera do ACK
    i = HIP["mcu_ativo_rf_off"] + (HIP["oled_ligado"] if oled else 0)
    i += d_tx * HIP[ptx_key] + (1 - d_tx) * HIP["sx1262_rx"]  # radio fica em RX continuo (downlink)
    p33 = i * V_SYS / 1000
    p5 = p33 / EFF_LDO_5V
    return {"SF": sf, "tx": ptx_key, "oled": oled, "duty_tx_pct": round(100 * d_tx, 2),
            "duty_rx_ack_pct": round(100 * d_rx, 2), "i_media_mA": round(i, 1), "p_3v3_W": round(p33, 3),
            "p_entrada_5V_W": round(p5, 3), "fracao_potencia_drone_pct": round(100 * p5 / P_DRONE_W, 3),
            "energia_voo_20min_Wh": round(p5 / 3, 3)}


def main():
    rows = [budget(sf, k, oled=o) for sf in (7, 9, 10) for k in ("sx1262_tx_14dBm", "sx1262_tx_22dBm")
            for o in (False, True)]
    out = {"hipoteses_mA": HIP, "v_sistema": V_SYS, "eficiencia_ldo": EFF_LDO_5V, "p_drone_ref_W": P_DRONE_W,
           "periodo_POS_s": 5.0, "linhas": rows,
           "aviso": "estimativa com correntes tipicas/hipoteses; medir no prototipo"}
    (ROOT / "resultados").mkdir(exist_ok=True)
    (ROOT / "resultados" / "orcamento_energia.json").write_text(json.dumps(out, indent=2, ensure_ascii=False),
                                                               encoding="utf-8")
    print(f"{'SF':>3} {'TX':>16} {'OLED':>5} {'dutyTX%':>8} {'I med mA':>9} {'P 5V W':>7} {'% drone':>8}")
    for r in rows:
        print(f"{r['SF']:>3} {r['tx']:>16} {str(r['oled']):>5} {r['duty_tx_pct']:>8} {r['i_media_mA']:>9} "
              f"{r['p_entrada_5V_W']:>7} {r['fracao_potencia_drone_pct']:>8}")


if __name__ == "__main__":
    main()
