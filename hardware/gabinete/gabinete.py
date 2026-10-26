"""Gabinete parametrico do modulo LoRa (Heltec WiFi LoRa 32 V3) para fixacao no drone.

Gera base + tampa em STL (trimesh + manifold3d), calcula volume/massa estimada e
renderiza uma vista isometrica explodida (matplotlib).

  python hardware/gabinete/gabinete.py            # usa os parametros abaixo
  python hardware/gabinete/gabinete.py --wall 2.0 # exemplo de variacao

Saidas: hardware/gabinete/gabinete_base.stl, gabinete_tampa.stl,
        resultados/gabinete.json, evidencias/fig_gabinete.png
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / "tools"))

ENGINE = "manifold"


def box(lx, ly, lz, cx=0.0, cy=0.0, z0=0.0):
    m = trimesh.creation.box(extents=(lx, ly, lz))
    m.apply_translation((cx, cy, z0 + lz / 2))
    return m


def cyl(r, h, cx, cy, z0, axis="z", sections=48):
    m = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    if axis == "x":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (1, 0, 0)))
    m.apply_translation((cx, cy, z0))
    return m


def diff(a, *bs):
    return trimesh.boolean.difference([a, *bs], engine=ENGINE)


def union(*ms):
    return trimesh.boolean.union(list(ms), engine=ENGINE)


def build(p):
    L = p.pcb_l + 2 * p.clear
    W = p.pcb_w + 2 * p.clear
    H = p.under + p.pcb_t + p.comp_h + p.top_gap
    Lo, Wo = L + 2 * p.wall, W + 2 * p.wall
    Ho = H + p.floor

    # ---- base: caixa aberta
    base = diff(box(Lo, Wo, Ho), box(L, W, H + 1, z0=p.floor))
    # trilhos de apoio da placa (ao longo do comprimento)
    z_pcb = p.floor + p.under
    rails = [box(L, 1.4, p.under, cy=s * (W / 2 - 0.7), z0=p.floor) for s in (-1, 1)]
    base = union(base, *rails)
    # orelhas de fixacao M3 (padrao 30,5 mm em X) nos dois lados longos
    ear_w = 7.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * 15.25, sy * (Wo / 2 + ear_w / 2 - 0.01)
            ear = diff(box(9.0, ear_w, 3.0, cx=cx, cy=cy), cyl(1.7, 10, cx, cy, 0))
            base = union(base, ear)
    # recortes: USB-C (-X), passa-cabo JST-GH 5V/UART (-X, abaixo), SMA (+X)
    # (as orelhas aceitam parafuso M3 ou abracadeira de 2,5 mm)
    z_usb = z_pcb + p.pcb_t + 1.8
    base = diff(base, box(p.wall * 3, 10.0, 4.6, cx=-Lo / 2, z0=z_usb - 2.3))
    base = diff(base, cyl(2.6, p.wall * 3, -Lo / 2, -6.0, p.floor + 1.0 + 2.6, axis="x"))
    base = diff(base, cyl(p.sma_d / 2, p.wall * 3, Lo / 2, 0.0, Ho * 0.55, axis="x"))

    # ---- tampa com ressalto e janela do OLED
    lid = box(Lo, Wo, p.lid_t)
    lip = diff(box(L - 2 * p.fit, W - 2 * p.fit, 2.0, z0=-2.0),
               box(L - 2 * p.fit - 2.4, W - 2 * p.fit - 2.4, 2.2, z0=-2.1))
    lid = union(lid, lip)
    lid = diff(lid, box(p.oled_w, p.oled_h, p.lid_t * 4, cx=p.oled_dx, cy=0.0, z0=-p.lid_t))
    # furos de ventilacao (dissipacao do PA durante TX)
    for k in range(5):
        lid = diff(lid, box(1.2, 8.0, p.lid_t * 4, cx=-L / 2 + 4 + 1.8 * k, cy=0, z0=-p.lid_t))
    dims = {"interno_mm": [round(L, 2), round(W, 2), round(H, 2)],
            "externo_mm": [round(Lo, 2), round(Wo + 2 * ear_w, 2), round(Ho + p.lid_t, 2)]}
    return base, lid, dims


def render(base, lid, path, Ho):
    import plot_style as ps
    ps.apply()
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(11.5, 5.4))
    light = np.array([0.4, -0.5, 0.9])
    light /= np.linalg.norm(light)
    for k, (m, col, dz) in enumerate(((base, np.array([0.17, 0.47, 0.84]), 0.0),
                                      (lid, np.array([0.92, 0.41, 0.20]), Ho + 14.0))):
        ax = fig.add_subplot(1, 2, 1, projection="3d") if k == 0 else ax
        v = m.vertices.copy()
        v[:, 2] += dz
        tri = v[m.faces]
        shade = 0.45 + 0.55 * np.clip(m.face_normals @ light, 0, 1)
        fc = np.clip(col[None, :] * shade[:, None], 0, 1)
        pc = Poly3DCollection(tri, facecolors=fc, edgecolors=fc, linewidths=0.05)
        ax.add_collection3d(pc)
    allv = np.vstack([base.vertices, lid.vertices + [0, 0, Ho + 14.0]])
    mn, mx = allv.min(0), allv.max(0)
    c = (mn + mx) / 2
    r = (mx - mn).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r * 0.6, c[2] + r * 0.6)
    ax.set_box_aspect((1, 1, 0.6))
    ax.view_init(elev=28, azim=-58)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("")
    ax.set_title("Vista explodida (base + tampa), eixos em mm")
    # vista de cima da base
    ax2 = fig.add_subplot(1, 2, 2)
    fig.subplots_adjust(wspace=0.35)
    def planar(sec, z):
        t = np.eye(4)
        t[2, 3] = -z  # mantem as coordenadas x, y originais
        return sec.to_2D(to_2D=t)

    sec = base.section(plane_origin=[0, 0, 2.0], plane_normal=[0, 0, 1])
    if sec is not None:
        p2, _ = planar(sec, 2.0)
        for e in p2.discrete:
            ax2.plot(e[:, 0], e[:, 1], color=ps.CAT[0], lw=1.2)
    sec2 = base.section(plane_origin=[0, 0, Ho - 1.0], plane_normal=[0, 0, 1])
    if sec2 is not None:
        p3, _ = planar(sec2, Ho - 1.0)
        for e in p3.discrete:
            ax2.plot(e[:, 0], e[:, 1], color=ps.INK2, lw=1.0, ls="--")
    ax2.set_aspect("equal")
    ax2.set_title("Cortes da base: z = 2 mm (azul) e topo (tracejado)")
    ax2.set_xlabel("x (mm)")
    ax2.set_ylabel("y (mm)")
    ps.save(fig, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcb-l", dest="pcb_l", type=float, default=50.2)   # Heltec V3: 50,2 x 25,5 mm
    ap.add_argument("--pcb-w", dest="pcb_w", type=float, default=25.5)
    ap.add_argument("--pcb-t", dest="pcb_t", type=float, default=1.6)
    ap.add_argument("--comp-h", dest="comp_h", type=float, default=8.6)  # componentes/OLED acima da placa
    ap.add_argument("--under", type=float, default=3.0)                  # espaco sob a placa (pinos/solda)
    ap.add_argument("--top-gap", dest="top_gap", type=float, default=0.8)
    ap.add_argument("--clear", type=float, default=0.6)
    ap.add_argument("--wall", type=float, default=1.8)
    ap.add_argument("--floor", type=float, default=1.6)
    ap.add_argument("--lid-t", dest="lid_t", type=float, default=1.6)
    ap.add_argument("--fit", type=float, default=0.25)
    ap.add_argument("--sma-d", dest="sma_d", type=float, default=6.6)
    ap.add_argument("--oled-w", dest="oled_w", type=float, default=24.0)
    ap.add_argument("--oled-h", dest="oled_h", type=float, default=13.0)
    ap.add_argument("--oled-dx", dest="oled_dx", type=float, default=4.0)
    ap.add_argument("--density", type=float, default=1.27, help="g/cm3 (PETG)")
    p = ap.parse_args()
    base, lid, dims = build(p)
    base.export(HERE / "gabinete_base.stl")
    lid.export(HERE / "gabinete_tampa.stl")
    Ho = p.under + p.pcb_t + p.comp_h + p.top_gap + p.floor
    info = {
        "parametros": vars(p), **dims,
        "base": {"volume_cm3": round(base.volume / 1000, 3), "estanque": bool(base.is_watertight),
                 "faces": int(len(base.faces))},
        "tampa": {"volume_cm3": round(lid.volume / 1000, 3), "estanque": bool(lid.is_watertight),
                  "faces": int(len(lid.faces))},
    }
    info["massa_estimada_g_100pct_preenchimento"] = round((base.volume + lid.volume) / 1000 * p.density, 1)
    (ROOT / "resultados").mkdir(exist_ok=True)
    (ROOT / "resultados" / "gabinete.json").write_text(json.dumps(info, indent=2, ensure_ascii=False),
                                                      encoding="utf-8")
    print(json.dumps(info, indent=1, ensure_ascii=False))
    render(base, lid, ROOT / "evidencias" / "fig_gabinete.png", Ho)


if __name__ == "__main__":
    main()
