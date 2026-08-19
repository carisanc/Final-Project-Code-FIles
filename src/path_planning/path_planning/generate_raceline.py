#!/usr/bin/env python3

# ============================================================
# generate_raceline.py — CLI offline: mapa → raceline CSV.
#
# Orquesta el pipeline completo:
#   mapa (.yaml) → centerline + anchos → QP mínima curvatura →
#   perfil de velocidad → CSV (x, y, heading, kappa, v) + PNG debug.
#
# Uso:
#   ros2 run path_planning generate_raceline \
#       --map /ruta/al/mapa.yaml \
#       --params /ruta/raceline_params.yaml \
#       --out /ruta/salida.csv \
#       [--debug-png /ruta/debug.png]
#
# Imprime métricas de verificación (lazo cerrado, κ_max, clearance
# mínimo, tiempo de vuelta estimado) y falla con error claro si la
# raceline viola el margen de seguridad.
# ============================================================

import argparse
import sys

import numpy as np
import yaml
from scipy import ndimage

from path_planning.map_utils import load_map
from path_planning.centerline import (
    extract_centerline, clean_free_mask, corridor_from_seed, track_widths)
from path_planning.min_curvature import optimize_raceline, curvature_of
from path_planning.velocity_profile import velocity_profile, lap_time_estimate


def clearance_along(points, map_data, free):
    """Distancia real de cada punto a la pared más cercana (m), vía EDT."""
    edt = ndimage.distance_transform_edt(free) * map_data.resolution
    rows, cols = map_data.world_to_px(points[:, 0], points[:, 1])
    r = np.clip(np.round(rows).astype(int), 0, map_data.height - 1)
    c = np.clip(np.round(cols).astype(int), 0, map_data.width - 1)
    return edt[r, c]


def _arc_length(points):
    """Distancia acumulada s_i (m) a lo largo del lazo cerrado, s_0 = 0."""
    d = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    return np.concatenate(([0.0], np.cumsum(d)[:-1])), d.sum()


def save_debug_png(path, map_data, center, race, v):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax_map, ax_v) = plt.subplots(
        1, 2, figsize=(20, 10 * map_data.height / map_data.width),
        gridspec_kw={'width_ratios': [1.3, 1.0]})

    # --- Panel izquierdo: vista de arriba, mapa + centerline + raceline ---
    extent = [map_data.origin[0],
              map_data.origin[0] + map_data.width * map_data.resolution,
              map_data.origin[1],
              map_data.origin[1] + map_data.height * map_data.resolution]
    ax_map.imshow(map_data.img, cmap='gray', extent=extent, origin='upper')
    ax_map.plot(np.append(center[:, 0], center[0, 0]),
                np.append(center[:, 1], center[0, 1]),
                'b--', lw=1.0, label='centerline')
    sc = ax_map.scatter(race[:, 0], race[:, 1], c=v, cmap='plasma', s=6,
                        label='raceline (color = v)')
    fig.colorbar(sc, ax=ax_map, label='v [m/s]', shrink=0.7)
    ax_map.legend(loc='upper right')
    ax_map.set_xlabel('x [m]'); ax_map.set_ylabel('y [m]')
    ax_map.set_title('Raceline de mínima curvatura')

    # --- Panel derecho: perfil de velocidad vs distancia recorrida ---
    s, total_len = _arc_length(race)
    s_closed = np.append(s, total_len)
    v_closed = np.append(v, v[0])

    ax_v.plot(s_closed, v_closed, color='tab:orange', lw=1.5)
    ax_v.scatter(s, v, c=v, cmap='plasma', s=8)
    i_min, i_max = int(np.argmin(v)), int(np.argmax(v))
    ax_v.annotate(f"v_min={v[i_min]:.2f} m/s\n@s={s[i_min]:.0f} m",
                 xy=(s[i_min], v[i_min]), xytext=(10, -25),
                 textcoords='offset points',
                 arrowprops=dict(arrowstyle='->', color='gray'))
    ax_v.annotate(f"v_max={v[i_max]:.2f} m/s\n@s={s[i_max]:.0f} m",
                 xy=(s[i_max], v[i_max]), xytext=(10, 15),
                 textcoords='offset points',
                 arrowprops=dict(arrowstyle='->', color='gray'))
    ax_v.axhline(v.mean(), color='gray', ls=':', lw=1,
                label=f'v media = {v.mean():.2f} m/s')
    ax_v.set_xlabel('distancia recorrida s [m]')
    ax_v.set_ylabel('v [m/s]')
    ax_v.set_title('Perfil de velocidad vs distancia')
    ax_v.legend(loc='lower right')
    ax_v.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description='Genera la raceline de mínima curvatura desde un mapa.')
    p.add_argument('--map', required=True, help='Ruta al .yaml del mapa (formato map_server)')
    p.add_argument('--params', required=True, help='Ruta a raceline_params.yaml')
    p.add_argument('--out', required=True, help='CSV de salida')
    p.add_argument('--debug-png', default=None, help='PNG de depuración (mapa + líneas)')
    p.add_argument('--corridor-seed', default=None, metavar='X,Y',
                   help='Punto (x,y en metros) sobre la pista para aislar el '
                        'corredor por flood-fill. Úsalo con mapas ground-truth '
                        'binarios (p. ej. SaoPaulo_map, semilla 0,0). Si se '
                        'omite, se limpia la máscara con clean_free_mask '
                        '(mapas SLAM de 3 estados).')
    p.add_argument('--objective', default='curvature',
                   choices=['curvature', 'time'],
                   help="'curvature' = QP de mínima curvatura (default, la "
                        "validada); 'time' = refina esa solución minimizando "
                        "el TIEMPO DE VUELTA directamente (L-BFGS-B con el "
                        "perfil de velocidad como objetivo; ver min_time.py).")
    args = p.parse_args(argv)

    seed = None
    if args.corridor_seed is not None:
        try:
            sx, sy = (float(t) for t in args.corridor_seed.split(','))
            seed = (sx, sy)
        except ValueError:
            print(f"ERROR: --corridor-seed inválido: '{args.corridor_seed}' "
                  "(esperado 'X,Y', p. ej. '0,0').")
            return 1

    with open(args.params) as f:
        prm = yaml.safe_load(f)

    print(f"[1/4] Cargando mapa: {args.map}")
    m = load_map(args.map)

    print("[2/4] Extrayendo centerline (skeletonización)...")
    if seed is not None:
        print(f"      modo ground-truth: aíslo el corredor por flood-fill desde {seed}")
    center, tang, w_left, w_right, length = extract_centerline(
        m, n_points=prm['n_points'], spline_smoothing=prm['spline_smoothing'],
        corridor_seed=seed)
    k_center = curvature_of(center)
    print(f"      {len(center)} pts, longitud={length:.1f} m, "
          f"ancho medio={np.mean(w_left + w_right):.2f} m")

    print("[3/4] Optimizando mínima curvatura (QP)...")
    race, alpha, k_race = optimize_raceline(
        center, w_left, w_right,
        car_width=prm['car_width'], safety_margin=prm['safety_margin'],
        smoothing_lambda=prm['smoothing_lambda'],
        iterations=prm['qp_iterations'])

    if args.objective == 'time':
        print("[3b/4] Refinando a MÍNIMO TIEMPO (L-BFGS-B sobre T(α))...")
        from path_planning.min_time import optimize_min_time
        phys = {k: prm[k] for k in
                ('a_lat_max', 'a_accel', 'a_brake', 'v_max', 'v_min')}
        race, alpha, k_race, v = optimize_min_time(
            center, w_left, w_right,
            car_width=prm['car_width'], safety_margin=prm['safety_margin'],
            alpha0=alpha, phys=phys,
            smooth_lambda=prm.get('min_time_smooth_lambda', 2.0),
            maxiter=prm.get('min_time_maxiter', 60))

    print("[4/4] Perfil de velocidad...")
    v = velocity_profile(race, k_race,
                         a_lat_max=prm['a_lat_max'], a_accel=prm['a_accel'],
                         a_brake=prm['a_brake'], v_max=prm['v_max'],
                         v_min=prm['v_min'])

    # heading de la raceline
    fwd = np.roll(race, -1, axis=0) - np.roll(race, 1, axis=0)
    heading = np.arctan2(fwd[:, 1], fwd[:, 0])

    # ---- métricas de verificación ----
    # misma máscara que usó extract_centerline, para un clearance honesto
    if seed is not None:
        free = corridor_from_seed(m.free_mask(), m, seed)
    else:
        free = clean_free_mask(m.free_mask())
    clear = clearance_along(race, m, free)
    d_safe = prm['car_width'] / 2.0 + prm['safety_margin']
    # Espacio lateral POR LADO desde la raceline (head-to-head: el offset
    # máximo de adelantamiento en cada waypoint = d_lado − semi-ancho −
    # margen). Un solo d_wall no basta: hay que saber QUÉ lado tiene hueco.
    tang_race = np.stack([np.cos(heading), np.sin(heading)], axis=1)
    d_left, d_right = track_widths(race, tang_race, m, free)
    gap = float(np.linalg.norm(race[0] - race[-1]))
    ds = length / len(race)
    t_lap = lap_time_estimate(race, v)

    print("\n===== MÉTRICAS =====")
    print(f"lazo cerrado:        gap={gap:.3f} m (Δs={ds:.3f}) → {'OK' if gap < 2*ds else 'FALLO'}")
    print(f"κ_max centerline:    {np.abs(k_center).max():.3f} 1/m")
    print(f"κ_max raceline:      {np.abs(k_race).max():.3f} 1/m → "
          f"{'OK (menor)' if np.abs(k_race).max() < np.abs(k_center).max() else 'FALLO'}")
    print(f"clearance mínimo:    {clear.min():.3f} m (requerido ≥ {prm['car_width']/2:.3f} contacto, "
          f"objetivo ≥ {d_safe:.3f})")
    print(f"velocidad:           min={v.min():.2f} media={v.mean():.2f} max={v.max():.2f} m/s")
    print(f"espacio lateral:     d_left min={d_left.min():.2f} "
          f"d_right min={d_right.min():.2f} m (medias {d_left.mean():.2f}/"
          f"{d_right.mean():.2f})")
    print(f"tiempo de vuelta:    {t_lap:.1f} s estimado ({length:.0f} m)")

    ok = (gap < 2 * ds
          and np.abs(k_race).max() < np.abs(k_center).max()
          and clear.min() >= prm['car_width'] / 2.0)
    if not ok:
        print("\nERROR: la raceline no pasa las verificaciones — no se escribe el CSV.")
        return 1

    # d_left/d_right: columnas extra retro-compatibles (PP/MPC indexan las
    # primeras 5; el ADELANTAR del head-to-head usa las nuevas).
    header = 'x,y,heading,kappa,v,d_left,d_right'
    data = np.column_stack([race[:, 0], race[:, 1], heading, k_race, v,
                            d_left, d_right])
    np.savetxt(args.out, data, delimiter=',', header=header, comments='', fmt='%.5f')
    print(f"\nCSV escrito: {args.out} ({len(data)} waypoints)")

    if args.debug_png:
        save_debug_png(args.debug_png, m, center, race, v)
        print(f"PNG debug:   {args.debug_png}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
