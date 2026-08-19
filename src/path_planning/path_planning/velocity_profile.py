#!/usr/bin/env python3

# ============================================================
# velocity_profile.py — perfil de velocidad físicamente alcanzable.
#
# 1. Límite de fricción por punto:  v_i = sqrt(a_lat_max / |κ_i|)
#    (recortado a [v_min, v_max]).
# 2. Pasada hacia ADELANTE: no se puede acelerar más que la aceleración
#    longitudinal DISPONIBLE según el círculo de fricción (ver abajo)
#       v_{i+1} ≤ sqrt(v_i² + 2·a_lon_disp·Δs_i)
# 3. Pasada hacia ATRÁS: ídem para la frenada
#       v_i ≤ sqrt(v_{i+1}² + 2·a_lon_disp·Δs_i)
#
# Círculo de fricción: el neumático reparte su agarre entre lateral y
# longitudinal. Si en el punto i ya se usa a_lat = v_i²·|κ_i|, lo que
# queda para acelerar/frenar es
#       a_lon_disp = a_lon_max · sqrt(max(0, 1 − (a_lat/a_lat_max)²))
# Sin esto, el perfil mandaba a_accel COMPLETA en plena salida de curva
# y Pure Pursuit (que lee v del waypoint más cercano, sin anticipación)
# aceleraba con el volante girado → se abría y rozaba la pared exterior
# (medido: choques en la salida (27,57) de SaoPaulo con speed_scale≥0.8).
# Con el círculo, la rampa de aceleración se corre sola hacia la recta.
#
# Las pasadas son cíclicas (el lazo se recorre 2 veces) para que la
# condición también se cierre a través del punto de partida.
# ============================================================

import numpy as np


def velocity_profile(points, kappa, a_lat_max, a_accel, a_brake,
                     v_max, v_min, eps=1e-4):
    """
    points: (n,2) raceline cerrada; kappa: (n,) curvatura con signo.
    Devuelve v (n,) en m/s.
    """
    n = len(points)
    ds = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)

    # 1) límite de fricción lateral
    v = np.sqrt(a_lat_max / np.maximum(np.abs(kappa), eps))
    v = np.clip(v, v_min, v_max)

    # Nota de implementación (7 jul): mismas fórmulas de siempre, pero con
    # math.sqrt y listas Python en el loop — np.sqrt escalar es ~30× más
    # lento y este perfil ahora es la función OBJETIVO del optimizador de
    # mínimo tiempo (miles de evaluaciones). Semántica idéntica (verificado
    # bit a bit contra la versión anterior).
    from math import sqrt as _sqrt

    vl = v.tolist()
    kl = kappa.tolist()
    dl = ds.tolist()

    def a_lon_disp(a_lon_max, v_i, kappa_i):
        # Círculo de fricción: agarre longitudinal restante tras la lateral.
        ratio = (v_i * v_i) * abs(kappa_i) / a_lat_max
        r = min(ratio, 1.0)
        return a_lon_max * _sqrt(max(0.0, 1.0 - r * r))

    # 2) pasada adelante (2 vueltas para cerrar el lazo)
    for _ in range(2):
        for i in range(n):
            j = (i + 1) % n
            a = a_lon_disp(a_accel, vl[i], kl[i])
            cap = _sqrt(vl[i] * vl[i] + 2.0 * a * dl[i])
            if cap < vl[j]:
                vl[j] = cap

    # 3) pasada atrás (2 vueltas)
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            a = a_lon_disp(a_brake, vl[j], kl[j])
            cap = _sqrt(vl[j] * vl[j] + 2.0 * a * dl[i])
            if cap < vl[i]:
                vl[i] = cap

    return np.maximum(np.asarray(vl), v_min)


def lap_time_estimate(points, v):
    """Tiempo de vuelta estimado: Σ Δs_i / v_media_del_tramo."""
    ds = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    v_seg = 0.5 * (v + np.roll(v, -1))
    return float(np.sum(ds / v_seg))
