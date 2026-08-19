#!/usr/bin/env python3

# ============================================================
# min_time.py — raceline de MÍNIMO TIEMPO (refinamiento sobre la de
# mínima curvatura).
#
# Por qué: min Σκ² produce la línea "más recta", pero el tiempo de vuelta
# es T = Σ Δs/v con v dictada por el perfil (agarre lateral + fricción
# longitudinal). Minimizar curvatura trata todas las curvas igual; el
# tiempo se gana en las curvas LENTAS y en las salidas a recta (la v de
# salida se arrastra toda la recta). Aquí se minimiza T DIRECTAMENTE:
#
#   min_α  T(α) + λ·Σ(Δα)²      s.a.  α ∈ [lb, ub]  (el corredor)
#
# donde P(α) = centro + α·n̂ (misma parametrización que el QP),
# κ = curvature_of(P), v = velocity_profile(P, κ, ...) — EL MISMO perfil
# (círculo de fricción incluido) que consume el MPC: una sola fuente de
# verdad. Solver: scipy L-BFGS-B con cotas de caja y gradiente numérico
# (sin dependencias nuevas). Warm start = la solución de mínima curvatura
# (está cerca del óptimo → pocas iteraciones y sin mínimos locales raros).
# ============================================================

import numpy as np
from scipy.optimize import minimize

from .min_curvature import _tangents_normals, curvature_of
from .velocity_profile import velocity_profile, lap_time_estimate


def _interp_matrix(n, ctrl_idx):
    """S (n, m): interpolación LINEAL PERIÓDICA de m puntos de control a
    los n waypoints (α_full = S @ α_ctrl). Parametrizar con menos puntos
    hace el problema mejor condicionado y la línea suave por construcción."""
    m = len(ctrl_idx)
    S = np.zeros((n, m))
    ext = np.append(ctrl_idx, ctrl_idx[0] + n)   # cierre periódico
    for i in range(n):
        j = int(np.searchsorted(ext, i, side='right')) - 1
        j = max(0, min(j, m - 1))
        i0, i1 = ext[j], ext[j + 1] if j + 1 <= m - 1 else ext[m]
        # tramo [i0, i1) → mezcla entre control j y (j+1) % m
        t = (i - i0) / max(i1 - i0, 1)
        S[i, j] = 1.0 - t
        S[i, (j + 1) % m] = t
    return S


def optimize_min_time(center, w_left, w_right, car_width, safety_margin,
                      alpha0, phys, smooth_lambda=2.0, maxiter=100,
                      ctrl_every=4, grad_eps=1e-2, verbose=True):
    """Refina la raceline minimizando el tiempo de vuelta.

    center (n,2), w_left/w_right (n,): centerline y anchos (como el QP).
    alpha0 (n,): warm start (el α de optimize_raceline).
    phys: dict con a_lat_max, a_accel, a_brake, v_max, v_min (del yaml).
    ctrl_every: submuestreo de los puntos de control de α (1 de cada k;
      con 400 wp y k=8 son 50 variables — el gradiente numérico a
      resolución completa es ruidoso por los min() del perfil y L-BFGS-B
      aborta la búsqueda de línea; medido: full-res mejoraba solo 0.09 s).
    grad_eps: paso del gradiente numérico (cm — promedia las "esquinas").
    Devuelve (raceline (n,2), alpha (n,), kappa (n,), v (n,)).
    """
    n = len(center)
    d_safe = car_width / 2.0 + safety_margin
    lb = -(np.asarray(w_right) - d_safe)
    ub = np.asarray(w_left) - d_safe

    # Normales del CENTRO, fijas (misma convención de cotas que el QP:
    # el corredor se mide desde la línea central).
    _, normals = _tangents_normals(center)

    # FORMULACIÓN RESIDUAL: α = α₀ + S·d, con d (puntos de control) que
    # arranca en 0. Preserva EXACTA la línea de mínima curvatura al inicio
    # (submuestrear α₀ directo la destrozaba: control cada 7 m recorta los
    # ápices — medido: el arranque interpolado daba 51+ s vs 46.45) y busca
    # desviaciones SUAVES por construcción.
    ctrl_idx = np.arange(0, n, ctrl_every)
    S = _interp_matrix(n, ctrl_idx)
    m = len(ctrl_idx)
    # Cotas del residual, conservadoras por ventana de influencia: α₀+S·d
    # debe quedar dentro del corredor en TODOS los waypoints.
    lb_d = np.empty(m)
    ub_d = np.empty(m)
    half = ctrl_every
    alpha0 = np.asarray(alpha0, dtype=float)
    for j, cj in enumerate(ctrl_idx):
        win = (cj + np.arange(-half, half + 1)) % n
        lb_d[j] = (lb[win] - alpha0[win]).max()
        ub_d[j] = (ub[win] - alpha0[win]).min()
    lb_d = np.minimum(lb_d, 0.0)   # d=0 siempre factible
    ub_d = np.maximum(ub_d, 0.0)

    def lap_time_of(pts):
        kappa = curvature_of(pts)
        v = velocity_profile(pts, kappa, phys['a_lat_max'], phys['a_accel'],
                             phys['a_brake'], phys['v_max'], phys['v_min'])
        return lap_time_estimate(pts, v)

    def cost(d):
        alpha = alpha0 + S @ d
        pts = center + alpha[:, None] * normals
        rough = float(np.sum((np.roll(d, -1) - d) ** 2))
        return lap_time_of(pts) + smooth_lambda * rough

    t0 = lap_time_of(center + alpha0[:, None] * normals)
    if verbose:
        print(f"      min-time: T inicial (mín-curvatura) = {t0:.2f} s "
              f"({m} puntos de control, residual)")

    res = minimize(
        cost, np.zeros(m), method='L-BFGS-B', bounds=list(zip(lb_d, ub_d)),
        options={'maxiter': maxiter, 'eps': grad_eps, 'ftol': 1e-9})

    alpha = alpha0 + S @ res.x
    pts = center + alpha[:, None] * normals
    kappa = curvature_of(pts)
    v = velocity_profile(pts, kappa, phys['a_lat_max'], phys['a_accel'],
                         phys['a_brake'], phys['v_max'], phys['v_min'])
    t1 = lap_time_estimate(pts, v)
    if verbose:
        print(f"      min-time: T final = {t1:.2f} s "
              f"(mejora {t0 - t1:+.2f} s, {res.nit} iters, "
              f"{res.nfev} evals, converged={res.success})")
    return pts, alpha, kappa, v
