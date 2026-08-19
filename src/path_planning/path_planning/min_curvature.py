#!/usr/bin/env python3

# ============================================================
# min_curvature.py — optimización de mínima curvatura (QP).
#
# Idea (Heilmeier et al., TUM): la raceline se escribe como el
# centerline más un desplazamiento lateral α_i sobre la normal
# de cada punto:
#
#     P_i = C_i + α_i · n̂_i
#
# La curvatura discreta se aproxima con la segunda derivada por
# diferencias finitas periódicas. Como P es lineal en α, la
# curvatura también (linealización):
#
#     κ ≈ J·α + κ₀
#
# y minimizar Σ κ_i² con cotas de caja sobre α es exactamente un
# problema de mínimos cuadrados acotado:
#
#     min ‖J·α + κ₀‖²   s.a.   α_min ≤ α ≤ α_max
#
# que resuelve scipy.optimize.lsq_linear sin dependencias extra.
# Se re-linealiza 2-3 veces (recalculando normales) para refinar.
# ============================================================

import numpy as np
from scipy.optimize import lsq_linear


def _tangents_normals(points):
    """Tangentes y normales unitarias por diferencias centradas periódicas."""
    fwd = np.roll(points, -1, axis=0)
    bwd = np.roll(points, 1, axis=0)
    tang = fwd - bwd
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normals = np.stack([-tang[:, 1], tang[:, 0]], axis=1)  # +90° (izquierda)
    return tang, normals


def _second_diff_matrix(n, ds):
    """Matriz D (n×n) de segundas diferencias periódicas: (D·P) ≈ P''."""
    D = np.zeros((n, n))
    idx = np.arange(n)
    D[idx, idx] = -2.0
    D[idx, (idx + 1) % n] = 1.0
    D[idx, (idx - 1) % n] = 1.0
    return D / ds**2


def curvature_of(points):
    """Curvatura discreta κ_i (con signo) de una secuencia cerrada."""
    n = len(points)
    d = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    ds = d.mean()
    D = _second_diff_matrix(n, ds)
    _, normals = _tangents_normals(points)
    ddP = D @ points                      # (n, 2) ≈ P''
    return np.einsum('ij,ij->i', ddP, normals)


def optimize_raceline(center, w_left, w_right, car_width, safety_margin,
                      smoothing_lambda=0.1, iterations=3):
    """
    center: (n,2) centerline cerrado; w_left/w_right: anchos libres (n,).
    Devuelve raceline (n,2), α final (n,) y κ de la raceline (n,).
    """
    n = len(center)
    d_safe = car_width / 2.0 + safety_margin

    # Cotas de α fijas respecto del CENTERLINE (las normales se recalculan,
    # pero el corredor disponible se mide desde la línea central).
    lb = -(np.asarray(w_right) - d_safe)
    ub = np.asarray(w_left) - d_safe
    if np.any(lb >= ub):
        bad = int(np.sum(lb >= ub))
        raise RuntimeError(
            f"{bad} puntos sin corredor: la pista es más angosta que "
            f"car_width/2 + safety_margin = {d_safe:.2f} m a cada lado.")

    pts = center.copy()
    alpha_total = np.zeros(n)

    for _ in range(iterations):
        d = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
        ds = d.mean()
        D = _second_diff_matrix(n, ds)
        _, normals = _tangents_normals(pts)

        # κ(α) = J·α + κ₀ con P(α) = pts + diag(α)·N
        # (D·P)_i·n̂_i es lineal en α:  J[i,j] = D[i,j]·(n̂_j·n̂_i)
        kappa0 = np.einsum('ij,ij->i', D @ pts, normals)
        J = D * (normals @ normals.T)     # producto elemento a elemento

        # Término de suavidad sobre α (primeras diferencias periódicas):
        # desalienta cambios bruscos de desplazamiento entre puntos vecinos.
        if smoothing_lambda > 0:
            D1 = np.zeros((n, n))
            idx = np.arange(n)
            D1[idx, idx] = 1.0
            D1[idx, (idx + 1) % n] = -1.0
            A = np.vstack([J, np.sqrt(smoothing_lambda) * D1])
            b = np.concatenate([-kappa0, np.zeros(n)])
        else:
            A, b = J, -kappa0

        # Cotas restantes en esta iteración (α acumulado no puede exceder
        # el corredor original)
        lb_it = lb - alpha_total
        ub_it = ub - alpha_total

        res = lsq_linear(A, b, bounds=(lb_it, ub_it), max_iter=200)
        alpha = res.x

        pts = pts + alpha[:, None] * normals
        alpha_total += alpha

    kappa_final = curvature_of(pts)
    return pts, alpha_total, kappa_final
