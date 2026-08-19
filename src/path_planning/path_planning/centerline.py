#!/usr/bin/env python3

# ============================================================
# centerline.py — extrae la línea central de la pista desde un
# occupancy grid.
#
# Pipeline:
#   1. Binarizar el espacio libre (la pista).
#   2. Skeletonizar con Zhang-Suen → línea de 1 píxel de ancho.
#   3. Podar ramas espurias (quitar puntas iterativamente) hasta
#      que solo queden ciclos, y quedarse con el ciclo más grande.
#   4. Ordenar los píxeles del ciclo como secuencia cerrada
#      (caminando por vecindad-8).
#   5. Pasar a metros, remuestrear uniforme en arco y suavizar
#      con una spline periódica.
#   6. Medir el ancho libre a cada lado (marchando por la normal
#      hasta chocar con pared).
# ============================================================

import numpy as np
from scipy import ndimage
from scipy.interpolate import splprep, splev


# ----------------------------------------------------------
# 1. Limpieza de la máscara libre
# ----------------------------------------------------------
def clean_free_mask(free):
    """
    Los mapas de SLAM traen "motas" de píxeles no-libres dentro de la
    pista (ruido de mapeo: ~1700 en slam_saopaulo). Zhang-Suen preserva
    topología, así que cada mota generaría un mini-lazo y el esqueleto
    saldría como una malla en vez de un anillo limpio.

    Regla: (a) conservar solo la componente libre más grande (la pista);
    (b) rellenar todos los agujeros no-libres SALVO los dos dominantes,
    que en un circuito cerrado son exactamente el mundo exterior y la
    isla interior. Es la topología correcta de un anillo.
    """
    # (a) componente libre más grande
    labels, n = ndimage.label(free, structure=np.ones((3, 3)))
    if n == 0:
        raise RuntimeError("Máscara libre vacía: revisa el umbral de binarizado.")
    sizes = ndimage.sum(free, labels, index=range(1, n + 1))
    free = labels == (int(np.argmax(sizes)) + 1)

    # (b) rellenar agujeros salvo los 2 más grandes (exterior + isla)
    inv = ~free
    labels, n = ndimage.label(inv)
    if n > 2:
        sizes = ndimage.sum(inv, labels, index=range(1, n + 1))
        keep = np.argsort(sizes)[::-1][:2] + 1
        fill = inv & ~np.isin(labels, keep)
        free = free | fill
    return free


def corridor_from_seed(free, map_data, seed_world):
    """
    Aísla el corredor de la pista con un flood-fill desde un punto semilla.

    Para mapas GROUND-TRUTH (p. ej. `SaoPaulo_map`, el que usa el simulador
    para física y LiDAR) `clean_free_mask` NO sirve: son binarios (pared vs
    libre) sin estado "desconocido", así que "el espacio libre más grande" es
    todo el mapa (corredor + isla interior + exterior forman una sola región
    conexa por fuera de las paredes). En cambio, si las paredes cierran el
    circuito, la componente conexa 4-vecinos del espacio libre que contiene un
    punto de la PISTA es exactamente el corredor de carrera.

    `seed_world`: (x, y) en metros, un punto sobre la pista (típicamente el
    arranque). Devuelve la máscara booleana del corredor, con la misma
    semántica que `clean_free_mask` (True = pista) para el resto del pipeline.
    """
    r, c = map_data.world_to_px(seed_world[0], seed_world[1])
    r, c = int(round(float(r))), int(round(float(c)))
    if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1]):
        raise RuntimeError(f"Semilla {seed_world} fuera del mapa (px {r},{c}).")
    if not free[r, c]:
        raise RuntimeError(
            f"Semilla {seed_world} (px {r},{c}) cae en pared/no-libre; "
            "elige un punto sobre la pista.")
    # 4-conexo (estructura en cruz): una pared de 1 px en diagonal ya corta el
    # relleno, evitando fugas del corredor hacia el interior/exterior.
    labels, _ = ndimage.label(free)
    corridor = labels == labels[r, c]
    return corridor


# ----------------------------------------------------------
# 2. Skeletonización Zhang-Suen (vectorizada con numpy)
# ----------------------------------------------------------
def zhang_suen_skeleton(mask, max_iter=500):
    """
    Adelgaza una máscara binaria hasta un esqueleto de 1 píxel.
    Implementación clásica de Zhang-Suen (1984), dos sub-pasadas
    por iteración, vectorizada sobre todo el grid.
    """
    img = mask.astype(np.uint8).copy()
    # Asegurar borde vacío (np.roll envuelve en los bordes)
    img[0, :] = img[-1, :] = 0
    img[:, 0] = img[:, -1] = 0

    def neighbors(im):
        # P2..P9 en sentido horario empezando por el norte.
        # np.roll(im, 1, axis=0)[r, c] == im[r-1, c] (vecino norte), etc.
        P2 = np.roll(im, 1, axis=0)                      # N
        P3 = np.roll(np.roll(im, 1, axis=0), -1, axis=1)  # NE
        P4 = np.roll(im, -1, axis=1)                     # E
        P5 = np.roll(np.roll(im, -1, axis=0), -1, axis=1)  # SE
        P6 = np.roll(im, -1, axis=0)                     # S
        P7 = np.roll(np.roll(im, -1, axis=0), 1, axis=1)  # SW
        P8 = np.roll(im, 1, axis=1)                      # W
        P9 = np.roll(np.roll(im, 1, axis=0), 1, axis=1)  # NW
        return [P2, P3, P4, P5, P6, P7, P8, P9]

    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            P = neighbors(img)
            B = sum(P)  # nº de vecinos encendidos
            # A = nº de transiciones 0→1 en la secuencia P2,P3,...,P9,P2
            seq = P + [P[0]]
            A = sum(((seq[k] == 0) & (seq[k + 1] == 1)).astype(np.uint8)
                    for k in range(8))
            if step == 0:
                cond = (P[0] * P[2] * P[4] == 0) & (P[2] * P[4] * P[6] == 0)
            else:
                cond = (P[0] * P[2] * P[6] == 0) & (P[0] * P[4] * P[6] == 0)
            remove = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & cond
            if remove.any():
                img[remove] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


# ----------------------------------------------------------
# 2b. Adelgazado a ancho unitario
# ----------------------------------------------------------
def _neighbor_components(patch):
    """
    Nº de componentes 8-conexas de los vecinos encendidos dentro del
    parche 3×3 (excluyendo el centro). Si es 1, quitar el centro no
    rompe la conectividad local (el píxel es redundante).

    Nota: el clásico "crossing number" del anillo NO sirve aquí — en
    escaleras dobles dos vecinos no consecutivos del círculo pueden ser
    diagonales entre sí, y el anillo los cuenta como 2 componentes
    cuando en realidad son 1.
    """
    cells = [(r, c) for r in range(3) for c in range(3)
             if (r, c) != (1, 1) and patch[r, c]]
    if not cells:
        return 0
    remaining = set(cells)
    comps = 0
    while remaining:
        comps += 1
        stack = [remaining.pop()]
        while stack:
            cr, cc = stack.pop()
            for nr in (cr - 1, cr, cr + 1):
                for ncc in (cc - 1, cc, cc + 1):
                    if (nr, ncc) in remaining:
                        remaining.discard((nr, ncc))
                        stack.append((nr, ncc))
    return comps


def thin_to_unit_width(skel, max_passes=50):
    """
    Zhang-Suen no garantiza ancho de 1 píxel: deja "escaleras" de 2 px
    en tramos diagonales (limitación conocida del algoritmo). Esta
    post-pasada elimina secuencialmente píxeles *simples* redundantes:
    aquellos cuya remoción no rompe la conectividad local (A(p) == 1)
    y que no son puntas (B >= 3). Se procesan en orden raster,
    re-evaluando el vecindario ya modificado — el orden secuencial es
    lo que garantiza no cortar la curva.
    """
    sk = skel.copy()
    for _ in range(max_passes):
        ncount = ndimage.convolve(sk.astype(np.uint8), _NEIGH_KERNEL,
                                  mode='constant', cval=0)
        # candidatos: todo píxel con >= 2 vecinos (coincide con la condición
        # interna B >= 2; los píxeles de paso normales sobreviven porque sus
        # 2 vecinos no son adyacentes entre sí → 2 componentes)
        cand = np.argwhere(sk & (ncount >= 2))
        removed = 0
        for r, c in cand:
            if not sk[r, c]:
                continue
            patch = sk[r - 1:r + 2, c - 1:c + 2]
            if patch.shape != (3, 3):
                continue
            B = patch.sum() - 1
            # B >= 2: en una curva limpia los 2 vecinos de un píxel de paso
            # NO son adyacentes entre sí (→ 2 componentes → protegido); solo
            # caen esquinas/triángulos redundantes donde sí lo son.
            if B >= 2 and _neighbor_components(patch) == 1:
                sk[r, c] = False
                removed += 1
        if removed == 0:
            break
    return sk


# ----------------------------------------------------------
# 3. Poda de ramas: quitar puntas hasta que solo queden ciclos
# ----------------------------------------------------------
_NEIGH_KERNEL = np.array([[1, 1, 1],
                          [1, 0, 1],
                          [1, 1, 1]], dtype=np.uint8)


def prune_to_cycles(skel, max_iter=2000):
    """
    Elimina iterativamente los píxeles-punta (con ≤1 vecino).
    Las ramas espurias se comen desde la punta hacia adentro;
    los ciclos (donde todo píxel tiene ≥2 vecinos) sobreviven.
    """
    sk = skel.copy()
    for _ in range(max_iter):
        ncount = ndimage.convolve(sk.astype(np.uint8), _NEIGH_KERNEL,
                                  mode='constant', cval=0)
        tips = sk & (ncount <= 1)
        if not tips.any():
            break
        sk[tips] = False
    return sk


def largest_cycle(skel):
    """Se queda con la componente conexa (vecindad-8) más grande."""
    labels, n = ndimage.label(skel, structure=np.ones((3, 3)))
    if n == 0:
        raise RuntimeError("El esqueleto quedó vacío tras la poda: "
                           "revisa la binarización del mapa.")
    sizes = ndimage.sum(skel, labels, index=range(1, n + 1))
    return labels == (int(np.argmax(sizes)) + 1)


# ----------------------------------------------------------
# 4. Ordenar el ciclo como secuencia cerrada
# ----------------------------------------------------------
def order_cycle_fast(skel):
    """
    Camina el ciclo píxel a píxel por vecindad-8 y devuelve la
    secuencia ordenada de (fila, col). En un ciclo limpio cada
    píxel tiene exactamente 2 vecinos, así que el recorrido es
    determinista: siempre avanzar al vecino que no es el anterior.
    Usa un set de visitados para que el recorrido sea O(n).
    """
    pix = set(zip(*np.nonzero(skel)))
    start = next(iter(pix))
    order = [start]
    visited = {start}
    prev = None
    cur = start
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1),
               (-1, -1), (-1, 1), (1, -1), (1, 1)]  # preferir 4-vecindad
    while True:
        nbrs = [(cur[0] + dr, cur[1] + dc) for dr, dc in offsets
                if (cur[0] + dr, cur[1] + dc) in pix]
        nxt = None
        for cand in nbrs:
            if cand == prev:
                continue
            if cand == start and len(order) > 2:
                nxt = None  # ciclo cerrado
                break
            if cand not in visited:
                nxt = cand
                break
        if nxt is None:
            break
        order.append(nxt)
        visited.add(nxt)
        prev, cur = cur, nxt
    if len(order) < 0.9 * len(pix):
        raise RuntimeError(
            f"Recorrido del ciclo incompleto ({len(order)}/{len(pix)} px). "
            "El esqueleto aún tiene bifurcaciones; sube la poda.")
    return np.array(order)


# ----------------------------------------------------------
# 5. Remuestreo uniforme + spline periódica
# ----------------------------------------------------------
def resample_smooth(points_xy, n_points, smoothing=5.0):
    """
    points_xy: (M, 2) en metros, secuencia cerrada ordenada.
    Devuelve (n_points, 2) uniformes en longitud de arco, suavizados
    con spline cúbica periódica, más tangentes unitarias (n_points, 2).
    """
    pts = np.asarray(points_xy, dtype=np.float64)
    # splprep periódica: cierra la curva sola (per=1)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing, per=1)

    # Muestrear denso para medir longitud de arco real
    u_dense = np.linspace(0.0, 1.0, 20 * n_points, endpoint=False)
    xd, yd = splev(u_dense, tck)
    d = np.sqrt(np.diff(xd, append=xd[0]) ** 2 + np.diff(yd, append=yd[0]) ** 2)
    s_cum = np.concatenate(([0.0], np.cumsum(d)[:-1]))
    total_len = np.sum(d)

    # u(s): invertir la longitud de arco para muestrear uniforme
    s_targets = np.linspace(0.0, total_len, n_points, endpoint=False)
    u_uniform = np.interp(s_targets, s_cum, u_dense)

    x, y = splev(u_uniform, tck)
    dx, dy = splev(u_uniform, tck, der=1)
    tang = np.stack([dx, dy], axis=1)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    return np.stack([x, y], axis=1), tang, total_len


# ----------------------------------------------------------
# 6. Anchos libres a cada lado (marcha sobre la normal)
# ----------------------------------------------------------
def track_widths(points_xy, tangents, map_data, free, max_width=5.0):
    """
    Para cada punto, marcha sobre la normal (izquierda = +90° de la
    tangente) en pasos de resolution/2 hasta salir del espacio libre.
    Recibe la máscara libre YA limpia (clean_free_mask) para que las
    motas de ruido no produzcan anchos falsos ~0.
    Devuelve (w_left, w_right) en metros.
    """
    step = map_data.resolution / 2.0
    n_steps = int(max_width / step)

    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)  # +90°
    w_left = np.full(len(points_xy), max_width)
    w_right = np.full(len(points_xy), max_width)

    for sign, out in ((1.0, w_left), (-1.0, w_right)):
        for k in range(1, n_steps + 1):
            probe = points_xy + sign * normals * (k * step)
            rows, cols = map_data.world_to_px(probe[:, 0], probe[:, 1])
            r = np.clip(np.round(rows).astype(int), 0, map_data.height - 1)
            c = np.clip(np.round(cols).astype(int), 0, map_data.width - 1)
            hit = ~free[r, c] & (out == max_width)
            out[hit] = (k - 1) * step
    return w_left, w_right


# ----------------------------------------------------------
# Orquestador del módulo
# ----------------------------------------------------------
def extract_centerline(map_data, n_points=400, spline_smoothing=5.0,
                       corridor_seed=None):
    """
    Mapa → centerline (n,2), tangentes (n,2), anchos (n,), (n,), longitud.

    `corridor_seed`: si es None (default), se limpia la máscara con
    `clean_free_mask` (mapas SLAM de 3 estados). Si es (x, y) en metros, se
    aísla el corredor con `corridor_from_seed` (mapas ground-truth binarios).
    """
    if corridor_seed is None:
        free = clean_free_mask(map_data.free_mask())
    else:
        free = corridor_from_seed(map_data.free_mask(), map_data, corridor_seed)
    skel = zhang_suen_skeleton(free)
    # thin y prune se retroalimentan: al podar una rama, el píxel de unión
    # queda con 2 vecinos adyacentes entre sí (triángulo redundante) que solo
    # thin sabe quitar; y al adelgazar pueden aparecer puntas nuevas que solo
    # prune sabe comer. Alternar hasta punto fijo (converge en 2-4 vueltas).
    for _ in range(10):
        before = skel.sum()
        skel = prune_to_cycles(thin_to_unit_width(skel))
        if skel.sum() == before:
            break
    skel = largest_cycle(skel)
    order_px = order_cycle_fast(skel)

    x, y = map_data.px_to_world(order_px[:, 0], order_px[:, 1])
    pts = np.stack([x, y], axis=1)

    center, tang, length = resample_smooth(pts, n_points, spline_smoothing)
    w_left, w_right = track_widths(center, tang, map_data, free)
    return center, tang, w_left, w_right, length
