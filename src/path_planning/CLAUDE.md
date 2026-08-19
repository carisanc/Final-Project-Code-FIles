# path_planning — memoria del paquete

## Revisión 7 jul 2026: safety_margin 0.32 + limpieza de racelines
- **`safety_margin` 0.25 → 0.32**: clearance 0.38→0.48 m (+26%) por +0.4 s de
  vuelta teórica (<1%). Trade-off medido, favorable — margen cinemático contra
  el derrape del MPC a ss=1.0. Raceline `saopaulo_gt.csv` regenerada + PP
  sanity lap limpio.
- **Borrada `slam_saopaulo.csv`** (v1, pre-círculo-de-fricción, desfasada) y
  repunteados los defaults de `raceline_view_launch` y `pure_pursuit_launch`
  a `saopaulo_gt.csv`. Racelines vigentes: `saopaulo_gt.csv` (operativa) y
  `slam_saopaulo_v3.csv` (mapa SLAM propio). Detalle: README §"Limpieza".

## Comandos exactos
```bash
# generar desde mapa GROUND-TRUTH (la OPERATIVA; binario, flood-fill):
ros2 run path_planning generate_raceline \
  --map maps/SaoPaulo_map.yaml --corridor-seed 0,0 \
  --params src/path_planning/config/raceline_params.yaml \
  --out src/path_planning/racelines/saopaulo_gt.csv \
  --debug-png docs/media/raceline_saopaulo_gt.png

# generar desde mapa SLAM propio (v3, offline ~90 s; falla si viola límites):
ros2 run path_planning generate_raceline \
  --map maps/slam_saopaulo_v3.yaml \
  --params src/path_planning/config/raceline_params.yaml \
  --out src/path_planning/racelines/slam_saopaulo_v3.csv \
  --debug-png docs/media/raceline_slam_saopaulo_v3.png

# visualizar:
ros2 launch path_planning raceline_view_launch.py            # con RViz
ros2 launch path_planning raceline_view_launch.py use_rviz:=false
```

## Gotcha: mapa SLAM vs ground-truth (`--corridor-seed`)
`clean_free_mask()` asume mapas de 3 estados (SLAM, con "desconocido"). Para
mapas ground-truth **binarios** (`SaoPaulo_map`) usar `--corridor-seed X,Y`, que
aísla el corredor con `corridor_from_seed()` (flood-fill 4-conexo desde la
semilla). Sin la semilla, en un mapa binario el "espacio libre más grande" es el
mapa entero y la raceline sale mal. La raceline ground-truth (`saopaulo_gt.csv`)
comparte frame con la física del sim → es la que usa Pure Pursuit (el mapa SLAM
estaba desfasado 2-3 m; ver `src/pure_pursuit/`).

## Gotcha: re-ejecutar generate_raceline requiere rebuild
`generate_raceline` es determinista y sin estado — correrlo de nuevo (p.ej.
tras cambiar `raceline_params.yaml`) simplemente sobreescribe `--out` y
`--debug-png`. **Pero** escribe en el árbol FUENTE
(`src/path_planning/racelines/*.csv`), mientras que `raceline_publisher` y
`raceline_view_launch.py` usan por defecto la ruta INSTALADA
(`install/path_planning/share/path_planning/racelines/...`, vía
`get_package_share_directory`). Igual que el gotcha de `sim.yaml` en SLAM:
tras regenerar el CSV hace falta `colcon build --packages-select
path_planning` para que el launch vea la versión nueva.

## Contrato del CSV (para Pure Pursuit / MPC)
`racelines/*.csv`, header `x,y,heading,kappa,v`, 400 filas, frame `map`,
lazo cerrado (la fila 0 NO se repite al final). El publicador lo sirve en
`/raceline` (nav_msgs/Path) con QoS **transient_local** — suscriptores deben
usar durabilidad transient_local o llegarán tarde al latch.

## Gotchas (ya resueltos, no re-descubrir)
1. **Mapas SLAM traen ~1,700 motas** de píxeles no-libres dentro de la pista
   → sin `clean_free_mask()` el esqueleto sale como malla (mini-lazos por
   cada mota). Regla: mayor componente libre + rellenar agujeros salvo los 2
   dominantes (exterior + isla).
2. **Zhang-Suen deja escaleras de 2 px** en diagonales (limitación clásica del
   algoritmo) → `thin_to_unit_width()` con test de componentes 8-conexas
   reales del vecindario (el "crossing number" del anillo NO detecta
   adyacencias diagonales entre vecinos no consecutivos — da falsos
   no-removibles).
3. **`thin` y `prune` deben alternarse hasta punto fijo**: podar ramas crea
   triángulos redundantes; adelgazar crea puntas nuevas. Una sola pasada de
   cada uno deja bifurcaciones y el ordenado del ciclo falla.
4. **`splprep(per=1)` avisa** `Setting x[...]=x[0]` — benigno, es el cierre
   del lazo periódico.
5. **QP = mínimos cuadrados acotado:** no hay cvxpy en el sistema;
   `scipy.optimize.lsq_linear(J, -κ₀, bounds)` resuelve el problema tal cual.
   J = D ⊙ (N·Nᵀ) con D segundas diferencias periódicas.
6. **El generador cachea nada:** cada corrida re-extrae el centerline (~70 s).
   Para iterar en el optimizador durante desarrollo, cachear centerline+anchos
   en un .npz temporal.

## Círculo de fricción en velocity_profile (6 jul)
La `a_lon` disponible por tramo es `a_max·√(1−(v²κ/a_lat_max)²)` — NO
constante. Se añadió porque PP aceleraba en plena salida de curva (27,57) y
chocaba a ss≥0.8; con el círculo la rampa se corre a la recta (vuelta teórica
44.6→46.0 s) y ss=0.8 quedó 100% fiable. No quitarlo "para ganar 1.4 s".

## Tuning
- Si la raceline zigzaguea: subir `smoothing_lambda`.
- Si roza paredes: subir `safety_margin` (el QP se pega a las cotas).
- Si las curvas salen lentas: subir `a_lat_max` (con cuidado: 6.0 ya es
  conservador; el sim tolera μ·g ≈ 10).
- `n_points` 400 va bien para ~350 m (Δs 0.86 m); pistas más chicas pueden
  bajar a 300.

## Mínimo tiempo (7 jul): la geometría YA es óptima; la palanca son los LÍMITES
`min_time.py` (`--objective time`): optimiza T directamente. HALLAZGO: en
SaoPaulo min-curvatura ≈ min-tiempo (+0.01 s) — no re-optimizar la línea,
subir límites. `saopaulo_racing.csv` (a_lat 8/a 6/v 10, teórico 38.5 s) =
EXPERIMENTAL: 38.1 s offline con el MPC dinámico, pero en sim AMCL no da la
precisión a 10 m/s y choca. Gotchas del optimizador: formulación RESIDUAL
sobre el warm start (submuestrear α directo destroza ápices), grad_eps 1e-2
(las esquinas de los min() del perfil), velocity_profile acelerado 25×
(bit-idéntico) porque es la función objetivo.

## Estado de racelines (7 jul tarde)
saopaulo_gt (46.0 validado) | **saopaulo_fast (41.5 ×2 VALIDADO en sim,
a_lat 7/a 5/v 9, MPC a±5 por CLI)** | saopaulo_racing (38 s solo con pose
GT — AMCL es el techo a 10 m/s, ver localization_bringup).
