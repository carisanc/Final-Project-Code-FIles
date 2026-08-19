# pure_pursuit — Seguimiento geométrico de la raceline

## 1. ¿Qué es y para qué sirve?

**Pure Pursuit** es un controlador que hace que el coche **siga la raceline**.
Es el primer eslabón que **junta toda la cadena**: usa el mapa (SLAM), la pose
del coche (Localización) y la línea de carrera (Path planning) para de verdad
*conducir*. Si Pure Pursuit da vueltas limpias, es que toda la cadena de abajo
funciona. Es el **paso de validación previo al MPC**.

Es *geométrico*: no usa un modelo del coche ni optimización (eso es el MPC).
Solo pura geometría, y por eso es fácil de entender.

## 2. Estructura de archivos

```
pure_pursuit/
├── pure_pursuit/pure_pursuit_node.py   ← el controlador (nodo ROS 2)
├── config/
│   ├── pure_pursuit_params.yaml        ← parámetros (lookahead, velocidad…)
│   └── pure_pursuit.rviz               ← vista de RViz (mapa + raceline + coche)
├── launch/pure_pursuit_launch.py       ← lanza sim + AMCL + raceline + control + RViz
├── package.xml · setup.py · setup.cfg
└── README.md
```

## 3. Teoría para aprender

### La intuición: la zanahoria en un palo

El coche persigue un punto que va corriendo por delante sobre la raceline, a
una distancia fija `L` (el *lookahead*, la "zanahoria"). Girar siempre hacia
ese punto lo mantiene sobre la línea. Nunca lo alcanza, pero **perseguirlo es
el algoritmo completo**.

### La matemática (4 pasos)

1. **Goal point.** Buscar el punto de la raceline a distancia ≈ `L` por delante
   (la raceline es un lazo cerrado → cuidado con el wrap-around).
2. **Pasarlo al marco del coche** (x = adelante, y = izquierda), con la pose
   `(x, y, θ)`:
   ```
   x_body =  cos(θ)·dx + sin(θ)·dy
   y_body = -sin(θ)·dx + cos(θ)·dy      (dx,dy = goal − coche)
   ```
3. **Curvatura del arco.** El círculo que sale del eje trasero y pasa por el
   goal da: `γ = 2·y_body / L²`. Si `y_body = 0` → recto; cuanto más al lado
   está el goal, más cerrado el giro.
4. **Ángulo de volante** (modelo de bicicleta, distancia entre ejes `W ≈ 0.33`):
   `δ = atan(W · γ)`, limitado a ±0.4 rad (máximo físico).

La **velocidad NO se calcula aquí**: la raceline ya trae `v` por waypoint (del
perfil de velocidad de Path planning); se lee la del waypoint más cercano.

### El parámetro clave: el lookahead `L`

Es la única perilla real, con un compromiso clásico:
- `L` pequeño → preciso pero **oscila** (sobre-corrige) a alta velocidad.
- `L` grande → suave pero **corta las curvas** y reacciona tarde.

Solución estándar: **lookahead adaptativo** `L = clip(k·v, L_min, L_max)` —
grande en recta rápida, chico en curva lenta.

## 4. Cómo ejecutarlo desde cero

Repo clonado + `docs/00-setup.md` hecho. Pure Pursuit necesita el simulador,
la localización, la raceline y su propio paquete — no hace falta compilar
todo el workspace:

**0. Compilar solo lo que este algoritmo usa:**
```bash
cd ~/roboracer-f1tenth
colcon build --packages-select f1tenth_gym_ros localization_bringup path_planning pure_pursuit
source install/setup.bash    # repetir en CADA terminal nueva
```

### A) Corrida directa (simulador primero, controlador después)

RViz puede tardar varios segundos en abrir (más en máquinas lentas). Si
lanzas todo junto, el controlador arranca YA y el coche puede llevar un
rato andando (o ya haber chocado) para cuando por fin ves algo. Con
`start_controller:=false` levantas **sim + AMCL + raceline + RViz sin que el
coche se mueva** — esperas a que RViz esté listo y RECIÉN AHÍ lanzas el
controlador, así lo ves desde el arranque:

```bash
# Terminal A — todo menos el controlador. Espera a que abra RViz:
ros2 launch pure_pursuit pure_pursuit_launch.py start_controller:=false

# Terminal B — una vez RViz ya está abierto y quieto, el controlador:
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  --params-file install/pure_pursuit/share/pure_pursuit/config/pure_pursuit_params.yaml \
  -p csv_path:=install/path_planning/share/path_planning/racelines/saopaulo_gt.csv \
  -p pose_source:=tf -p speed_scale:=0.6
```

Si tu máquina abre RViz rápido y no te importa verlo ya en marcha, el mismo
launch de un solo golpe (todo junto, `start_controller:=true` por defecto):
```bash
ros2 launch pure_pursuit pure_pursuit_launch.py pose_source:=tf speed_scale:=0.6
# args: pose_source(tf|odom) · speed_scale · lookahead_k · use_rviz · csv_path · map_yaml
```

### B) Corrida paso a paso, un terminal por pieza (para entender qué hace cada una)

**Terminal A — sim + AMCL**, localizando contra el mapa ground-truth (da la
pose del coche vía TF `map→base_link`):
```bash
ros2 launch localization_bringup localization_launch.py use_rviz:=false \
  map_yaml:=$PWD/maps/SaoPaulo_map.yaml
```

**Terminal B — la raceline** (publica `/raceline`, la línea a seguir):
```bash
ros2 run path_planning raceline_publisher --ros-args \
  -p csv_path:=install/path_planning/share/path_planning/racelines/saopaulo_gt.csv
```

**Terminal C — RViz** (dibuja mapa + raceline + coche). Espera a que abra
del todo antes del siguiente paso:
```bash
rviz2 -d install/pure_pursuit/share/pure_pursuit/config/pure_pursuit.rviz
```

**Terminal D — el controlador** (lee pose + raceline, publica `/drive`:
volante + velocidad). Con `pose_source:=odom` usa la pose ground-truth del
sim en vez de AMCL, para aislar el controlador de la localización:
```bash
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  --params-file install/pure_pursuit/share/pure_pursuit/config/pure_pursuit_params.yaml \
  -p csv_path:=install/path_planning/share/path_planning/racelines/saopaulo_gt.csv \
  -p pose_source:=tf -p speed_scale:=0.6
```

## 5. Parámetros que puedes tunear (`config/pure_pursuit_params.yaml`)

| Parámetro | Valor | Qué controla / en qué afecta |
|---|---|---|
| `pose_source` | `tf` | `tf` = pose de AMCL (valida la cadena) · `odom` = ground-truth (aísla el control) |
| `lookahead_k` | 0.3 | Lookahead por velocidad: `L = clip(k·v, min, max)`. Subir = más suave, corta más |
| `lookahead_min`/`_max` | 0.6 / 1.5 | Cotas del lookahead (m). **Grandes → corta el ápice** y roza la pared interior |
| `nearest_search_back`/`_fwd` | 10 / 40 | Ventana **local** de búsqueda del punto más cercano. Evita saltar al ramal de vuelta en horquillas |
| `speed_scale` | 1.0 | Factor sobre la v de la raceline. Bajar para pruebas seguras |
| `max_steering` | 0.4 | Límite físico del volante (rad) |
| `control_rate` | 30.0 | Hz del bucle de control |

`speed_scale` y `lookahead_k` también son *launch arguments* (tunear sin
recompilar).

## 6. Resultado esperado

El coche **completa vueltas limpias** (10+ seguidas, cero choques) por todo el
circuito —eses y horquilla incluidas— tanto con `odom` como con `tf` (AMCL),
a `speed_scale=0.6`. El **máximo 100% fiable es ss=0.8** (v ≈ 6.4 m/s), vuelta
limpia ~50-57 s. A ss=0.9 choca ~1/3 de las veces; ss=1.0 es el límite claro.

## 7. Problemas que tuvimos y cómo se resolvieron

| Problema | Causa | Solución |
|---|---|---|
| Chocaba en la horquilla (con `odom`) | El mapa SLAM estaba **desfasado 2-3 m** vs lo que ve el LiDAR → la raceline quedaba corrida de las paredes reales | Regenerar la raceline desde el mapa **ground-truth** `SaoPaulo_map` (`saopaulo_gt.csv`), que comparte frame con la física |
| Cortaba las curvas | Lookahead largo (el goal cruzaba la curva) + búsqueda **global** del más cercano (saltaba al ramal de vuelta) | Lookahead corto (`k=0.3`) + búsqueda **local** (ventana `nearest_search_*`) |
| Con `tf` (AMCL) iba 1-2 m atrasado en curva → cortaba y chocaba | AMCL tuneado para odom **ruidosa**; la del sim es ground-truth → inyectaba ruido artificial | **Perfil racing** de AMCL (`alphas 0.2→0.01`). Error de pose horquilla **2.24 → 0.10 m** |
| A ss≥0.8 chocaba siempre en la salida rápida (27,57) | El perfil aceleraba en plena curva y PP no anticipa | **Círculo de fricción** en `velocity_profile.py`: no deja acelerar hasta salir de la curva |

**La lección grande:** el fallo residual a ss≥0.9 es el **límite estructural
del controlador geométrico** — sin modelo ni horizonte, no puede frenar *antes*
de la curva ni acelerar *al salir*. Ese es exactamente el argumento para pasar
al **MPC**.

## 8. Glosario

- **Path tracking:** seguir una trayectoria dada (aquí, la raceline).
- **Lookahead point / zanahoria:** el punto adelante sobre la línea al que el
  coche apunta.
- **Marco del coche (body frame):** coordenadas centradas en el coche
  (x adelante, y izquierda), vs el marco global `map`.
- **Curvatura (γ, kappa):** cuánto se curva la trayectoria (1/radio).
- **Modelo de bicicleta / Ackermann:** aproximación de un coche con 2 ruedas;
  relaciona giro del volante con la curvatura.
- **Cross-track error (XTE):** distancia perpendicular del coche a la línea.
- **TF:** el árbol de transformaciones de ROS (cómo se relacionan los marcos
  `map`, `odom`, `base_link`).
- **speed_scale (ss):** factor que escala toda la velocidad de la raceline.

## 9. Cierre

Pure Pursuit está **COMPLETO** y valida la cadena entera SLAM→Localización→
Planning→Control. Su límite (ss=0.8 fiable) es el **baseline oficial del MPC**:
el siguiente algoritmo (`src/mpc/`) debe acercarse a ss=1.0 (46 s) sin chocar,
usando un modelo del coche y un horizonte de predicción para hacer lo que Pure
Pursuit no puede.
