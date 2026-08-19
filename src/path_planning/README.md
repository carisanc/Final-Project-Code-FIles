# path_planning — Raceline de mínima curvatura (código propio)

## 1. ¿Qué es y para qué sirve?

Con el mapa (SLAM) y la pose (Localización) resueltos, Path planning responde
**por dónde conviene ir**: calcula una sola vez por pista (offline) la **línea
de carrera (raceline)** óptima y la guarda como un CSV de waypoints. Pure
Pursuit y el MPC **no deciden la ruta** — la *siguen*. A diferencia de SLAM y
Localización (que integran librerías), aquí **el código es propio**.

## 2. Estructura de archivos

```
path_planning/
├── path_planning/
│   ├── map_utils.py          ← carga mapas, conversión píxel ↔ metros
│   ├── centerline.py         ← esqueleto de la pista + anchos a cada pared
│   ├── min_curvature.py      ← la optimización QP (línea más recta posible)
│   ├── min_time.py           ← variante: minimiza el TIEMPO de vuelta
│   ├── velocity_profile.py   ← perfil de velocidad (fricción + acel/frenado)
│   ├── generate_raceline.py  ← CLI que orquesta todo → CSV + PNG + métricas
│   └── raceline_publisher.py ← nodo: CSV → /raceline (para RViz y controladores)
├── racelines/                ← los CSV generados (saopaulo_gt, _fast, …)
├── config/raceline_params.yaml · raceline_view.rviz
├── launch/raceline_view_launch.py
└── README.md
```

## 3. Teoría para aprender

### ¿Por qué "mínima curvatura" y no el centro de la pista?

- **Centerline** (centro): seguro pero lento (más curvatura en cada curva).
- **Shortest path** (pegado por dentro): corto pero con picos de curvatura en
  los ápices → frenadas fuertes.
- **Mínima curvatura:** la línea del piloto real, **"out-in-out"** (entrar
  abierto, tocar el ápice, salir abierto). No es la más corta, pero minimiza
  la curvatura.

**Por qué la curvatura manda:** la fricción limita la aceleración lateral, así
que en un punto de curvatura `κ` la velocidad sostenible es:

```
v² · κ ≤ a_lat_max      →      v_max(s) = sqrt(a_lat_max / κ(s))
```

Menos curvatura → más velocidad → menos frenadas → menor tiempo de vuelta.

### El pipeline (de mapa a CSV), en 3 pasos

1. **Centerline + anchos.** El mapa es una imagen; con **skeletonización**
   (adelgazar la zona libre a una línea de 1 píxel) se saca el esqueleto de la
   pista. La **transformada de distancia** da, en cada punto, cuánto hay a cada
   pared → el margen para mover la línea.
2. **Optimización QP de mínima curvatura.** La raceline = centerline + un
   **desplazamiento lateral `α_i`** por punto (a lo largo de la normal),
   acotado por los anchos menos medio coche y un margen. Aproximando la
   curvatura de forma lineal en los `α`, queda un **programa cuadrático**:
   ```
   min  Σ κ_i(α)²      sujeto a      α_min,i ≤ α_i ≤ α_max,i
   ```
   Es el enfoque estándar en F1TENTH (Heilmeier/TUM). Se resuelve con
   `scipy.optimize.lsq_linear` en segundos.
3. **Perfil de velocidad.** Con la curvatura final: `v = sqrt(a_lat_max/κ)`
   (límite de fricción), y **dos pasadas** (adelante limitando la aceleración,
   atrás limitando la frenada) para que sea físicamente alcanzable.

### El círculo de fricción (clave, añadido para Pure Pursuit)

El neumático reparte su agarre entre girar y acelerar. La aceleración
longitudinal disponible NO es constante:

```
a_lon_disponible = a_max · √(1 − (v²κ / a_lat_max)²)
```

En plena curva queda poco para acelerar → la rampa de aceleración se corre
sola hacia la recta. Sin esto, el perfil mandaba acelerar en plena salida de
curva y Pure Pursuit (que no anticipa) chocaba la pared exterior. Cuesta
+1.4 s teóricos pero desbloqueó ss=0.8 fiable.

## 4. Cómo ejecutarlo desde cero

```bash
cd ~/roboracer-f1tenth && colcon build && source install/setup.bash

# 1) Generar la raceline (offline, ~90 s). La OPERATIVA sale del mapa
#    ground-truth (--corridor-seed aísla el corredor en mapas binarios):
ros2 run path_planning generate_raceline \
  --map maps/SaoPaulo_map.yaml --corridor-seed 0,0 \
  --params src/path_planning/config/raceline_params.yaml \
  --out src/path_planning/racelines/saopaulo_gt.csv \
  --debug-png docs/media/raceline_saopaulo_gt.png

# 2) OJO: el generador escribe en src/, pero el visualizador usa la copia
#    INSTALADA → recompilar para que el paso 3 vea el CSV nuevo:
colcon build --packages-select path_planning && source install/setup.bash

# 3) Visualizar la raceline sobre el mapa en RViz:
ros2 launch path_planning raceline_view_launch.py
```

El CSV tiene header `x,y,heading,kappa,v` (una fila por waypoint, frame `map`).
Es el **contrato de entrada** de Pure Pursuit y del MPC. Para la línea de
mínimo tiempo, añade `--objective time`.

## 5. Parámetros que puedes tunear (`config/raceline_params.yaml`)

| Parámetro | Valor | Qué controla / en qué afecta |
|---|---|---|
| `car_width`/`safety_margin` | 0.31 / **0.32** m | Corredor permitido. **Subir el margen** = la línea no pega tanto el ápice (más clearance, poco más lenta) |
| `a_lat_max` | 6.0 m/s² | Fricción lateral utilizable → v_max por curvatura. Subir = curvas más rápidas (riesgo derrape) |
| `a_accel`/`a_brake` | 4.0 / 4.0 | Aceleración/frenado del perfil |
| `v_max`/`v_min` | 8.0 / 1.5 | Topes de velocidad |
| `n_points` | 400 | Nº de waypoints (Δs ≈ 0.86 m). Pistas chicas: bajar a 300 |
| `smoothing_lambda` | 0.1 | Suavidad del desplazamiento (subir si la línea zigzaguea) |

> **Cómo tunear:** si roza paredes → sube `safety_margin`. Si zigzaguea → sube
> `smoothing_lambda`. Si las curvas salen lentas → sube `a_lat_max` (con
> cuidado). Regenera (paso 1) y **recompila** (paso 2).

## 6. Resultado esperado

Una línea suave dentro de la pista, cortando los ápices "out-in-out", con
curvatura máxima menor que la del centerline. La raceline **operativa** es
`saopaulo_gt.csv` (mapa ground-truth). Hay 3 puntos de operación validados:

| CSV | Límites (a_lat/a/v) | Teórico | En sim (tf/AMCL) | Estado |
|---|---|---|---|---|
| `saopaulo_gt.csv` | 6/4/8 | 46.4 s | **46.0 s limpio** | operativa conservadora |
| `saopaulo_fast.csv` | 7/5/9 | 42.0 s | **41.5 s ×2 limpio** | **operativa rápida** |
| `saopaulo_racing.csv` | 8/6/10 | 38.5 s | choca (AMCL) | experimental (38.3 s con pose GT) |

**Hallazgo importante:** en SaoPaulo la línea de mínima curvatura **ya es la de
mínimo tiempo** (+0.01 s). Con `v_max` capando casi toda la vuelta, la palanca
NO es la geometría sino **los límites** (fricción/aceleración/velocidad). El
techo de 38 s lo bloquea AMCL a 10 m/s, no la línea.

## 7. Problemas que tuvimos y cómo se resolvieron

| Problema | Causa | Solución |
|---|---|---|
| El esqueleto salía como una malla (41k px) | El mapa SLAM tiene ~1700 motas de ruido; el thinning preserva topología → un mini-lazo por mota | `clean_free_mask()`: mayor componente libre + rellenar agujeros salvo los 2 dominantes |
| Zhang-Suen dejaba líneas de 2 px de ancho | Limitación clásica del algoritmo (escaleras diagonales) | `thin_to_unit_width()` con test de componentes 8-conexas reales del vecindario |
| En mapa ground-truth (binario) la línea trazaba el perímetro entero | No hay estado "desconocido" que acote el corredor | `--corridor-seed X,Y`: flood-fill desde una semilla en la pista |
| El MPC en el límite (ss=1.0) rozaba en 3 puntos | El derrape abría el coche ~0.4 m; clearance 0.38 no daba | `safety_margin` 0.25 → 0.32 (clearance 0.48 m, +26%, por +0.4 s) |
| PP aceleraba en plena curva y chocaba | El perfil daba aceleración plena sin importar la curvatura | **Círculo de fricción** en `velocity_profile.py` |

## 8. Glosario

- **Raceline:** la línea de carrera, un CSV de waypoints `(x,y,heading,κ,v)`.
- **Waypoint:** un punto de la trayectoria con su rumbo, curvatura y velocidad.
- **Curvatura (κ):** 1/radio; cuánto se dobla la trayectoria en ese punto.
- **Ápice (apex):** el punto más interior de una curva.
- **Skeletonización / thinning:** adelgazar una región a una línea de 1 píxel.
- **Transformada de distancia:** imagen donde cada píxel guarda su distancia al
  obstáculo más cercano (aquí, a la pared).
- **QP (programa cuadrático):** optimización de una función cuadrática con
  restricciones lineales; aquí, minimizar Σκ².
- **Perfil de velocidad:** la `v` asignada a cada waypoint según la física.
- **Círculo de fricción:** el agarre total del neumático repartido entre girar
  y acelerar/frenar.

## 9. Cierre

Path planning está **completo**: raceline operativa `saopaulo_gt.csv` +
variantes rápidas, con el círculo de fricción integrado. Con una línea que
seguir, los siguientes eslabones son los controladores: primero **Pure Pursuit**
(`src/pure_pursuit/`, geométrico) para validar la cadena, y luego el **MPC**
(`src/mpc/`, óptimo) para acercarse al límite físico.
