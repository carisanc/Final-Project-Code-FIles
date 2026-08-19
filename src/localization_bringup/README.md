# localization_bringup — Localización (AMCL / filtro de partículas)

## 1. ¿Qué es y para qué sirve?

Con el mapa ya hecho (SLAM), la **Localización** responde una sola pregunta en
tiempo real: **¿dónde está el coche dentro del mapa, ahora?** La necesitan Pure
Pursuit y el MPC: ellos siguen una raceline definida en el marco `map`, y para
medir su error respecto al siguiente punto necesitan la pose del coche en `map`
—no solo la odometría, que se desvía—. Este paquete es *bringup* para correr
[`nav2_amcl`](https://github.com/ros-planning/navigation2) + un nodo propio
mínimo, `scan_throttle`.

## 2. Estructura de archivos

```
localization_bringup/
├── launch/localization_launch.py           ← sim + map_server + amcl + throttle (+RViz)
├── config/
│   ├── amcl_params.yaml                     ← parámetros de AMCL (perfil racing)
│   ├── amcl_params_hispeed.yaml             ← perfil experimental para 10 m/s
│   └── localization.rviz                    ← vista con la nube de partículas
├── localization_bringup/scan_throttle.py    ← nodo aux: baja /scan de 250→40 Hz
├── package.xml · setup.py · setup.cfg
└── README.md
```

## 3. Teoría para aprender

### Filtro de partículas (Monte Carlo Localization)

La creencia sobre la pose no es un punto, sino una **nube de hipótesis**
(partículas), cada una con un peso. Por cada odometría + scan:

1. **Predicción (modelo de movimiento):** mueve todas las partículas según la
   odometría + ruido. La nube se esparce.
2. **Corrección (modelo de sensor):** por cada partícula se hace **ray-casting**
   contra el mapa ("si el coche estuviera aquí, ¿qué vería el LiDAR?") y se
   compara con el `/scan` real. Las que coinciden reciben peso alto.
3. **Resampling:** se sortea una nube nueva proporcional al peso. Las buenas se
   reproducen, las malas mueren → la nube colapsa sobre la pose real y la sigue.

**AMCL** ("Adaptive") ajusta el número de partículas (muchas cuando está
perdido, pocas al converger) e inyecta partículas aleatorias para recuperarse
del "kidnapped robot".

### La salida clave: la TF `map → odom`

AMCL publica la transformada `map → ego_racecar/odom`, **el mismo eslabón que
publicaba `slam_toolbox`**. Por eso reusa el mismo árbol TF y los mismos
parches de `gym_bridge.py`. Esa TF es lo que consumen Pure Pursuit y el MPC.

> **Honestidad sobre el sim:** en `f1tenth_gym_ros` la odometría es
> ground-truth, así que AMCL converge casi al instante y `map→odom ≈ identidad`.
> Eso NO es un defecto: el valor es (a) aprender MCL y (b) producir la pose en
> `map` que necesitan los controladores. En un coche real la odom deriva y AMCL
> sí trabaja de verdad.

## 4. Cómo ejecutarlo desde cero

```bash
cd ~/roboracer-f1tenth && colcon build && source install/setup.bash

# 1) sim + AMCL + map_server + scan_throttle (+ RViz). Localiza contra el mapa
#    ground-truth por defecto (el validado).
ros2 launch localization_bringup localization_launch.py

# 2) En otra terminal, conducir para ver a AMCL seguir al coche:
source install/setup.bash
ros2 run ftg_tuned ftg_node
```

En RViz, la **nube de partículas** (`/particlecloud`, roja) debe colapsar sobre
el coche y seguirlo. Qué hace cada pieza: `map_server` sirve el mapa fijo;
`scan_throttle` reentrega `/scan` a 40 Hz como `/scan_amcl`; `amcl` corre el
filtro y publica la TF `map→odom` + `/amcl_pose`.

**Verificar que localiza bien:** `ros2 run tf2_ros tf2_echo map ego_racecar/odom`
→ si es ≈ identidad, AMCL coincide con el ground-truth.

Launch args útiles: `use_rviz:=false` (headless), `map_yaml:=…` (otro mapa;
p. ej. `maps/slam_saopaulo_v3.yaml` para el pipeline auto-consistente),
`scan_rate_hz:=40`.

## 5. Parámetros que puedes tunear (`config/amcl_params.yaml`)

| Parámetro | Valor | Qué hace / en qué afecta |
|---|---|---|
| `alpha1..alpha5` | **0.01** | Ruido asumido de la odometría. **LA palanca:** la odom del sim es exacta → bajarlo pega la nube a la odom. En hardware, subirlo |
| `update_min_d`/`update_min_a` | 0.05 / 0.05 | Cuánto debe moverse para actualizar el filtro. Bajar = corrige más seguido |
| `transform_tolerance` | 0.3 | Cuánto "hacia el futuro" se estampa la TF publicada |
| `min_particles`/`max_particles` | 300 / 1000 | Tamaño de la nube. Menos = más CPU libre para corregir |
| `scan_topic` | `/scan_amcl` | Scan ya throttleado (NO `/scan` crudo, ver §7) |
| `laser_max_range` | 30.0 | Coincide con el `range_max` del LiDAR |
| `set_initial_pose`+`initial_pose` | `(0,0,-1.3177)` | Arranca localizado sin RViz. El `yaw` DEBE coincidir con `stheta` de `sim.yaml` |

Tras cambiarlos: `colcon build`.

## 6. Resultado esperado

Con el **perfil racing**, error de pose (TF `map→base_link` vs ground-truth,
50 Hz) **media 0.11 m** a `speed_scale=0.6` (v hasta 4.8 m/s), con 10+ vueltas
de Pure Pursuit sin chocar. La horquilla —el peor punto— pasó de **2.24 m a
0.10 m**.

| Zona | ANTES (config genérica, ss=0.3) | DESPUÉS (perfil racing, ss=0.6) |
|---|---|---|
| TOTAL media | 1.25 m | **0.11 m** |
| Horquilla media / máx | 2.24 / 3.24 m | **0.10 / 0.23 m** |

**Techo medido:** a ~10 m/s AMCL ya no da la precisión (el coche recorre
0.2-0.25 m entre scans) y es un límite **estructural**. Punto rápido validado:
**9 m/s** (`saopaulo_fast`, 41.5 s limpio). Los 10 m/s esperan un localizador
mejor (particle filter GPU / fusión).

## 7. Problemas que tuvimos y cómo se resolvieron

| Problema | Causa | Solución |
|---|---|---|
| AMCL divergía (pose a ~47 m) | `gym_bridge` publica `/scan` a **250 Hz** → el message filter de AMCL se satura ("queue full") | Nodo propio **`scan_throttle`**: reentrega `/scan` → `/scan_amcl` a 40 Hz |
| Con `tf`, Pure Pursuit iba 1-2 m atrasado en curva | AMCL tuneado para odom **ruidosa** (`alphas=0.2`); esparcía la nube con ruido artificial | **Perfil racing** `alphas 0.2→0.01` (mejora ~×10) + `update_min 0.05` |
| ¿Subir el scan a 80 Hz mejora la pose? | Hipótesis razonable | **Medido: NO** (error idéntico) y 1/2 corridas divergió → 40 Hz se queda |
| `scan_throttle` muere con segfault esporádico | Bug rclpy/DDS; deja a AMCL sin scans | `respawn=True` en el launch (lo relanza solo) |

> Requiere los 2 parches de `gym_bridge.py` de SLAM (no se versionan;
> `docs/00-setup.md` §3bis). `map_server` y `amcl` son nodos *lifecycle*: los
> arranca el `lifecycle_manager` (`autostart: true`).

## 8. Glosario

- **AMCL:** Adaptive Monte Carlo Localization = filtro de partículas adaptativo.
- **Partícula:** una hipótesis de dónde puede estar el coche, con un peso.
- **Ray-casting:** simular qué vería el LiDAR desde una partícula, para
  compararlo con el scan real.
- **Resampling:** rehacer la nube favoreciendo las partículas de más peso.
- **Occupancy grid / `/map`:** el mapa fijo contra el que se localiza.
- **Kidnapped robot:** que "teletransporten" al robot; AMCL se recupera
  inyectando partículas aleatorias (desactivado en sim, TODO-hardware).
- **Nodo lifecycle:** nodo ROS 2 con estados (unconfigured→active) gestionado
  por un `lifecycle_manager`.

## 9. Cierre

La localización está **completa** con el perfil racing (0.11 m). Con mapa + pose
en tiempo real, el siguiente eslabón es el **Path planning**
(`src/path_planning/`): trazar la mejor línea de carrera (raceline) sobre el
mapa, que luego seguirán Pure Pursuit y el MPC.
