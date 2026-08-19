# pure_pursuit — memoria del paquete

## Estado: completa vueltas con `odom` ✅ Y con `tf` (AMCL) ✅ (6 jul 2026)
Con la raceline del mapa **ground-truth** (`saopaulo_gt.csv`) completa vueltas
limpias en ambas configs. Los tres problemas resueltos, en orden: (1) **causa
raíz** = mapa SLAM desfasado 2-3 m → raceline desde `SaoPaulo_map` (opción C);
(2) el controlador **cortaba curvas** → búsqueda local + lookahead corto;
(3) **Etapa B** = AMCL iba 1-2 m atrasado en curva → **perfil racing** en
localization_bringup (alphas 0.2→0.01 fue LA palanca: la odom del sim es
ground-truth y AMCL le inyectaba ruido artificial; + update_min 0.05,
transform_tolerance 0.3). Resultado: error de pose horquilla 2.24→0.10 m,
10+ vueltas a ss=0.6 sin chocar. Runbook completo en README §"Etapa B".

**Fase C (límite de velocidad, cerrada):** el perfil de velocidad ganó el
**círculo de fricción** (velocity_profile.py de path_planning) porque PP
aceleraba en plena salida de curva (27,57) y chocaba a ss≥0.8. Con el fix:
**ss=0.8 = máximo 100% fiable** (cero atascos); 0.9 choca ~1/3 en (27,57);
1.0 = límite claro. NO es AMCL (err 0.02-0.15 m al chocar) — es el límite
estructural del controlador geométrico (sin anticipación). Baseline para el
MPC: ss=0.8, vuelta limpia ~50-57 s, teórico a ss=1.0 = 46 s. README §Fase C.

## Comandos
```bash
# Etapa B (cadena completa con AMCL) — runbook completo en README §"Etapa B":
ros2 launch localization_bringup localization_launch.py use_rviz:=false \
  map_yaml:=/home/hectorros/roboracer-f1tenth/maps/SaoPaulo_map.yaml
# (+ raceline_publisher + rviz2 -d .../pure_pursuit.rviz para ver)
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  --params-file install/pure_pursuit/share/pure_pursuit/config/pure_pursuit_params.yaml \
  -p csv_path:=install/path_planning/share/path_planning/racelines/saopaulo_gt.csv \
  -p pose_source:=tf -p speed_scale:=0.6

# Config mínima sin localización (aísla el controlador, pose ground-truth):
ros2 run f1tenth_gym_ros gym_bridge --ros-args -r __node:=bridge \
  --params-file install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  --params-file install/pure_pursuit/share/pure_pursuit/config/pure_pursuit_params.yaml \
  -p csv_path:=install/path_planning/share/path_planning/racelines/saopaulo_gt.csv \
  -p pose_source:=odom -p speed_scale:=0.6
```

## Cómo funciona (resumen)
`pure_pursuit_node.py`: carga la raceline CSV, obtiene la pose (TF
`map→base_link` de AMCL, o `/ego_racecar/odom`), busca el lookahead point,
lo pasa al marco del auto, `γ=2·y_body/L²`, `δ=atan(W·γ)`, lee `v` del
waypoint más cercano, publica `/drive`. Geometría verificada correcta.

## Gotchas (ya vividos, no re-descubrir)

1. **RViz desde `pure_pursuit_launch.py` — RESUELTO (7 jul).** Eran DOS
   problemas en capas: (a) el `IncludeLaunchDescription` de localization pasaba
   `use_rviz: 'false'` y ese `SetLaunchConfiguration` **se filtraba al scope
   del padre**, sobrescribiendo su `use_rviz` → el `rviz_node` (evaluado
   después) veía 'false' y nunca se creaba (por eso NI aparecía en el log).
   Fix: envolver el include en `GroupAction(scoped=True)`. (b) Ya spawneando,
   rviz2 **segfaultaba (-11)** en la init de OpenGL por contención de GPU al
   arrancar junto a gym_bridge+AMCL (standalone nunca falla). Fix: `TimerAction
   (period=6.0)` para arrancarlo cuando el stack ya está levantado. Verificado:
   rviz vive y el stack completo sano. El workaround viejo (rviz aparte) sigue
   valiendo si se quiere.

2. **En modo `odom`, AMCL no publica `map→odom` de forma fiable** dentro del
   launch bundleado (activa pero no procesa scans → `map` frame inválido en
   RViz → el auto sale como cuadrado blanco, Fixed Frame en rojo).
   **Workaround para ver en RViz:** publicar TF estática identidad
   `ros2 run tf2_ros static_transform_publisher --frame-id map
   --child-frame-id ego_racecar/odom` (válido porque map≈odom en el sim).
   OJO: quitarla si se usa `pose_source:=tf` (conflicto con AMCL).
   **Pendiente:** entender por qué AMCL no procesa scans en el launch
   bundleado (funciona en localization_launch aislado).

3. **Reset del auto al arranque** (para pruebas repetibles): publicar en
   `/initialpose` (PoseWithCovarianceStamped, frame map) con
   qz=-0.6142 qw=0.7892 (= yaw -1.3177). gym_bridge lo resetea.

4. **Búsqueda del nearest ahora es LOCAL** (ventana `nearest_search_back/_fwd`
   alrededor del último índice; global solo en el 1er ciclo). RESUELVE el salto
   al ramal de vuelta en horquillas/eses plegadas — era una de las causas del
   fallo en curvas. Antes era `np.argmin` global sobre los 400 pts.

5. **Pose por TF**: lookup `map→ego_racecar/base_link`. AMCL da `map→odom`,
   gym_bridge da `odom→base_link`. Reusa el árbol de SLAM/localización.

6. **`speed_scale` y `lookahead_k` son launch args** (ParameterValue con
   value_type=float, o el launch falla con type error).

7. **RESUELTO (9 jul) — colisión de nombre `params_file` rompía AMCL en
   silencio.** `pure_pursuit_launch.py` declaraba su propio arg
   `params_file` (para `pure_pursuit_params.yaml`) con el MISMO NOMBRE que
   el `params_file` que declara `localization_launch.py` (para
   `amcl_params.yaml`). Con `GroupAction(scoped=True, forwarding=True)`
   sobre el include, el valor del padre se filtraba DENTRO del include y
   pisaba el default de `amcl_params.yaml` — `DeclareLaunchArgument` solo
   fija su default si el nombre no está YA seteado. Resultado: AMCL
   arrancaba con parámetros de fábrica (`base_frame_id=base_footprint`,
   `odom_frame_id=odom`, `scan_topic=/scan` sin throttle,
   `set_initial_pose=false`) — el árbol TF quedaba con `map→odom` (genérico,
   huérfano) desconectado de `ego_racecar/*`, y RViz mostraba **"Global
   Status: Fixed Frame" y "RobotModel: Status Error"** aunque el resto de la
   cadena estuviera sana. **Fix:** renombrar el arg del padre a
   `pp_params_file` (nombres únicos = colisión imposible). `forwarding=True`
   se mantiene (hace falta para que la sustitución `map_yaml` resuelva
   dentro del scope del grupo) — el riesgo real era el nombre duplicado, no
   el forwarding en sí. Verificar tras cualquier cambio a este launch:
   `ros2 param get /amcl base_frame_id` debe dar `ego_racecar/base_link`,
   NO `base_footprint`.

8. **RViz lento → `start_controller:=false`.** En máquinas donde RViz tarda
   en abrir, el controlador arrancaba de inmediato (no esperaba a RViz) y el
   usuario veía el auto ya en marcha/chocado al primer frame. Arg nuevo (9
   jul): `start_controller:=false` levanta sim+AMCL+raceline+RViz SIN el
   `pure_pursuit_node` — lanzarlo aparte por `ros2 run` una vez RViz ya está
   listo. Ver README §"Corrida directa".

## Dependencias de la cadena (todo debe estar bien aguas arriba)
mapa → raceline (path_planning) → pose → pure_pursuit. **Lección de esta
sesión:** si el mapa está desfasado respecto a lo que ve el LiDAR, el auto choca
aunque el controlador esté bien — y localizar (AMCL) contra ese mismo mapa
tampoco salva. Por eso se pasó a la raceline del mapa **ground-truth**
(`saopaulo_gt.csv`), que comparte frame con la física del sim. El mapa SLAM
(`slam_saopaulo`) se conserva como artefacto de los hitos SLAM/Localización,
pero NO se usa para el tracking fino de Pure Pursuit.
