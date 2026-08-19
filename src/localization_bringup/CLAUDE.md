# localization_bringup — memoria del paquete

## Perfil racing (6 jul 2026) — AMCL ya NO es el cuello de botella
La config está tuneada para racing en el sim: `alpha1..5=0.01` (la odom del sim
es ground-truth; con 0.2 la pose publicada iba 1-2 m atrás en curva),
`update_min_d/a=0.05`, `transform_tolerance=0.3`, partículas 300/1000. Resultado
medido: error de pose (TF map→base_link vs odom GT) media 0.11 m, horquilla
máx 0.23 m a ss=0.6 — antes: horquilla media 2.24 m a ss=0.3. Pure Pursuit
completa 10+ vueltas con `pose_source:=tf`. Detalle en README §Perfil racing.
**OJO hardware real:** odom deriva → subir alphas de nuevo y re-medir.

## Revisión 7 jul 2026 — defaults corregidos + scan_rate confirmado
- **Default de `map_yaml` → `SaoPaulo_map` (GT), antes `slam_saopaulo`**: todo
  lo validado corre contra el GT; el default viejo apuntaba a un mapa nunca
  usado ("default engañoso"). Alternativa auto-consistente:
  `slam_saopaulo_v3.yaml` (0.36 m vs 0.11 m del GT). Bare launch verificado
  (map→odom = identidad).
- **`scan_rate_hz` 40 confirmado óptimo**: subir a 80 Hz NO baja el error de
  pose (p95 0.21 m en ambos a ss=0.8) y 1 de 2 corridas divergió. 40 se queda.
- **`recovery_alpha` anotado como TODO-hardware** (0 en el sim; en el real hay
  que activarlo para recuperarse de perder la localización).

## Comando exacto para correrlo
```bash
ros2 launch localization_bringup localization_launch.py       # sim + amcl + map_server + throttle (+ rviz)
ros2 run ftg_tuned ftg_node                                    # en otra terminal, para recorrer la pista
# headless: ros2 launch localization_bringup localization_launch.py use_rviz:=false
```

## Gotchas (ya resueltos, no re-descubrir)

1. **AMCL diverge por el scan a 250 Hz — LA causa raíz.** `gym_bridge` publica
   `/scan` a 250 Hz (paso de física del sim). El message filter de AMCL se
   satura ("discarding message because the queue is full") y la localización
   se va a decenas de metros del ground truth. `nav2_amcl` NO tiene
   `throttle_scans` y `topic_tools` no está instalado. **Fix:** el nodo propio
   `scan_throttle` reentrega `/scan` → `/scan_amcl` a 40 Hz; AMCL consume
   `/scan_amcl`. Si AMCL vuelve a divergir, revisar primero que `scan_throttle`
   esté corriendo y que `scan_topic: /scan_amcl` en `amcl_params.yaml`.

2. **map ≈ odom → pose inicial (0, 0, yaw=-1.3177).** El sim arranca el coche
   en `sx=sy=0`, `stheta=-1.3177` (la tangente real de la pista en el arranque;
   ver gotcha #7 de slam_bringup — antes era 0.0 y causaba un giro brusco). El
   frame `map` (GT o SLAM) queda anclado en esa pose de arranque, así que
   `set_initial_pose: true` con `initial_pose (x=0, y=0, yaw=-1.3177)` deja a
   AMCL localizado de entrada y `map→odom ≈ identidad`. **El `yaw` del
   `initial_pose` debe coincidir con `stheta` de sim.yaml** — si cambia uno,
   actualizar el otro. Si se localiza contra otro punto de arranque, dar la
   pose inicial correcta (params o RViz "2D Pose Estimate").

3. **Verificación = `map→odom ≈ identidad`.** Como la odometría del sim es
   ground-truth, comparar `/amcl_pose` con `/ego_racecar/odom` en dos `--once`
   distintos es engañoso (el coche se mueve entre lecturas). Lo correcto:
   `ros2 run tf2_ros tf2_echo map ego_racecar/odom` — si es ≈0, AMCL coincide
   con el ground truth.

4. **Reuso del parche de gym_bridge (SLAM).** AMCL publica `map→odom` igual que
   `slam_toolbox`; el árbol TF es el mismo. Los dos parches a `gym_bridge.py`
   (frame `map`→`ego_racecar/odom`; `angle_increment`) son requisito y NO se
   versionan (sim gitignorado). Ver `docs/00-setup.md` §3bis.

5. **Nodos lifecycle.** `map_server` y `amcl` no arrancan solos: necesitan el
   `lifecycle_manager` con `autostart: true` gestionándolos. Si `/amcl_pose`
   no publica, revisar `ros2 lifecycle get /amcl` (debe ser `active`).

6. **QoS de `/map`:** `transient_local`. Para depurar con `ros2 topic echo /map`
   pasar `--qos-durability transient_local --qos-reliability reliable`.

7. **`ros2 topic hz` se cuelga en esta máquina** (problema DDS/shm, sale
   "Terminated"). Verificar tasas por otros medios (logs del nodo, tf2_echo).

8. **`scan_throttle` puede morir con segfault esporádico** (rclpy/DDS, exit
   -11). Si muere, AMCL se queda sin scans y `map→odom` se congela → el control
   pierde la pose (con transform_tolerance 0.3 el lookup falla en ~0.3 s). El
   launch lo relanza solo (`respawn=True`); si un run se comporta raro, grep
   "process has died" en el log del launch.

9. **Los alphas NO son un parámetro genérico a dejar en default.** Codifican
   cuánto ruido tiene TU odometría. Sim = ground-truth → 0.01. Se midió con un
   probe (TF vs odom a 50 Hz): bajar alphas 0.2→0.01 redujo el error en la
   horquilla de 2.24 m a 0.07 m. Es la lección central del hito Localización.

## Techo de AMCL medido (7 jul tarde): 9 m/s SÍ (41.5 s), 10 m/s NO
Con pose GT el MPC hace 38.3 s → AMCL es el bloqueador a 10 m/s y es
ESTRUCTURAL (probado con amcl_params_hispeed.yaml: update 0.02/beams 90/
scan 50 — mejora la pose en crucero pero el pico 0.66 m a 10 m/s mata).
Punto validado: saopaulo_fast (9 m/s) = 41.5 s con el perfil racing normal.
Desbloqueo futuro: f1tenth particle_filter (GPU) o fusión. OJO ambiental:
navegador pesado = fallos desde t=0 (Brave 180% CPU vivido).
