# TP1 - Teach & Repeat, Grupo 4

ROS 2 Jazzy, TurtleBot3 Burger, Gazebo Harmonic y Pixi/RoboStack en macOS ARM64.
Código: `ros_ws/src/Grupo4/tp1_teach_repeat/`.

## Compilación

Ejecutar desde la raíz **Advance-Robotics-tp1**, en cada terminal:

```bash
cd /Users/matias/5-TO/Robotica-Avanzada/Advance-Robotics-tp1
pixi run --frozen build
pixi run --frozen test
```

`.pixi` enlaza al entorno compartido `../ros_ws/.pixi`: no borrarlo ni reinstalarlo.
La activación carga ROS, el underlay de TurtleBot3 y el overlay del TP. Se compila
con `colcon build`, **sin `--symlink-install`**. Recompilar tras modificar código,
launch o configuración: la instalación contiene copias. Open3D 0.19 y NumPy se
obtienen de Pixi; fuera de este entorno también deben instalarse.
La tarea `test` desactiva plugins externos de pytest porque el `launch_testing`
instalado es incompatible con pytest 9. No modifica las dependencias del entorno.

## Simulación: Teach

Iniciar **una sola simulación**. Si Gazebo ya está abierto en una ventana del TP,
usar esa instancia y omitir `pixi run sim`. Dos servidores/bridges sobre los mismos
tópicos mezclan relojes y producen `Moved backwards in time` y regresiones de encoders.
El launch incluye un bloqueo para rechazar una segunda instancia del mismo TP.
Si ya ocurrió el conflicto, detener ambas simulaciones y Teach/Repeat, iniciar una
sola simulación y comenzar Teach con una ruta nueva; conservar la anterior para revisión.

En ventanas separadas de tmux, desde la raíz del repositorio:

```bash
# Simulación: robot, bridge y robot_state_publisher
pixi run --frozen sim

# Aprendizaje: elegir encoders, lidar o mixed
pixi run --frozen teach mode:=mixed route_file:=routes/mi_ruta.json

# Teleoperación
pixi run --frozen teleop

# Visualización
pixi run --frozen rviz
```

Teclas: `w/s` adelante/atrás, `a/d` giro, espacio o `x` detiene y `q` sale.
Mantener/repetir tecla: un timeout de 0.4 s detiene el movimiento.
No ejecutar teleoperación y Repeat simultáneamente sobre `/cmd_vel`.

Teach publica `/rel_odom` y guarda un keyframe al superar **0.20 m o 0.25 rad**
respecto al último. Cada keyframe almacena pose, transformación al anterior,
timestamp, escaneo crudo (rangos no finitos como `null`) y nube en su marco base.
El mapa de referencia es esta secuencia de escaneos y poses; se visualiza como nube
de puntos, no como grilla de ocupación. También se guarda la trayectoria Teach.

Para conservar el punto final, detener el robot y ejecutar:

```bash
pixi run --frozen ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
```

Terminar Teach con `Ctrl+C`. También se guarda al salir y en cada keyframe mediante
reemplazo atómico. Una ruta existente se rechaza; para reemplazarla deliberadamente
agregar `overwrite:=true`, o elegir otro archivo.
Encoders funciona sin LiDAR. Si hay scans se almacenan para visualizar y permitir
Repeat con LiDAR. Repeat con LiDAR/mixto rechaza rutas con keyframes sin nube válida.

## Simulación: Repeat

1. Detener teleoperación y Teach.
2. Volver a la **misma pose física inicial de Teach**. Puede reiniciarse Gazebo
   (detener y volver a lanzar `sim`) antes de iniciar Repeat.
3. Ejecutar:

```bash
pixi run --frozen repeat mode:=mixed route_file:=routes/mi_ruta.json
```

Se recorren los keyframes en orden al cumplir posición **0.035 m** y orientación
**0.06 rad**. `/tnr/done=true` indica finalización; el controlador mantiene cero.
Se admiten segmentos en reversa y giros en el lugar. No hay planificación ni evasión
de obstáculos: la ruta enseñada debe estar libre.

Si se conoce un desplazamiento inicial respecto al origen de Teach:

```bash
pixi run --frozen repeat mode:=mixed route_file:=routes/mi_ruta.json \
  initial_x:=0.05 initial_y:=0.0 initial_yaw:=0.02
```

Es una semilla local, no relocalización global. Encoders depende enteramente de esa
colocación/medición. ICP necesita una semilla cercana y geometría no ambigua.

## Localización y control

El mismo ejecutable `localization` se usa en ambas etapas:

| Modo | Fuente de pose | Método |
|---|---|---|
| `encoders` | `/joint_states` | Integra posiciones por nombre de rueda con la exponencial exacta del movimiento diferencial. Los scans solo aportan mapa. |
| `lidar` | `/scan` | ICP Open3D actual → anterior, sin suscripción a encoders. En Repeat además alinea contra el escaneo Teach activo. |
| `mixed` | Ambas | Predice a frecuencia de encoders y corrige con cada ICP válido. En Repeat además alinea contra Teach. |

Las nubes se filtran y se llevan del marco del scan a `base_link` mediante TF.
Los puntos tienen z=0 y el resultado de Open3D se proyecta a SE(2). Se rechazan
pocos puntos, fitness bajo, RMSE alto, reflexiones fuera del plano y saltos no
plausibles. Ante rechazo, LiDAR conserva la última estimación válida y mixto
continúa con encoders.

Mixto interpola ruedas al tiempo del scan, actualiza
`T_odom_wheel = T_odom_base(scan) * inverse(T_wheel_base(scan))` y aplica esa
corrección a las ruedas actuales. Conserva así el avance posterior a la medición.
Un scan adelantado espera el siguiente encoder; no se extrapola al futuro.

`controller` recibe `/tnr/setpoint` (`PoseStamped` en `base_link`), aplica control
polar, saturaciones y alineación final. Valida `k_rho > 0`, `k_beta < 0`,
`k_alpha > k_rho`. Su watchdog de pared detiene ante setpoints viejos o ausencia
de mensajes durante 0.6 s, incluso si se pausa `/clock`. El driver real debe tener
además timeout propio ante terminación abrupta del proceso o pérdida de red.

## Tópicos y TF

| Tópico | Tipo / finalidad |
|---|---|
| `/joint_states`, `/scan` | `sensor_msgs/JointState` (radianes), `sensor_msgs/LaserScan` |
| `/tnr/odom` | `nav_msgs/Odometry`, pose en `odom`; no usa la odometría del simulador |
| `/rel_odom` | `nav_msgs/Odometry`, pose de `base_link` en `reference` |
| `/tnr/setpoint` | `geometry_msgs/PoseStamped`, objetivo relativo en Repeat |
| `/cmd_vel` | `geometry_msgs/TwistStamped` por defecto, o `Twist` |
| `/tnr/teach_path`, `/tnr/repeat_path` | `nav_msgs/Path` |
| `/tnr/keyframes`, `/tnr/reference_map` | `visualization_msgs/MarkerArray`, `sensor_msgs/PointCloud2` |
| `/tnr/done`, `/tnr/status` | `std_msgs/Bool`, `std_msgs/String` |

```text
odom
└── reference
    └── base_link
        ├── base_scan, imu_link, ruedas… (robot_state_publisher)
        ├── map_current_keyframe       (solo Repeat)
        │   └── map_next_keyframe      (solo Repeat)
        └── setpoint                  (solo Repeat)
```

`T_parent_child` expresa la pose del hijo en el padre. Por eso
`base_link -> map_current_keyframe` es la inversa de la pose del robot respecto
al keyframe activo. `reference` es el último keyframe en Teach y el activo en Repeat.
Todos los TF dinámicos usan timestamps de medición, sin adelantarlos. Encoders
con tiempo repetido/regresivo se descartan. Si se reinicia `/clock`, reiniciar
Teach/Repeat. Los launch usan `use_sim_time:=true` por defecto.

El launch propio de simulación evita TF de Gazebo y elimina **en memoria**
`base_footprint -> base_link` de la descripción original: `base_link` tiene un solo
padre. No modifica el underlay. No ejecutar el launch original de TurtleBot3 y
`pixi run sim` simultáneamente.
RViz muestra modelo, TF, scans, mapa, keyframes y ambas trayectorias; mapa y paths
usan QoS Transient Local para poder conectar RViz después.

## Parámetros y robot real

Parámetros: `config/params.yaml`. Para personalizar, copiar solo ese YAML a un
archivo propio y usar `params_file:=/ruta/absoluta/robot.yaml`. Mantener iguales las
tolerancias de localización/control. Calibrar radio/separación y extrínseca del
láser. `joint_angles_wrapped: true` solo si el driver envía ángulos envueltos;
por defecto las posiciones de ruedas se consideran continuas.

En el robot real:

1. Iniciar drivers con `/joint_states`, `/scan` y TF `base_link -> <frame del scan>`.
   Ajustar los remapeos de los launch si sus tópicos difieren. No iniciar Gazebo.
2. Desactivar TF odométrica del driver y adaptar URDF/robot_state_publisher para
   que `base_link` sea raíz, sin `base_footprint -> base_link`. Debe existir un solo
   publicador del padre dinámico de `base_link`.
3. Configurar el mismo `ROS_DOMAIN_ID`, middleware y red. La activación local usa
   descubrimiento `LOCALHOST`: para otra máquina usar `SUBNET` dentro de Pixi.
4. Usar reloj de sistema y el tipo de velocidad que espere el driver:

```bash
# Ejemplo para drivers remotos que esperan Twist
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ros2 launch tp1_teach_repeat teach.launch.py use_sim_time:=false \
  mode:=mixed route_file:=routes/real.json

pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ros2 run tp1_teach_repeat teleop --ros-args \
  -p use_sim_time:=false -p publish_stamped:=false

# Guardar, terminar Teach y volver a la pose inicial; luego:
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ros2 launch tp1_teach_repeat repeat.launch.py use_sim_time:=false \
  mode:=mixed route_file:=routes/real.json publish_stamped:=false
```

Si el driver espera `TwistStamped`, mantener `publish_stamped:=true`. En RViz real
usar `rviz2 -d <ruta a tp1_view.rviz> --ros-args -p use_sim_time:=false` dentro del
entorno y con el mismo descubrimiento. La demostración física requiere el robot.

## Validación reproducible

Con **solo la simulación** iniciada, sin otro Teach/Repeat ni teleoperación:

```bash
pixi run --frozen python scripts/validate_gazebo.py --output validation/gazebo
pixi run --frozen python scripts/plot_validation.py validation/gazebo
```

El script imprime comandos, espera sensores de Gazebo, enseña avance y giro, guarda,
reposiciona `burger` mediante `/world/default/set_pose` y ejecuta Repeat para los
**tres modos**. No reinicia el reloj ni publica sensores sintéticos. Exige terminar
con error estimado menor a 5 cm y 0.08 rad, comprueba las cinco TF y ausencia de
regresiones. **Solo para simulación:** el script mueve el robot.
Guarda rutas, trayectorias y `results.json`. Los errores son respecto a la pose
estimada, no ground truth. La comparación de precisión física y presentación del
punto 3.5 requieren una referencia externa o las poses verdaderas de Gazebo.

El descubrimiento DDS puede tardar varios segundos:

```bash
pixi run --frozen ros2 topic list --no-daemon --spin-time 5
pixi run --frozen ros2 topic echo /scan sensor_msgs/msg/LaserScan --once
```

Ante advertencias Ogre2 comprobar rangos válidos en `/scan`; la presencia del
bridge por sí sola no garantiza sensores.

## Alcance y antecedentes

`docs/analisis_tp1.md` relaciona consigna y clases 1–4. Se implementan secciones
1–2, entregables 3.1–3.4 y los tres modos de la sección 4. La presentación oral y
la demostración física son entregas adicionales al código.
