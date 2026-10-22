# Referencia técnica — TP1, Grupo 4

Para compilar y ejecutar, seguir [README.md](../README.md).

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
| `/tnr/reference_grid`, `/tnr/reference_walls` | `nav_msgs/OccupancyGrid`, `visualization_msgs/MarkerArray`; mapa derivado para presentación |
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
Se integran todas las mediciones de encoders, pero TF/odometría se publican hasta
`publish_rate: 50.0` Hz y las trayectorias hasta 10 Hz para no saturar DDS/RViz
cuando Gazebo entrega articulaciones a 1 kHz. El punto final se guarda con su
timestamp de medición más reciente aunque no coincida con el último TF publicado.

El launch propio de simulación evita TF de Gazebo y elimina **en memoria**
`base_footprint -> base_link` de la descripción original: `base_link` tiene un solo
padre. No modifica el underlay. No ejecutar el launch original de TurtleBot3 y
`pixi run sim` simultáneamente.
RViz muestra modelo, TF, scans, mapa, keyframes y ambas trayectorias; mapa y paths
usan QoS Transient Local para poder conectar RViz después.

### Lectura de RViz y diagnóstico de alineación

- Vista cenital con fondo oscuro. `Mapa 2D - paredes` muestra celdas de 4 cm
  con impactos en al menos dos keyframes (uno si solo existe un keyframe).
  Reduce puntos aislados y densidad visual sin cambiar las poses ni los scans.
  `map_resolution` ajusta la resolución; la grilla se recalcula al agregar keyframes.
  `Nube original (diagnostico)` conserva los puntos originales y
  `Ocupacion (diagnostico)` permite inspeccionar celdas libres, ocupadas y desconocidas.
  Ambos están desactivados inicialmente. Esta presentación no corrige la deriva
  histórica ni se utiliza para controlar el robot o para ICP.
- Teach: naranja, banda de 7.5 cm. Repeat: turquesa, banda de 3.5 cm ligeramente
  elevada. Si coinciden, se ven el centro turquesa y los bordes naranjas; las
  coordenadas de los tópicos de trayectoria no se desplazan ni se suavizan.
- Keyframes: puntos para todas las referencias y etiquetas cada tres keyframes
  (`keyframe_label_stride`), más inicio, final y referencias seleccionadas.
  Verde para la referencia activa y magenta
  para la siguiente. Al terminar se etiqueta el destino `FINAL`, conservando la
  identificación de la referencia que realmente utiliza TF.
- Setpoint: flecha y disco magenta en la pose objetivo, línea desde el robot y
  etiqueta de destino. Display `Setpoint y alineacion`, tópico
  `/tnr/navigation_markers`. Se actualiza hasta 10 Hz y elimina marcadores de la
  etapa anterior al cambiar de ruta o de etapa.
- El mismo display muestra un cartel con solo el nombre del modo activo del nodo:
  `ENCODERS` en azul (solo ruedas), `LIDAR` en violeta (LiDAR/ICP) y `MIXTO`
  en verde (ruedas + LiDAR/ICP). Se ubica junto al recorrido y permite identificar
  el método en las grabaciones aunque ambos sigan prácticamente la misma ruta.
- El texto compacto `RMSE mapa` muestra el RMSE de distancias al punto más cercano de la
  referencia y el porcentaje de puntos dentro de `icp_distance`. Verde indica
  residual <= 6 cm y solapamiento >= 55%; naranja alerta o indica falta de scans
  recientes. Esto evalúa consistencia de nubes, **no error real de posición** ni
  implica que se ejecute ICP: también se muestra en modo `encoders`, donde el
  escaneo sirve para el mapa y el diagnóstico, pero no modifica la pose.
  `/tnr/alignment` publica los valores, el timestamp del scan y el tipo de referencia
  como JSON en `std_msgs/String`. P95 incluye también puntos fuera del umbral.
- TF y la flecha de odometría relativa quedan disponibles como displays opcionales,
  inicialmente apagados para evitar superponer ejes y flechas sobre el recorrido.
  Para mostrar el árbol TF ya activado en una vista separada, ejecutar
  `pixi run --frozen rviz-tf` (tras compilar). Esta vista conserva la configuración
  del mapa y las trayectorias; cerrar antes otra instancia de RViz si el equipo
  no puede renderizar ambas con fluidez.
- `LiDAR localizado` muestra `/tnr/localized_scan` en `odom`, transformado con la
  estimación real al timestamp del escaneo. En mixto, la TF histórica de ese
  instante puede contener la predicción anterior a la llegada del ICP; usar esa
  TF para dibujar el scan crudo producía una separación adicional respecto al mapa.
  La nube localizada evita esa mezcla temporal sin modificar TF pasadas ni
  desplazar puntos para forzar una coincidencia. `LiDAR crudo (TF historica)`
  conserva `/scan` como display opcional rojo para comparar ambas representaciones.

Para reducir la acumulación de deriva en nuevos Teach con LiDAR/mixto, cada scan
válido se alinea además con un submapa de los últimos tres keyframes, conservados
como anclas. Las correcciones están limitadas a 20 cm y 0.15 rad por actualización.
Si el ajuste no es válido se conserva la estimación scan-a-scan. El parámetro
`teach_submap_keyframes: 0` desactiva esta corrección para comparar. Encoders
continúa calculando la pose exclusivamente desde las ruedas.

Las nubes de mapas antiguos conservan la deriva con la que se enseñaron: este
cambio no reescribe rutas ni mueve los scans blancos solo para que coincidan.
Para evaluar la corrección, enseñar una ruta nueva. Para auditar una existente:

```bash
pixi run --frozen python scripts/audit_alignment.py routes/mi_ruta_corregida.json \
  --output validation/rviz_improvements/saved_route_alignment.json
pixi run --frozen ros2 topic echo /tnr/alignment std_msgs/msg/String
```

La auditoría compara cada scan con otros keyframes a menos de 0.8 m y reporta
residual/solapamiento. No puede determinar por sí sola la causa de una separación
ni sustituye una comparación contra ground truth. Si el solapamiento es bajo,
un RMSE pequeño de los pocos puntos coincidentes no demuestra buena localización.

