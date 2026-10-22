# TP1 — Teach & Repeat · Grupo 4

Sistema ROS 2 Jazzy para TurtleBot3 Burger con localización por encoders, LiDAR o modo mixto.
Entorno probado: macOS ARM64, Pixi/RoboStack y Gazebo Harmonic.
Código: `ros_ws/src/Grupo4/tp1_teach_repeat/`.

## 1. Compilar y configurar

Ejecutar todos los comandos desde la raíz de `Advance-Robotics-tp1`.
Se requieren Pixi y el entorno compartido de la materia, con ROS 2 Jazzy,
Gazebo Harmonic, TurtleBot3 y Open3D instalados.

```bash
pixi run --frozen build
pixi run --frozen test
```

- `.pixi` enlaza a `../ros_ws/.pixi`: no borrarlo ni reinstalarlo.
- La activación carga el underlay de TurtleBot3 y el overlay local automáticamente.
- Recompilar después de modificar código, launch o configuración. No usar `--symlink-install`.
- Parámetros: [`config/params.yaml`](ros_ws/src/Grupo4/tp1_teach_repeat/config/params.yaml).

El argumento `mode` selecciona el mismo sistema de localización para Teach y Repeat:

| Argumento | Fuente de localización |
|---|---|
| `mode:=encoders` | Solo posiciones de ruedas en `/joint_states`. |
| `mode:=lidar` | Solo `/scan`, mediante ICP con Open3D. |
| `mode:=mixed` | Predicción con encoders y corrección con cada escaneo LiDAR válido. |

En encoders, los escaneos se usan para guardar y mostrar el mapa y calcular el
indicador de alineación; no corrigen la pose.

## 2. Simulación: enseñar una ruta

Usar cinco paneles de tmux. Ejecutar cada comando en su panel:

**Panel 1 — Gazebo.** Si ya está abierto con este proyecto, conservar esa instancia.

```bash
pixi run --frozen sim
```

**Panel 2 — Teach.** Elegir un archivo nuevo para conservar recorridos anteriores.

```bash
pixi run --frozen teach mode:=mixed route_file:=routes/mi_recorrido.json
```

**Panel 3 — Teleoperación.**

```bash
pixi run --frozen teleop
```

**Panel 4 — RViz.**

```bash
pixi run --frozen rviz
```

En el panel 3, usar `w/s` para avanzar/retroceder, `a/d` para girar y espacio para
parar. Mantener o repetir la tecla. Enseñar una ruta libre de obstáculos.
Teach crea keyframes cada 0.20 m o 0.25 rad; deben aparecer varios en el panel 2.

Al terminar, detener con espacio y salir de teleop con `q`.
En el **panel 5**, guardar el punto final:

```bash
pixi run --frozen ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
```

Esperar `success: true` y detener Teach con `Ctrl+C` en el panel 2.
El JSON contiene keyframes, escaneos y trayectoria. Para reemplazar una ruta
existente deliberadamente, agregar `overwrite:=true` al comando Teach.

## 3. Simulación: repetir y comparar modos

Con Teach y teleoperación detenidos, volver a la misma posición y orientación
física donde comenzó Teach. Si la ruta comenzó en el inicio estándar de Gazebo
(`x=-2.0`, `y=-0.5`, orientación cero), ejecutar en el **panel 5**:

```bash
pixi run --frozen gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "burger" position: {x: -2.0 y: -0.5 z: 0.01} orientation: {w: 1.0}'
```

Esperar un segundo. En el **panel 2**, ejecutar uno de estos comandos:

```bash
pixi run --frozen repeat mode:=encoders route_file:=routes/mi_recorrido.json
```

```bash
pixi run --frozen repeat mode:=lidar route_file:=routes/mi_recorrido.json
```

```bash
pixi run --frozen repeat mode:=mixed route_file:=routes/mi_recorrido.json
```

Probar un modo por vez. Antes de cambiar, detener Repeat con `Ctrl+C` y devolver
el robot al inicio. Gazebo y RViz pueden seguir abiertos. No ejecutar teleop durante Repeat.

Se puede reutilizar el mismo archivo en los tres modos: debe tener al menos dos
keyframes y, para LiDAR/mixto, al menos 20 puntos por keyframe. El argumento `mode`
selecciona la localización de Repeat aunque el archivo se haya grabado en otro modo.
Para evaluar también Teach, grabar una ruta con cada modo.

**Resultado esperado:** el robot sigue las referencias y se detiene al finalizar;
el panel 2 muestra `REPEAT COMPLETADO`. Desde el panel 5:

```bash
pixi run --frozen ros2 param get /tnr_localization mode
pixi run --frozen ros2 topic echo /tnr/done --once --qos-durability transient_local
```

Deben aparecer el modo seleccionado y `data: true` al completar el recorrido.

## 4. Qué ver en RViz

- Cartel con el método activo: `ENCODERS`, `LIDAR` o `MIXTO`.
- Robot, mapa de paredes y escaneo LiDAR actual.
- Teach naranja y Repeat turquesa; referencia activa verde y objetivo magenta.
- `RMSE mapa` y `Overlap`: concordancia entre escaneo y referencia; no son error real de posición.
- Para inspeccionar el árbol TF, activar el display `TF` o ejecutar `pixi run --frozen rviz-tf`.

Teach publica `/rel_odom` y `odom → reference → base_link`.
Repeat agrega `base_link → map_current_keyframe → map_next_keyframe` y
`base_link → setpoint`. Los launch de simulación usan `use_sim_time:=true`
y `/cmd_vel` de tipo `TwistStamped`.

## 5. Robot real

La prueba física está pendiente. Estos pasos requieren los drivers del robot:

1. Publicar `/joint_states`, `/scan` y la TF estática desde `base_link` al láser.
2. Evitar otro publicador de TF odométrica hacia `base_link`; adaptar el URDF si usa `base_footprint`.
3. Usar el mismo `ROS_DOMAIN_ID` y descubrimiento por red en todas las terminales.
4. Configurar radio/separación de ruedas, nombres de articulaciones y extrínseca del láser.
   Si los tópicos difieren, ajustar remapeos en `tp1_teach_repeat/launching.py` y los drivers.
5. No iniciar Gazebo. Usar `use_sim_time:=false` y el tipo de `/cmd_vel` que espere el driver.

Crear una configuración para el robot desde la raíz del repositorio:

```bash
cp ros_ws/src/Grupo4/tp1_teach_repeat/config/params.yaml robot.yaml
```

Editar `robot.yaml` y usarlo en Teach y Repeat. En paneles separados:

```bash
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros2 launch tp1_teach_repeat teach.launch.py use_sim_time:=false mode:=mixed route_file:=routes/robot_real.json params_file:="$PWD/robot.yaml"
```

```bash
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros2 run tp1_teach_repeat teleop --ros-args -p use_sim_time:=false -p publish_stamped:=false
```

```bash
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET rviz2 -d ros_ws/install/tp1_teach_repeat/share/tp1_teach_repeat/config/tp1_view.rviz --ros-args -p use_sim_time:=false
```

Detener el robot y salir de teleop. Guardar desde otro panel:

```bash
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
```

Esperar `success: true`, detener Teach y colocar físicamente el robot en el inicio.
Ejecutar Repeat:

```bash
pixi run --frozen env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ros2 launch tp1_teach_repeat repeat.launch.py use_sim_time:=false mode:=mixed route_file:=routes/robot_real.json params_file:="$PWD/robot.yaml" publish_stamped:=false
```

Estos ejemplos usan `Twist`. Si el driver espera `TwistStamped`, usar
`publish_stamped:=true` tanto en teleop como en Repeat.

## 6. Validación automática y resolución de problemas

Con solo Gazebo abierto, sin Teach, Repeat ni teleop:

```bash
pixi run --frozen python scripts/validate_gazebo.py --modes encoders lidar mixed --output validation/nuevo_ensayo
pixi run --frozen python scripts/plot_validation.py validation/nuevo_ensayo
```

La prueba mueve el robot, ejecuta Teach/Repeat por modo y guarda rutas, resultados
y gráficas. Elegir una carpeta de salida nueva. Los errores reportados son de la
pose estimada; no sustituyen una medición de precisión física.

| Problema | Acción |
|---|---|
| Ruta existente al iniciar Teach | Elegir otro archivo o usar `overwrite:=true`. |
| Repeat no inicia por ruta de un solo keyframe | Enseñar un recorrido con varios keyframes y guardar el final. |
| Repeat inicia lejos de la ruta | Reponer posición **y orientación** iniciales de Teach. |
| Mensajes de tiempo hacia atrás | Detener simulaciones duplicadas y Teach/Repeat; iniciar una sola simulación y una nueva sesión Teach. |
| No aparecen sensores | Comprobar `/scan` y `/joint_states` con `pixi run --frozen ros2 topic list`. |
| No aparece el cartel del método | Activar `Setpoint y alineacion`; recompilar y reiniciar Teach/Repeat. |

Detalle: [referencia técnica](docs/referencia_tecnica.md),
[verificación de los tres modos](docs/verificacion_tres_modos.md) y
[relación con la consigna](docs/analisis_tp1.md).
