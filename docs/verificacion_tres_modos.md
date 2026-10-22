# Verificación de los tres modos en Gazebo

Trabajar desde `/Users/matias/5-TO/Robotica-Avanzada/Advance-Robotics-tp1`
en cada panel de la sesión tmux. Las grabaciones del laboratorio no intervienen.

## Ensayo automático reproducible

1. Dejar una sola simulación ejecutándose (`pixi run --frozen sim`). Si ya está
   abierta, utilizar esa misma. Detener Teach, Repeat y teleoperación anteriores
   con Ctrl+C en sus paneles; RViz puede quedar abierto.
2. En otro panel ejecutar:

```bash
pixi run --frozen build
pixi run --frozen test
pixi run --frozen python scripts/validate_gazebo.py --modes encoders lidar mixed --output validation/mi_verificacion
pixi run --frozen python scripts/plot_validation.py validation/mi_verificacion
```

El ensayo mueve el robot automáticamente: recta, giro y otra recta. Para cada modo
genera su propia ruta de seis keyframes, vuelve a colocar el robot en el inicio,
ejecuta Repeat, comprueba finalización y velocidad cero y cierra sus nodos.
La cantidad exacta de keyframes puede variar por el muestreo de los sensores.
La carpeta de salida conserva rutas, trayectorias, `results.json` y la comparación
`trajectories.png`. Elegir una carpeta distinta para conservar ensayos anteriores.
Las rutas en `routes/` no se sobrescriben.

Resultado esperado: tres líneas `RESULTADO`, una por modo, con
`completed: true`, `stopped_at_end: true` y `tf_regressions: []`.
El script falla si no termina dentro del tiempo límite, si el error final estimado
supera 5 cm o 0.08 rad, o si hay padres TF incorrectos. El controlador utiliza
tolerancias de llegada de 3.5 cm y 0.06 rad.

## Prueba manual visible en RViz

Usar cuatro paneles: simulación, Teach/Repeat, teleoperación y RViz.
Un quinto panel resulta útil para los servicios y las consultas.

1. Con Teach/Repeat y teleoperación detenidos, colocar el robot en la pose inicial:

```bash
pixi run --frozen gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "burger" position: {x: -2.0 y: -0.5 z: 0.01} orientation: {w: 1.0}'
```

2. En el panel Teach/Repeat elegir un modo e iniciar Teach. El archivo debe ser
   nuevo; para otro ensayo cambiar su nombre.

```bash
pixi run --frozen teach mode:=encoders route_file:=routes/prueba_encoders.json
```

3. En otros dos paneles ejecutar:

```bash
pixi run --frozen teleop
```

```bash
pixi run --frozen rviz
```

Con las teclas w/s y a/d enseñar un trayecto corto hacia adelante, un giro y otra
recta, sin tocar obstáculos. Parar con espacio y salir de teleop con q.
En la terminal de Teach deben aparecer nuevos `Keyframe`. En RViz deben verse
el robot, el recorrido naranja y las referencias.

4. Guardar el punto final desde el panel de consultas:

```bash
pixi run --frozen ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
```

Esperar `success: true` y detener Teach con Ctrl+C.
Volver a ejecutar el comando de reposicionamiento del paso 1; esperar un segundo.

5. En el panel Teach/Repeat ejecutar:

```bash
pixi run --frozen repeat mode:=encoders route_file:=routes/prueba_encoders.json
```

Debe moverse solo y recorrer las referencias en orden. RViz muestra el recorrido
Repeat turquesa, la referencia activa verde y el objetivo magenta. Al terminar,
la terminal muestra `REPEAT COMPLETADO`, el objetivo indica `OK` y el robot se detiene.

6. Mientras Repeat sigue abierto, consultar desde otro panel:

```bash
pixi run --frozen ros2 param get /tnr_localization mode
pixi run --frozen ros2 topic echo /tnr/done --once --qos-durability transient_local
pixi run --frozen ros2 topic echo /cmd_vel --once
pixi run --frozen ros2 node info /tnr_localization
```

Se espera el modo seleccionado, `data: true` y velocidades lineales y angulares
cero. En `node info`, LiDAR no debe tener suscripción a `/joint_states`.
Encoders y mixto sí la tienen. Los tres pueden recibir `/scan`: en encoders
se utiliza para guardar y mostrar el mapa, sin corregir la pose.

7. Detener Repeat con Ctrl+C y repetir desde el paso 1 con:

| Modo | Argumento | Archivo de ejemplo |
|---|---|---|
| Encoders | `mode:=encoders` | `routes/prueba_encoders.json` |
| LiDAR | `mode:=lidar` | `routes/prueba_lidar.json` |
| Mixto | `mode:=mixed` | `routes/prueba_mixed.json` |

Cambiar ambos comandos, Teach y Repeat, para usar el mismo modo y archivo.
No hace falta reiniciar Gazebo ni reiniciar su reloj.

## Qué demuestra cada prueba

Las pruebas automatizadas `test_sensor_sources_and_mixed_lidar_recovery` hacen
discrepar encoders y scans con ICP real de Open3D: encoders conserva la pose de
ruedas; LiDAR estima movimiento sin recibir ruedas; mixto predice con encoders,
recibe una corrección LiDAR y sigue integrando desde la pose corregida. Incluyen
un intervalo sin scans y la posterior recuperación de LiDAR.

El ensayo Gazebo comprueba el sistema completo en Teach y Repeat. Los errores
reportados son respecto al objetivo en la localización estimada; no son mediciones
independientes de error físico ni demuestran ausencia de deriva en recorridos largos.
