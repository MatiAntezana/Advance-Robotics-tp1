# Evidencia de ejecución - 21 de octubre de 2026

Entorno: macOS ARM64, ROS 2 Jazzy de RoboStack, Open3D 0.19.0, Gazebo Harmonic,
TurtleBot3 Burger en turtlebot3_world. Se utilizó la sesión tmux `Tp1-Robotica`.

## Resultado de la última ejecución completa

| Modo | Keyframes | Teach → Repeat | Error final de posición estimado | Error angular estimado |
|---|---:|---|---:|---:|
| Encoders | 6 | Completado, velocidad cero | 0.03422 m | 0.00422 rad |
| LiDAR | 6 | Completado, velocidad cero | 0.02632 m | 0.05415 rad |
| Mixto | 6 | Completado, velocidad cero | 0.03049 m | 0.05730 rad |

Los tres ensayos verificaron las cinco relaciones TF de la consigna y ausencia
de timestamps regresivos. Las rutas contienen avance, giro en el lugar y un
segundo avance. El robot se reposicionó con el servicio de Gazebo entre etapas,
sin reiniciar el reloj. Los nodos de localización no consumieron odometría de Gazebo.

Son errores respecto a la **pose estimada**, no precisión física contra ground
truth; tampoco permiten afirmar que un sensor sea más preciso que otro. LiDAR
presentó más variación en las poses y tardó aproximadamente 72 s en este Repeat,
especialmente en un giro; encoders tardó alrededor de 9 s y mixto alrededor de
25 s. La geometría y los mínimos locales de ICP afectan su convergencia. Las
trayectorias muestran esta variabilidad, sin suavizado que la oculte.

## Archivos

- `gazebo/results.json`: resultados y padres TF observados.
- `gazebo/*_route.json`: mapas con poses, escaneos, nubes y trayectoria Teach.
- `gazebo/*_trajectories.json`: muestras de odometría de las dos etapas.
- `gazebo/trajectories.png`: comparación gráfica de los tres ensayos.
- `tmux-validation.txt`: historial del desarrollo y ensayos, incluidas iteraciones
  iniciales fallidas antes de las correcciones. La última ronda completa es la
  que respalda la tabla.
- `tmux-build-tests.txt`: salida de compilación y pruebas automatizadas.

Compilación final: un paquete compilado correctamente, sin symlink-install.
Pruebas: **11 passed**. Las dos advertencias de pytest proceden de dependencias
`lark` que usan módulos obsoletos de Python, sin fallos de pruebas.

Se corrigieron durante las iteraciones: la inicialización del reloj del script,
el primer keyframe sin nube cuando la TF estática llega tarde y el cierre de nodos
para guardar y publicar la detención antes de cerrar el contexto ROS.
Gazebo emitió avisos Ogre2; se verificaron scans con rangos válidos y se completaron
los ensayos con dichos scans. RViz cargó la configuración del paquete. Al detener
localización y dejar solo Gazebo, RViz puede informar scans sin TF reciente.

No se realizó demostración física ni se generó una presentación oral. El comando
de reproducción del ensayo está en el README principal. Las ventanas tmux conservan
los comandos y salidas para inspección.
