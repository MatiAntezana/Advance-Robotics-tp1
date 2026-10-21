# Análisis del TP1 y de las clases anteriores

## Consigna (TP1.pdf, tres páginas)

La sección 1 exige aprender por teleoperación y repetir autónomamente con ruedas
y/o LiDAR 2D. La sección 2 detalla `/rel_odom`, las dos TF comunes, las tres TF de
Repeat, persistencia de keyframes por distancia/ángulo y un único sistema con tres
modos. La sección 3 enumera proyecto ROS, launch, RViz, README, validación con
presentación y demostración real. La sección 4 pondera encoders (4), LiDAR (4) y
mixto (2). La implementación cubre el sistema y los cuatro primeros entregables,
con un ensayo reproducible de Gazebo como soporte de validación. No reemplaza la
presentación oral ni la demostración física.

## Revisión local de antecedentes (solo lectura)

| Material | Aporte | Adaptación al TP |
|---|---|---|
| Clase 1 y underlay compartido TurtleBot3 | URDF, mundo, modelo Burger, bridge y launch de simulación | Launch propio, sin TF de Gazebo y con raíz `base_link`. Velocidad `TwistStamped`. |
| Clase 2, `doc/enunciado.md`, `rota_robmob/odometry_node.py` y variante Grupo2 | Diferencias de ruedas, geometría diferencial, SE(2) y mapa | Exponencial exacta del arco, selección estricta por nombre y único nodo de localización. El TP requiere mapa de referencias, no obliga a grilla. |
| Clase 3, `doc/enunciado.md`, `rota_icp/lidar_odometry.py` | LaserScan a nube z=0, Open3D ICP actual→anterior, semilla | Extrínseca del láser, rechazo por fitness/RMSE y proyección planar; sincronización temporal con ruedas. El archivo real se llama `lidar_odometry.py`. |
| Clase 4, `doc/enunciado.md`, `control/diff.md`, `tnr/sprunk.md`, `diff_control_node.py` y pruebas | Escaneos de keyframes, localización contra anclas Teach, control polar y alineación final | Separación localización/control, avance secuencial, árbol TF exigido, watchdog de pared. Se omite evasión reactiva: la clase excluye planificación de nuevas rutas. |

Se revisaron además launch, bridge y URDF del workspace compartido. No se
modificaron clases ni underlay. El código nuevo queda dentro del repositorio TP1.

## Decisiones

- `T_a_b` transforma de b a a. `T_reference_base` es `/rel_odom`; la arista
  `base_link -> map_current_keyframe` requiere su inversa.
- Un único nodo obtiene pose en ambas etapas. LiDAR puro no suscribe encoders;
  encoders solo utiliza scans como datos de mapa. Control consume setpoint relativo.
- Mixto interpola ruedas al timestamp del scan y conserva el desplazamiento
  posterior al corregir. Evita tratar mediciones retrasadas como simultáneas.
- Repeat con LiDAR alinea también contra el keyframe Teach guardado, reduciendo
  deriva frente a solo seguir coordenadas globales integradas.
- Rutas con escaneos, poses, vínculos relativos y trayectoria; escritura atómica
  por keyframe, servicio para el punto final y guardado al salir.
- Un padre por trama, sin ciclos. Se elimina en memoria el padre estático original
  de `base_link`; el underlay permanece intacto.
- TF dinámicos con timestamp de medición, comandos con reloj actual y expiración.
- ICP tiene alcance local y falla con poco solapamiento o simetrías; encoders deriva
  con deslizamiento. Un error de llegada estimado no demuestra precisión física.

## Verificación

Las pruebas del núcleo verifican geometría SE(2), arco diferencial, sentido del
ICP con transformación conocida y rechazo de nubes inválidas, fusión retrasada,
convergencia del control hacia delante/atrás/giro y persistencia.
`scripts/validate_gazebo.py` ejecuta Teach→Repeat sobre sensores reales de Gazebo,
comprueba finalización, error estimado, padres TF y monotonía, y guarda evidencia
en `validation/`. Se requiere hardware para validar el robot real.
