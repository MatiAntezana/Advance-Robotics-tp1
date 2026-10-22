# Verificación 2.C — 2026-10-21

Ensayo sobre Gazebo ya iniciado, realizado en la sesión tmux `Tp1-Robotica`.
El mismo ejecutable de localización se utilizó en Teach y Repeat en los tres modos.

| Modo | Keyframes | Repeat | Error final estimado | Error angular | Velocidad final |
|---|---:|---|---:|---:|---|
| encoders | 6 | completado | 3.43 cm | 0.00237 rad | cero |
| lidar | 6 | completado | 3.04 cm | 0.05453 rad | cero |
| mixed | 6 | completado | 3.09 cm | 0.05796 rad | cero |

Sin regresiones temporales de TF y con un único padre esperado por frame.
Detalles numéricos en `results.json`; rutas y trayectorias separadas por modo.
`trajectories.png` compara las trayectorias estimadas Teach y Repeat.
Estos errores no utilizan ground truth independiente de Gazebo.

Compilación correcta y 25 pruebas aprobadas. Se incorporó una regresión
parametrizada para selección de fuentes y recuperación LiDAR en mixto.
No fue necesario modificar el algoritmo de localización para superar el ensayo.

La ruta del usuario `routes/mi_ruta_corregida.json` permanece intacta:
SHA256 `8cbb8c09ec9736c1e3131134d4d033001f828cff484b6f4c7cc2c880e2d4018d`.

Pasos para repetir: [guía](../../docs/verificacion_tres_modos.md).
