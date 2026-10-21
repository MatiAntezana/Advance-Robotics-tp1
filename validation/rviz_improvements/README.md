# Mejoras RViz: trayectorias, keyframes, setpoint y alineación

Se implementaron los puntos 2, 3, 4 y 6 solicitados:

- Teach naranja con banda ancha y Repeat turquesa con banda fina, para ver ambas
  incluso cuando coinciden. El cambio de altura es solo de renderizado, no de pose.
- Flechas pequeñas y etiquetas KF; referencia activa verde y próximo destino magenta.
  En finalización se conserva el índice de referencia real y se marca el destino FINAL.
- Flecha/disco magenta de objetivo, etiqueta y línea robot→objetivo en odom.
- Residual scan/ancla, solapamiento y aviso de medición vieja. `/tnr/alignment`
  conserva valores numéricos y timestamp para diagnóstico.
- Teach LiDAR/mixto añade alineamiento a tres keyframes recientes como submapa fijo.
  Los encoders siguen siendo la única fuente de pose en el modo encoders.
- Integración de cada encoder; publicación de TF/odometría limitada a 50 Hz y paths
  a 10 Hz para evitar saturación cuando Gazebo entrega articulaciones a 1 kHz.
- `/tnr/localized_scan`: puntos observados en odom usando la pose estimada al tiempo
  de ese scan. El scan crudo con TF histórica queda disponible en otra capa opcional.
  Esto evita confundir la predicción histórica anterior al ICP con el resultado de
  localización corregido. No se ajustan puntos solo para coincidir visualmente.

La ruta del usuario `routes/mi_ruta_corregida.json` se conservó sin modificaciones
(SHA256: 8cbb8c09ec9736c1e3131134d4d033001f828cff484b6f4c7cc2c880e2d4018d).
Su auditoría local da aproximadamente 1.75 cm de residual medio entre nubes vecinas.
No equivale a un error de posición real y no elimina deriva histórica de esa ruta.
Las mejoras del mapa deben evaluarse en nuevos Teach. El mapa no se suaviza ni se
reescribe para ocultar discrepancias.

Verificación:

- 20 pruebas automatizadas aprobadas en ROS_DOMAIN_ID=43, aisladas de la simulación.
- Recuperación de un sesgo conocido de 4 cm mediante anclas fijas, frente a la
  configuración scan-a-scan de control de la misma prueba.
- Teach→Repeat completado para encoders, LiDAR y mixto: `results.json`.
- Comprobación de padres TF, timestamps no regresivos y velocidad cero al finalizar.
- Nueva ejecución mixta con la nube localizada y cierre seguro: `final_mixed/`.
- `live_markers.json`: comprobación por suscripción a los tópicos realmente
  publicados: etiquetas, marcos, extremos de la línea, paths y residual.
- `saved_route_alignment.json`: auditoría de la ruta original, sin modificarla.
- `new_teach_alignment.json`: auditoría de una ruta corta generada con los cambios.
  Las rutas son diferentes; sus residuales no representan un ensayo A/B controlado.

Durante un cierre con SIGINT apareció una excepción de conversión en pybind11 de
RoboStack. Se sustituyó la interrupción dentro de take_message por una solicitud
que deja salir al executor antes de guardar y cerrar ROS, verificada por una prueba
con un proceso real y por el ensayo mixto final.
