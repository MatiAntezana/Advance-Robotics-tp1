# Advance-Robotics-tp1 — Sistema Teach & Repeat

Trabajo Práctico 1 de Robótica Avanzada (Grupo 2): Diseño e implementación de un sistema de navegación autónoma **Teach and Repeat (T&R)** en ROS 2 Jazzy para un robot móvil diferencial (TurtleBot3 Burger) utilizando odometría de encoders, LiDAR 2D (ICP) y fusión mixta.

## Estructura del Repositorio

```text
Advance-Robotics-tp1/
├── TP1.pdf                 # Enunciado del Trabajo Práctico 1
├── pixi.toml               # Configuración del entorno y tareas reproducibles
├── pixi-activate.sh        # Script de activación de variables ROS 2 / Gazebo
└── ros_ws/                 # Workspace de ROS 2
    └── src/
        └── Grupo2/         # Paquetes desarrollados por el equipo
```

## Comandos Rápidos (con Pixi)

```bash
# Compilar el workspace
pixi run build

# Lanzar simulación en Gazebo
pixi run sim

# Etapa Teach (Aprendizaje)
pixi run teach

# Etapa Repeat (Repetición)
pixi run repeat

# Visualización en RViz2
pixi run rviz
```

## Compilación Estándar (con entorno ROS 2 activo)

```bash
cd ros_ws
colcon build
source install/setup.bash
```
