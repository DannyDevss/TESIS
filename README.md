# TesisT60 — Workspace ROS 2 del UGV de rescate

Workspace de **ROS 2 Jazzy** para un vehículo terrestre no tripulado (UGV) de
rescate con **orugas y flippers articulados**. Incluye el puente con el hardware
de motores, la odometría, la fusión sensorial (EKF), el modelo del robot (URDF)
y una simulación de navegación 2D con política de aprendizaje por refuerzo (RL).

La documentación de la tesis (memoria, presentación, guion de defensa, simulación
de escritorio y entrenamiento RL) vive en el repo hermano
[TesisT60doc](https://github.com/ThanquolElGris/TesisT60doc).

## Paquetes (`src/`)

| Paquete | Rol | Ejecutables |
|---------|-----|-------------|
| **ugv_bridge** | Puente ROS ↔ hardware de motores (orugas + flippers) + IMU, odometría, EKF, URDF y launch | `flipper_node`, `track_odometry_node` |
| **ugv_core** | Entorno del robot (`robot_env`) compartido con la simulación | — |
| **ugv_sim** | Simulación 2D: publica `/scan`, `/odom`, TF; escucha `/cmd_vel` | `sim_node` |
| **ugv_policy** | Cerebro de decisión (heurística / política RL) → `/cmd_vel` | `policy_node` |
| **ugv_gcs** | Estación de control en tierra (GCS) mínima, mini-mapa 2D | `gcs_node` |
| **ugv_msgs** | Mensajes propios (desactivado con `COLCON_IGNORE`) | — |

## Arquitectura del track de flippers (`ugv_bridge`)

```
política / teleop
   │  /cmd_tracks (vel. orugas)   /cmd_flippers (pos. flippers)
   ▼
flipper_node  ──►  RMD_Hardware (real pi3hat/CAN  o  gemelo digital)
   │  /joint_states   /imu/data_raw
   ├──► track_odometry_node ──► /odom ─┐
   │                                   ├──► EKF (robot_localization) ──► /odometry/filtered + TF
   └──► robot_state_publisher (URDF) ──► TF de cada flipper ──► RViz
```

- **8 motores**: IDs 1–4 orugas (comando de velocidad), IDs 5–8 flippers (comando de posición).
- **Flippers**: juntas `continuous`, giran **360°** sin topes.
- **Sim-to-real**: el mismo nodo sirve para simulación y hardware; solo cambia el
  parámetro `modo_simulacion`. El modo hardware real (pi3hat + CAN + IMU) queda como TODO.

## Compilar

```bash
cd ~/Proyectos/ugv_ws
colcon build
source install/setup.bash
```

## Uso rápido

```bash
# Posar el robot con sliders (RViz), sin locomoción:
ros2 launch ugv_bridge view.launch.py

# Sistema completo (driver + odometría + EKF + RViz):
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true use_rviz:=true

# Mover (un solo mensaje basta; el nodo retiene el último comando):
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [0.78,0.78,0.78,0.78]}"
ros2 topic pub --once /cmd_tracks   std_msgs/msg/Float64MultiArray "{data: [5.0,5.0,5.0,5.0]}"

# Navegación 2D (simulación + política):
ros2 run ugv_sim sim_node
ros2 run ugv_policy policy_node
ros2 run ugv_gcs gcs_node
```

> ⚠️ Usa `--once`, no `-r`. La velocidad de las orugas es persistente: para frenar,
> publica explícitamente `[0,0,0,0]` en `/cmd_tracks`.

## Documentación

- **[COMO_EJECUTAR.md](COMO_EJECUTAR.md)** — runbook detallado (entornos, launch, tópicos, inspección).
- **[DEFENSA_ROS2.txt](DEFENSA_ROS2.txt)** — guía de preguntas/respuestas de ROS 2 para la defensa.
- **[BITACORA_2026-07-11.txt](BITACORA_2026-07-11.txt)** — bitácora de construcción del track de flippers.
