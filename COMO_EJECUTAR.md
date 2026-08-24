# Cómo ejecutar cada cosa en ROS

Runbook práctico del proyecto (UGV de rescate). Cubre los dos entornos y todos los
nodos/launch. Guía rápida; el detalle de arquitectura está en `tesis/CLAUDE.md`.

## 0. Entornos ROS (no hay ROS en `/opt/ros` del host)

| Entorno | Distro | Cómo entrar |
|---------|--------|-------------|
| **Principal** | Jazzy | comando `rosdev` (= `distrobox enter ros2 -- bash`) o el acceso directo de escritorio "ROS 2 Jazzy" |
| Alternativo | Humble | env micromamba: `micromamba activate ros2` |

Dentro de `rosdev` el `~/.bashrc` ya sourcea Jazzy **y** el workspace `ugv_ws` en cada
terminal nueva. En **fish** (host), para cargar un workspace colcon: `rosws [ruta_ws]`.

> Cierra siempre los nodos con **Ctrl+C** en su terminal (evita procesos huérfanos).
> Si `ros2 node list` muestra nodos que ya cerraste: `ros2 daemon stop && ros2 daemon start`.

## 1. Compilar el workspace

```bash
rosdev                                   # entrar al contenedor Jazzy
cd ~/Proyectos/ugv_ws
colcon build                             # todos los paquetes
colcon build --packages-select ugv_bridge   # uno solo
source install/setup.bash                # tras cada build (en bash; en fish: rosws)
```

## 2. Track de FLIPPERS (paquete `ugv_bridge`)

Hay **dos launch** según lo que quieras hacer:

| Launch | Para qué | Corre |
|--------|----------|-------|
| **`view.launch.py`** | posar/mirar el robot: mover las 4 articulaciones con sliders, robot fijo y centrado | robot_state_publisher + joint_state_publisher_gui + RViz |
| **`flipper.launch.py`** | sistema completo: driver, IMU, odometría, EKF y locomoción | flipper_node + track_odometry_node + robot_state_publisher (+ EKF/RViz opcionales) |

### 2a. Posar los flippers con sliders (sin movimiento)

```bash
ros2 launch ugv_bridge view.launch.py
```

Abre RViz (fixed frame `base_link`, el robot no se desplaza) y una **ventana con un slider
por cada flipper** (`fl/fr/rl/rr`). Las juntas son `continuous` (**giro completo de 360°**),
así que el slider recorre todo el rango. Mueve un
slider y el brazo gira en tiempo real. Aquí `/joint_states` lo publica la GUI, así que **no**
se lanza `flipper_node` (entrarían en conflicto en el mismo tópico).

### 2b. Sistema completo (con locomoción, IMU y EKF)

```bash
# Básico: flipper_node + track_odometry_node + robot_state_publisher
ros2 launch ugv_bridge flipper.launch.py

# Con fusión sensorial EKF (robot_localization) y/o RViz:
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true use_rviz:=true

# Ajustar frecuencia del bucle de control o modo hardware:
ros2 launch ugv_bridge flipper.launch.py frecuencia_hz:=200.0 modo_simulacion:=true
```

Nodos sueltos (para depurar):

```bash
ros2 run ugv_bridge flipper_node
ros2 run ugv_bridge track_odometry_node --ros-args \
    --params-file src/ugv_bridge/config/geometria_robot.yaml
```

> Los parámetros de geometría (`radio_oruga`, `ancho_orugas`, antes
> `wheel_radius`/`track_width`) salen de `config/geometria_robot.yaml`, que es la
> única fuente de verdad y la comparte con el URDF y el guardián de colisiones.

### Cómo mover el robot en la simulación

El robot se controla con dos tópicos `std_msgs/Float64MultiArray` (4 valores, orden
`fl, fr, rl, rr`). `flipper_node` **retiene el último comando recibido** y lo aplica en
cada ciclo del bucle, así que **un solo mensaje basta** y persiste:

```bash
# FLIPPERS: posición en radianes. 0.78 = 45°, 0.0 = plano.  (se quedan ahí)
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [0.78,0.78,0.78,0.78]}"

# ORUGAS: velocidad en rad/s.  5.0 = avanzar (~0.25 m/s con r=0.05 m).
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [5.0,5.0,5.0,5.0]}"

# GIRAR en el sitio: lados opuestos (izq fl/rl vs der fr/rr):
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [5.0,-5.0,5.0,-5.0]}"

# DETENER las orugas (importante: la velocidad NO se resetea sola):
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0]}"
```

> ⚠️ **Usa `--once`, no `-r 20`.** Como el nodo retiene el último valor, no hace falta
> publicar en bucle. Un `ros2 topic pub -r 20 ...` deja un **publicador vivo** que sigue
> mandando velocidad para siempre; si lo olvidas (o abres varios), el robot se conduce solo
> indefinidamente y su pose se dispara (lo vimos llegar a ~180 m y salir de cuadro en RViz).
> Si usas `-r`, **ciérralo con Ctrl+C** al terminar. Para ver publicadores olvidados:
> `ros2 topic info /cmd_tracks` (Publisher count) o `pgrep -af "topic pub"`.

> Recordatorio: la velocidad de orugas es persistente. Tras avanzar, manda explícitamente
> `[0,0,0,0]` para frenar; no basta con dejar de publicar.

Inspeccionar:

```bash
ros2 topic echo /joint_states          # 8 motores (4 orugas + 4 flippers)
ros2 topic echo /imu/data_raw          # IMU (cuaternión)
ros2 topic echo /odom                  # odometría de orugas
ros2 topic echo /odometry/filtered     # salida del EKF (solo con use_ekf:=true)
ros2 run tf2_ros tf2_echo base_link flipper_fl_link   # TF de un flipper
ros2 run rqt_plot rqt_plot /joint_states/position[4]  # ángulo de un flipper en el tiempo
```

En RViz: fixed frame `base_link` (sin EKF) u `odom` (con EKF); displays RobotModel + TF.

### 2c. Foxglove: puente + layout del proyecto

Foxglove **no habla ROS 2 directamente**: necesita el puente WebSocket. Se levanta
aparte, sobre un sistema que ya esté corriendo (no hace falta relanzar nada):

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Luego, en Foxglove Studio, conectar a `ws://localhost:8765` (fuente *Foxglove
WebSocket*). Desde el host se puede abrir ya conectado, sin elegir fuente a mano:

```bash
foxglove-studio "foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765"
```

> El puente corre **dentro** de `rosdev`, pero distrobox comparte la red con el host,
> así que `localhost:8765` funciona desde Foxglove instalado en el host.

**Layout del proyecto:** `config/ugv_control_v2.json`. Se importa con
*Layouts → (menú ⋯) → Import from file…* y queda guardado en la app.

> ⚠️ **Foxglove guarda su PROPIA copia del layout, con el nombre del archivo.**
> Editar el archivo del repositorio no cambia nada de lo que ves en la app, y
> reimportar un archivo cuyo nombre ya existe deja dos entradas parecidas entre
> las que es facilísimo elegir la vieja: el sintoma es "lo importé y al salir de
> Foxglove volvió el layout de antes". Por eso el archivo lleva `_v2` en el
> nombre. Al importar, entra como layout **`ugv_control_v2`**; selecciónalo en el
> desplegable de arriba a la derecha y **borra el `foxglove_layout` antiguo**
> (*click derecho sobre el layout → Delete*) para no volver a confundirlos.
>
> Si aun asi los cambios no persisten al cerrar la app, mira el aviso del plan de
> la organizacion: con la cuenta por encima de su limite de usuarios, los layouts
> sincronizados con el equipo pueden dejar de guardarse. En ese caso, duplicalo
> como layout **personal** (*Make a personal copy*) y trabaja sobre esa copia.

Trae 8 paneles ya configurados: 3D (con **Display frame = `odom`** y el modelo desde
`/robot_description`, así nadie se vuelve a tropezar con el robot "plano"), gráficas
de ángulos de flipper y de velocidad de orugas, gráfica de la pose del EKF (para
vigilar que Z no derive), dos paneles de publicación para `/cmd_flippers` y
`/cmd_tracks`, la IMU cruda y `/rosout`.

Los paneles de publicación funcionan porque el puente expone la capability
`clientPublish` (activa por defecto). Recuerda que la velocidad de orugas es
persistente: hay que frenar mandando `[0,0,0,0]`.

### Que el modelo 3D se INCLINE en Foxglove (no sólo los flippers)

`robot_state_publisher` dibuja únicamente la geometría **interna** del robot: mueve
los flippers respecto a `base_link`, pero **nunca mueve `base_link` dentro del
mundo**. Por eso, aunque `/imu/data_raw` traiga una orientación perfecta, el modelo
se ve plano: falta quién publique dónde está el chasis.

Eso lo hace el **EKF**, con el TF `odom -> base_link`. Hay que levantarlo:

```bash
ros2 launch ugv_bridge can_sim.launch.py use_ekf:=true
# o, sin emulador CAN (gemelo digital):
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true
```

En el panel 3D de Foxglove, poner **Display frame = `odom`** (con `base_link` el
robot queda fijo y sólo se mueven los flippers, que es justo el síntoma).

Comprobar que la inclinación llega de verdad:

```bash
ros2 run tf2_ros tf2_echo odom base_link      # debe cambiar el RPY al inclinarse
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [1.0,1.0,0.0,0.0]}"
# -> pitch NEGATIVO (~-34°): en REP-103 el eje Y apunta a la izquierda, así que
#    morro ARRIBA es pitch negativo (al revés que en convención aeronáutica).
```

Las dos cadenas de TF son complementarias y no se pisan:

```
odom --[EKF]--> base_link --[robot_state_publisher]--> flippers, imu_link
```

`track_odometry_node` emite `odom -> base_link` **sólo cuando el EKF está apagado**
(el launch lo ajusta solo). Si ambos lo publicaran, `base_link` tendría dos padres
y el árbol TF quedaría inválido: en Foxglove el modelo tiembla o desaparece.

### 2d. Motores simulados + IMU FÍSICA: todo corre en la Raspberry

La IMU del pi3hat se lee por **SPI**, así que solo puede abrirla un proceso que
corra EN la Raspberry. Y el grafo ROS **no se puede repartir** entre el PC y la
Pi: el multicast de discovery no cruza la wifi (y `ROS_STATIC_PEERS` tampoco
enganchó). Con IMU real, entonces, todo se lanza allá y el PC solo mira:

Dentro de la Pi hay **dos comandos** que hacen todo (matar restos de la corrida
anterior, compilar y lanzar). Se usan desde cualquier directorio:

```bash
ssh ros2@robotdeteccion.local

compilar_simu      # motores SIMULADOS (gemelo digital) + IMU FISICA real
compilar_real      # motores REALES por los buses CAN del pi3hat + IMU real
```

Los dos encienden el EKF y el puente Foxglove. Aceptan argumentos extra, que se
pasan tal cual al launch: `compilar_simu frecuencia_hz:=200.0`.

#### Cableado CAN al pi3hat

Conectores **JST PH-3** (JC1..JC5, uno por bus), según la
[referencia de mjbots](https://github.com/mjbots/pi3hat/blob/master/docs/reference.md):

| Pin | Señal |
|-----|-------|
| 1   | CAN_H |
| 2   | CAN_L |
| 3   | GND   |

- La **terminación de 120 ohm ya viene soldada** en los cinco puertos del pi3hat:
  no hay que agregar resistencias de ese lado. Entre pin 1 y pin 2, con todo
  desconectado, se miden ~120 ohm (~60 si el driver del motor también termina).
- Para saber cuál extremo es el pin 1 sin adivinar: el pin 3 da continuidad con
  la masa de la Pi (chasis del conector Ethernet, o un GND del header de 40). El
  extremo opuesto es el pin 1.
- Conectar también el **GND**: con dos hilos el bus funciona solo si las masas ya
  se tocan por la alimentación. Si el driver va con fuente aparte, sin referencia
  común el CAN falla.
- Velocidad de fábrica del **GIM6010-8: 500 kbps** — es la que fija
  `BITRATE_CAN` en `pi3hat_backend.py`.

#### El protocolo es ODrive, no RMD

El driver del GIM6010-8 (CyberBeast BL72) es compatible ODrive, y su protocolo
CAN es el de ODrive:

    ID de arbitraje (11 bits) = (node_id << 5) | cmd_id

Tres consecuencias prácticas:

1. **El motor habla solo.** De fábrica emite `Heartbeat` cada 100 ms y
   `Get_Encoder_Estimates` (posición y velocidad) cada 10 ms. El driver no
   pregunta: manda su consigna y recoge lo que llegó. Por eso el diagnóstico
   (`escanear_can.py`, `verificar_pi3hat.py --motores`) sólo **escucha**.
2. **Hay que ARMAR los ejes.** Un eje en IDLE acepta las consignas y no se mueve,
   sin devolver ningún error. `driver_movimiento` lo hace al arrancar y cada vez
   que el watchdog se recupera de un fallo.
3. **El `node_id` sustituye al ID del motor.** Se lee y se cambia por USB con
   `odrv0.axis0.config.can.node_id`. El motor de pruebas viene con `node_id = 1`.

Para depurar por USB: `~/.venvs/odrive/bin/odrivetool` en el PC (necesita la
regla udev de `/etc/udev/rules.d/91-odrive.rules`).

Si no contesta nadie, `scripts/escanear_can.py` barre buses, velocidades e IDs y
muestra cualquier trama que llegue.

#### Probar con UN motor en el banco

Antes de lanzar nada, comprobar que el motor contesta en el bus (trama de
lectura, no mueve nada):

```bash
python3 ~/TESIS/src/ugv_bridge/scripts/verificar_pi3hat.py --id 1 --bus 1
```

Si responde, lanzar hablándole SOLO a ese motor:

```bash
compilar_real motores_presentes:=1
```

Sin `motores_presentes`, el driver exige respuesta de los 8: los 7 que no están
cableados disparan el watchdog, que declara `¡FALLO DE COMUNICACIÓN!` y fuerza
todos los comandos a cero — el motor conectado tampoco se mueve, aunque su
cableado esté perfecto. Con la lista, los ausentes se ignoran (siguen apareciendo
en `/joint_states`, quietos, porque el URDF necesita las 8 juntas).

Salen de `scripts/tesis_lanzar.sh` (un solo script; el nombre con el que se lo
invoca decide el modo) y se instalan con:

```bash
bash src/ugv_bridge/scripts/instalar_comandos.sh
```

El equivalente a mano, por si hace falta cambiar algo:

```bash
source /opt/ros/jazzy/setup.bash && source ~/TESIS/install/setup.bash
ros2 launch ugv_bridge flipper.launch.py \
    modo:=gemelo imu_fuente:=pi3hat_real use_ekf:=true use_foxglove:=true
```

Desde el PC, Foxglove se conecta al puente de la Pi (TCP, eso sí cruza):

```
ws://robotdeteccion.local:8765
```

> Usa el nombre mDNS y no la IP: la Pi salta entre wifi y cable y la IP cambia.
> El panel 3D, como siempre, con **Display frame = `odom`**.

Si un launch falla a medias deja **nodos huérfanos**, y el siguiente intento
muere con `pi3hat: could not acquire lock, is another process running?` (la placa
admite un solo dueño del SPI). Antes de relanzar:

```bash
pkill -9 -f "lib/ugv_bridge/"
```

**Trampa del sistema en la Pi:** `python3` tiene capabilities puestas para hablar
con el pi3hat sin root (`cap_dac_override,cap_sys_rawio`), y por eso glibc le
**borra `LD_LIBRARY_PATH`**. Todo nodo lanzado con `ros2 run`/`ros2 launch` (que
son Python) hereda esa variable vacía: el síntoma es un nodo C++ que muere con
`cannot open shared object file` sobre una librería que existe y tiene permisos.
La solución no es pelear con `LD_LIBRARY_PATH`, es registrar el directorio en
`/etc/ld.so.conf.d/ros2-jazzy.conf` y correr `sudo ldconfig`.

## 3. Track de NAVEGACIÓN 2D (simulación + política RL)

```bash
ros2 run ugv_sim sim_node        # avanza RobotEnv 20 Hz: publica /scan, /odom, TF; escucha /cmd_vel
ros2 run ugv_policy policy_node  # decide con el cerebro (heurística/RL) y publica /cmd_vel
#   con modelo RL: ros2 run ugv_policy policy_node --ros-args -p model_path:=/ruta/modelo_robot.zip
ros2 run ugv_gcs gcs_node        # GCS mínima ROS2 (mini-mapa 2D)

# Todo junto (sim + policy + la GUI simu.py de tesis/):
ros2 launch ~/Proyectos/ugv_ws/ugv.launch.py
```

## 4. App / simulación de escritorio (carpeta `tesis/`, venv propio)

No es ROS puro, pero `simu.py` importa `rclpy` (necesita ROS sourceado). Desde el venv de `tesis/`:

```bash
python simu.py             # GCS completa (PySide6 + OpenGL)
python robot_env.py        # demo headless del entorno + heurística (métricas)
python entrenar.py         # entrena PPO -> modelo_robot.zip (requiere gymnasium/sb3/torch)
python entrenar.py eval    # evalúa un modelo entrenado
```

## 5. Instalar dependencias ROS

```bash
# En Jazzy (dentro de rosdev):
sudo apt install ros-jazzy-<paquete>       # ya instalados: robot-localization, rviz2
# En Humble (micromamba):
mamba install ros-humble-<paquete>
```
