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

**Layout del proyecto:** `config/ugv_control_v3.json`. Se importa con
*Layouts → (menú ⋯) → Import from file…* y queda guardado en la app.

> ⚠️ **Foxglove guarda su PROPIA copia del layout, con el nombre del archivo.**
> Editar el archivo del repositorio no cambia nada de lo que ves en la app, y
> reimportar un archivo cuyo nombre ya existe deja dos entradas parecidas entre
> las que es facilísimo elegir la vieja: el sintoma es "lo importé y al salir de
> Foxglove volvió el layout de antes". Por eso el archivo lleva un número de
> versión en el nombre, que sube cada vez que cambia el layout. Al importar,
> entra como layout **`ugv_control_v3`**; selecciónalo en el desplegable de
> arriba a la derecha y **borra los antiguos** (`ugv_control_v2`,
> `foxglove_layout`) con *click derecho sobre el layout → Delete*, para no
> volver a confundirlos.
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

**Reparto: dos motores por bus, emparejados POR ESQUINA.** La oruga y el flipper
de una misma esquina comparten bus, porque ahí es donde cae el empalme del arnés:
los cuatro ramales salen idénticos y cortos. Cuatro buses en uso, el quinto libre.

| Bus (JC) | node_id | Junta | Esquina |
|---|---|---|---|
| 1 | 1 / 5 | `track_fl` / `flipper_fl` | delantera izquierda |
| 2 | 2 / 6 | `track_fr` / `flipper_fr` | delantera derecha |
| 3 | 3 / 7 | `track_rl` / `flipper_rl` | trasera izquierda |
| 4 | 4 / 8 | `track_rr` / `flipper_rr` | trasera derecha |

La fuente de verdad es `mapa_buses` en `config/geometria_robot.yaml`, en orden de
ID 1..8: `[1, 2, 3, 4, 1, 2, 3, 4]`. El mismo reparto está replicado en
`MAPA_BUSES_POR_DEFECTO` (`driver_movimiento.py`, para usar la clase sin ROS) y en
`MAPA_BUSES` de `verificar_pi3hat.py`. Si cambia el cableado, cambian los tres.

**El empalme es una Y, y eso cambia la terminación.** Cada bus queda con tres
nodos: el pi3hat y los dos motores de la esquina.

- El pi3hat **ya termina** su extremo (120 ohm soldados en los cinco puertos).
- En el otro extremo debe terminar **UN solo motor**, no los dos. El segundo
  cuelga como stub del empalme y va SIN terminación.
- Ese stub, corto: por debajo de ~30 cm a 500 kbps. Por eso conviene empalmar en
  la esquina y no en la Pi.
- Comprobación con todo conectado y sin alimentar: entre CAN_H y CAN_L de cada
  bus deben salir **~60 ohm**. Si salen ~40, hay una terminación de más (los dos
  motores terminando); si salen ~120, falta la del extremo lejano.
- El GND del JST PH-3 va empalmado igual que CAN_H/CAN_L: los dos motores tienen
  que compartir referencia con la Pi.

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

#### El Motor Wizard bajo Wine (visor opcional, sólo en el PC)

El manual (§3.1.2) ofrece dos programas de PC: el **Motor Wizard** de SteadyWin y
`odrivetool`. **La configuración de la tesis se hace con `odrivetool`** (apartado
siguiente): corre nativo, no depende de Wine y es lo que documenta el resto de
esta guía. El Wizard es un visor cómodo — estado, tensión, y pestañas de par,
velocidad y posición — pero es un `.exe` de 32 bits, sólo habla USB motor a
motor, y no toca el bus CAN. No sirve en la Raspberry (es ARM) ni dentro de
rosdev (no hay Wine ni acceso al USB del host).

Instalación, una sola vez:

```bash
curl -L -o /tmp/motorwizard.exe https://bl.cyberbeast.cn/actuator/steadywin_motorwizard.exe
WINEPREFIX=~/.local/share/wineprefixes/motorwizard wineboot -i
WINEPREFIX=~/.local/share/wineprefixes/motorwizard wine /tmp/motorwizard.exe \
    /VERYSILENT /SUPPRESSMSGBOXES /NORESTART
```

Queda en `~/.local/share/wineprefixes/motorwizard`, en un prefijo aparte para no
mezclarlo con el `~/.wine` de siempre. Después se abre con el comando que instala
`scripts/instalar_comandos.sh`:

```bash
motorwizard
```

> **El paso de Zadig del manual no aplica en Linux.** Zadig es el instalador del
> driver USB de Windows (`libusb` para el Wizard, `WinUSB` para `odrivetool`, y
> hay que ir cambiándolo al saltar de un programa al otro). Aquí el permiso lo da
> la regla udev `/etc/udev/rules.d/91-odrive.rules`, la misma para los dos, y no
> hay nada que reasignar.

Ruido de arranque que **no** es un fallo:

- `MESA-EGL: ... failed to create dri2 screen`: es la GL de la NVIDIA de este
  equipo, cae a software y la ventana sale igual.
- `[ERROR:...direct_manipulation.cc] ... failed`: Wine no implementa el API de
  táctil/lápiz que pide Flutter. Inofensivo.
- Sin motor alimentado y conectado por Type-C, la interfaz abre pero **no muestra
  una lectura de tensión válida** (el manual lo dice en §3.1.3). El lanzador
  avisa antes de abrir si no ve el `1209:0d32` en el USB.

Y dos fallos de verdad que salieron en la primera corrida, los dos ya resueltos:

- `PathNotFoundException: ... MotorWizard\datalog`. El Wizard **lista** su
  carpeta de registros al arrancar pero no la crea. `motorwizard.sh` la crea
  antes de lanzar; dentro del prefijo `Documents` es un enlace a `~/Documentos`,
  así que los registros quedan en `~/Documentos/MotorWizard/datalog`.
- `FormatException: Invalid radix-10 number ... 0.5` (`motor.dart:309`). La app
  guarda `NULLDATAID = "0.5"` en sus preferencias y luego la lee con
  `int.parse`. Importa porque el `forEach` que revienta **deja sin cargar todos
  los parámetros posteriores** de la lista (`RS`, `LS`, `VoltageConstant`,
  `TorqueConstant`, `Rshunt`, `AmplificationGain`). Se arregla poniendo un entero
  en ese campo, una sola vez:

  ```bash
  PREF=~/.local/share/wineprefixes/motorwizard/drive_c/users/$USER/AppData/Roaming/CyberBeast/电机精灵/shared_preferences.json
  cp "$PREF" "$PREF.bak"
  sed -i 's/"flutter.NULLDATAID":"0.5"/"flutter.NULLDATAID":"0"/' "$PREF"
  ```

  Con el motor delante conviene confirmar que esos valores salen ahora bien; si
  algo se descuadra, el `.bak` deja las preferencias como estaban.

Y el orden de conexión importa: **alimenta por XT30 con la fuente apagada,
enciende, y sólo entonces conecta el Type-C.** El USB es consola: no alimenta la
etapa de potencia ni mueve el motor.

Lo que queda por verificar con hardware delante es si el `libusb-1.0.dll` de
Windows llega al motor a través de Wine (va por su `winusb` hacia el libusb de
Linux). Si el Wizard no lista el dispositivo, no hay mucho que rascar en Wine:
`odrivetool` hace todo lo necesario y sí funciona.

#### Configurar los 8 motores por USB (una sola vez, antes de montarlos)

El `node_id` **no se asigna al arrancar ni se negocia en el bus**: vive en la
flash del driver y hay que grabarlo a mano, motor por motor, antes de montar el
arnés. `compilar_real` sólo compila y lanza; da por hecho que cada motor ya
responde a su ID. Y los IDs no son libres: el reparto de arriba está fijado en el
código (`IDS_ORUGAS`, `IDS_FLIPPERS`, `ID_A_NOMBRE`).

**Antes de empezar (una vez):** `odrivetool` instalado
(`~/.venvs/odrive/bin/odrivetool`), la regla udev
`/etc/udev/rules.d/91-odrive.rules`, una fuente de 15-60 V con XT30 y etiquetas.
El USB es sólo consola: la etapa de potencia se alimenta por el XT30.

> **Un solo driver conectado a la vez.** Todos vienen con el mismo `node_id` de
> fábrica; si juntas dos, se pisan en el bus y `odrivetool` te da `odrv0`/`odrv1`
> sin decirte cuál es cuál.

Por cada motor:

1. Aliméntalo por el XT30 y conéctalo al PC por USB. Nada de CAN todavía.
2. Abre `~/.venvs/odrive/bin/odrivetool`. Debe aparecer `odrv0`, firmware 0.6.5.
3. Anota cómo viene: `odrv0.axis0.config.can.node_id`, `odrv0.config.enable_can_a`
   (casi seguro `False`), `odrv0.can.config.baud_rate`.
4. Escribe la configuración del bus:

   ```python
   odrv0.axis0.config.can.node_id = N        # N = el de la tabla de buses
   odrv0.config.enable_can_a = True          # viene APAGADO de fábrica
   odrv0.can.config.baud_rate = 500000       # igual en los 8 y en BITRATE_CAN
   ```

5. Ponle límites conservadores, **sobre todo a los flippers**, que empujan contra
   topes mecánicos. El software no los toca: `_armar_motores()` sólo manda
   `CLEAR_ERRORS`, `SET_CONTROLLER_MODE` y `SET_AXIS_STATE`; `CMD_SET_LIMITS`
   está implementado en `protocolo_can.py` pero nadie lo llama, así que los
   límites se quedan en lo que tenga la flash.

   ```python
   odrv0.axis0.config.motor.current_soft_max = ...
   odrv0.axis0.controller.config.vel_limit = ...
   ```

6. `odrv0.save_configuration()`. El driver se reinicia y la sesión se cae: es
   normal, no es un fallo.
7. Reconecta y **vuelve a leer** `node_id`, `enable_can_a` y `baud_rate`. Este
   paso es el que evita descubrir en la Pi que el `save` no cuajó.
8. Comprueba que arma sin recalibrar: pide `AXIS_STATE_CLOSED_LOOP_CONTROL` y
   mira que llegue a estado 8 sin lanzar una calibración. Si cada arranque quiere
   recalibrar, el robot no podrá armarse solo en campo.
9. **Etiqueta el motor** con su número y su junta (`3 / track_rl`). El número
   tiene que quedar escrito en el hardware.

Dos cosas más que conviene resolver mientras están en el banco:

- **Medir la reducción.** `RELACION_REDUCCION = 8.0` en `protocolo_can.py` es
  lectura de manual, no medición, y de ella depende toda la odometría. Con el eje
  en IDLE, gira el eje de **salida** exactamente una vuelta a mano y mira cuánto
  cambia `odrv0.axis0.encoder.pos_estimate`: si cambia 8.0, el valor es correcto;
  si cambia 1.0, el encoder ya mide en la salida y hay que poner 1.0. Basta con
  hacerlo en un motor.
- **El cero de los flippers.** El código no aplica ningún offset: el 0 rad que
  manda ROS es el 0 del encoder. Si el encoder no es absoluto, el cero será donde
  estuviera el flipper al energizar, y una consigna de 0 lo mandará a una postura
  arbitraria. Fija el cero con el flipper en reposo y confirma que sobrevive a un
  ciclo de apagado.

Con los ocho grabados y el arnés montado, la verificación es escuchar el bus sin
transmitir nada (los ODrive emiten heartbeat y encoder de fábrica):

```bash
python3 ~/TESIS/src/ugv_bridge/scripts/verificar_pi3hat.py --motores
```

Tienen que salir los ocho, cada uno en su bus. Si no contesta nadie, antes de
sospechar del cableado: `python3 src/ugv_bridge/scripts/escanear_can.py
--bitrate 500000` (y prueba otros bitrates), que descarta velocidad mal puesta o
un `enable_can_a` que se quedó atrás.

Y verifica la correspondencia junta-motor antes de bajar el robot al suelo — un
ID intercambiado entre `track_fl` y `track_fr` no lo detecta nada, el robot
simplemente gira al revés.

##### "Mando el comando, llega, y el robot no se mueve" (ejes en IDLE)

Es el fallo silencioso de ODrive y no se parece a un fallo: el eje **acepta** la
consigna, no se mueve, no devuelve error y sigue emitiendo su trama de encoder
cada 10 ms, así que el watchdog de comunicación lo ve perfectamente vivo. Da
igual que la orden venga del panel Publish de Foxglove, de los paneles Teleop o
de la política: `/cmd_flippers` sale, `flipper_node` la manda al bus y ahí muere.

`flipper_node` lo vigila por el **heartbeat** y lo reintenta solo cada 0,5 s.
Cómo se ve:

```
[HARDWARE] Fuera de lazo cerrado (aceptan órdenes y no se mueven): 5(flipper_fl). Reintentando armado.
[HARDWARE] Secuencia de armado enviada a 5(flipper_fl).
```

y, si la cosa persiste más de un segundo, un `WARN` en `/rosout` — o sea visible
desde el panel **RosOut** de Foxglove sin mirar la terminal de la Pi:

```
Motores FUERA DE LAZO CERRADO: flipper_fl. Aceptan las órdenes y no se mueven...
```

Si el aviso NO se va solo, el rearmado no está llegando al motor:

- ¿está energizado por XT30? El USB no alimenta la etapa de potencia;
- `odrv0.config.enable_can_a` y `odrv0.can.config.baud_rate = 500000`, grabados
  con `odrv0.save_configuration()` (viene de fábrica en `False`);
- el `node_id` del motor, contra la tabla de buses;
- en simulación, que `motor_emulator` esté corriendo: su línea de estadísticas
  cada 5 s dice `armados=8/8`. Si dice `armados=0/8`, ningún eje está armado.

#### Probar con uno o dos motores en el banco

**Estado actual del banco: los motores 1 (`track_fl`, oruga) y 5
(`flipper_fl`).** Son la esquina delantera izquierda, y por el emparejamiento por
esquina del arnés los dos van al **bus 1** (`mapa_buses` en
`config/geometria_robot.yaml`). Es la pareja mínima que ejercita los dos modos de
control a la vez: la oruga por VELOCIDAD y el flipper por POSICIÓN.

Antes de lanzar nada, comprobar que contestan en el bus (trama de lectura, no
mueve nada; `--id` acepta varios):

```bash
python3 ~/TESIS/src/ugv_bridge/scripts/verificar_pi3hat.py --id 1 5 --bus 1
```

Si responden, lanzar hablándole SÓLO a esos motores:

```bash
compilar_real motores_presentes:=1,5
```

Sin `motores_presentes`, el driver exige respuesta de los 8: los que no están
cableados disparan el watchdog, que declara `¡FALLO DE COMUNICACIÓN!` y fuerza
todos los comandos a cero — los motores conectados tampoco se mueven, aunque su
cableado esté perfecto. Con la lista, los ausentes se ignoran (siguen apareciendo
en `/joint_states`, quietos, porque el URDF necesita las 8 juntas), y sólo se
sondean los buses donde hay algo.

Qué mirar en Foxglove con estos dos:

| Panel | Serie | Motor |
|---|---|---|
| Plot *Velocidad de orugas* | `/joint_states.velocity[0]` | 1 (`track_fl`) |
| Plot *Ángulos de flippers* | `/joint_states.position[4]` | 5 (`flipper_fl`) |

Los botones **Avanzar / PARAR orugas** mueven el 1; **Los 4 a 45°** y los paneles
Teleop mueven el 5. Los demás motores salen planos en 0, que es lo esperado.

> **El robot 3D gira solo y no está roto.** `track_odometry_node` calcula el
> rumbo como la diferencia entre el lado izquierdo y el derecho, y en el banco
> sólo gira el izquierdo (`track_fl`): la odometría concluye que el robot rota,
> el EKF se lo cree y el modelo da vueltas en el panel 3D. Para verificar los
> motores, fíate de los Plot y de los Gauge, no de la pose. Con las cuatro orugas
> cableadas desaparece solo.

Salen de `scripts/tesis_lanzar.sh` (un solo script; el nombre con el que se lo
invoca decide el modo) y se instalan con:

```bash
bash src/ugv_bridge/scripts/instalar_comandos.sh
```

Ese mismo script deja además `motorwizard` (el visor USB bajo Wine), pero sólo en
un PC x86_64: en la Raspberry ni lo intenta.

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
