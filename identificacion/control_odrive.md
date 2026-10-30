# El control de ODrive: los tres lazos que llevan los GIM6010-8

Resumen de la página oficial **Control Structure and Tuning** (ODrive v0.5.5,
<https://docs.odriverobotics.com/v/0.5.5/control.html>), con lo que cada parte
significa en este robot. Es el documento de referencia para entender *qué*
regula cada ganancia; el procedimiento para **medirlas** está en
[LEEME.md](LEEME.md).

Los ocho motores del robot son SteadyWin GIM6010-8 y su firmware es ODrive, así
que esta es la estructura de control que corre dentro de cada motor. Todo lo que
manda `protocolo_can.py` entra por uno de los tres escalones de abajo.

---

## 1. La cascada

El controlador es un lazo en cascada de **posición → velocidad → corriente**. No
son tres alternativas: son tres etapas encadenadas, y el modo de control elige
por dónde se entra.

```
                 ┌──────────┐      ┌──────────┐      ┌──────────┐
  pos_setpoint ──│ POSICIÓN │─vel──│ VELOCIDAD│─cur──│ CORRIENTE│──> V a los FET
                 │    (P)   │ cmd  │   (PI)   │ cmd  │   (PI)   │
                 └──────────┘      └──────────┘      └──────────┘
                      ▲                 ▲                 ▲
   modo posición ─────┘                 │                 │
   modo velocidad ──────────────────────┘                 │
   modo par ───────────────────────────────────────────────┘
```

| Modo | Qué etapas corren | Quién lo usa aquí |
|---|---|---|
| Posición | las tres | **flippers** (IDs 5-8) |
| Velocidad | velocidad + corriente | **orugas** (IDs 1-4) |
| Par | sólo corriente | los ensayos de `registrar_ensayo.py` |

Consecuencia directa: en las orugas el lazo de posición **no corre**, y por eso
`pos_gain` les da igual. En los flippers corren los tres, y la sintonía de
velocidad es el cimiento sobre el que se apoya la de posición.

---

## 2. Las ecuaciones

Tal como las da el manual.

**Posición** — es un lazo P puro, con una sola ganancia proporcional:

```
pos_error = pos_setpoint - pos_feedback
  vel_cmd = pos_error * pos_gain + vel_feedforward
```

**Velocidad** — PI:

```
       vel_error = vel_cmd - vel_feedback
current_integral += vel_error * vel_integrator_gain
     current_cmd = vel_error * vel_gain + current_integral + current_feedforward
```

**Corriente** — PI también:

```
    current_error = current_cmd - current_feedback
voltage_integral += current_error * current_integrator_gain
      voltage_cmd = current_error * current_gain + voltage_integral
                    + voltage_feedforward (cuando hay modelo de motor)
```

Tres detalles que el manual marca y conviene no olvidar:

- **`current_gain` y `current_integrator_gain` no se tocan a mano.** El firmware
  los calcula a partir de `motor.config.current_control_bandwidth`. Es
  exactamente el motivo por el que en [LEEME.md](LEEME.md) se descarta
  identificar el lazo de corriente: ya está resuelto por la calibración.
- **Par o corriente.** Las versiones modernas hablan en **N·m** a nivel de
  sistema en vez de amperios; hay un parámetro `torque_constant` que convierte
  entre ambos, y con esa traducción hecha el resto de la explicación vale igual.
- **La salida de cada etapa se satura antes de entrar en la siguiente.** El
  `vel_cmd` que sale del lazo de posición se recorta al límite de velocidad, y
  el par que sale del lazo de velocidad se recorta antes de llegar al de
  corriente. Los límites de velocidad y corriente no son alarmas: forman parte
  del camino de la señal.

---

## 3. Por qué no aparece el tiempo

En las ecuaciones de velocidad y corriente uno esperaría ver un `dt` (el
integrador integra *en el tiempo*). No está porque el periodo entre iteraciones
del controlador es **constante, 125 µs (8 kHz)**, y una constante se absorbe
dentro de las ganancias.

De ahí sale una advertencia práctica: las unidades de `vel_integrator_gain` ya
llevan el segundo dentro (`N·m/((rev/s)·s)`), y el número sólo tiene sentido
junto a la tasa del lazo que lo aplica. El lazo corre a 8 kHz **dentro del
motor** pase lo que pase con el bus: nuestra telemetría CAN a 100 Hz es lo que
nosotros vemos, no a lo que regula el driver.

---

## 4. Feedforward

`vel_feedforward` y `current_feedforward` existen para cuando el que manda ya
sabe qué va a pedir. El ejemplo del manual es el generador de trayectorias: con
un perfil trapezoidal de velocidad, en la rampa la velocidad sube mientras el
par es una constante no nula, y en el tramo plano la velocidad es constante y la
aceleración cero. Alimentando posición, velocidad y par a la vez, el error que
tienen que corregir los lazos es mucho menor.

En este robot hoy no se usan: el puente manda consigna a secas. Es una vía de
mejora para los flippers, donde el par de gravedad es predecible a partir del
ángulo.

---

## 5. Sintonía: el procedimiento del manual

Valores de partida que da la documentación (unidades de **vuelta**, no radianes):

| Parámetro | Unidad | Valor inicial |
|---|---|---|
| `pos_gain` | (turn/s)/turn | 20.0 |
| `vel_gain` | N·m/(turn/s) | 0.16 |
| `vel_integrator_gain` | N·m/((turn/s)·s) | 0.32 |

```
odrv0.axis0.controller.config.pos_gain = 20.0
odrv0.axis0.controller.config.vel_gain = 0.16
odrv0.axis0.controller.config.vel_integrator_gain = 0.32
```

Procedimiento aproximado (el autotune todavía no existe en esta versión):

1. Poner `vel_integrator_gain` a 0.
2. Asegurarse de partir de un sistema estable; si no lo es, bajar todas las
   ganancias hasta que lo sea.
3. Subir `vel_gain` un ~30 % por iteración hasta que el motor vibre.
4. Volver `vel_gain` al **50 %** del valor que vibraba.
5. Subir `pos_gain` un ~30 % por iteración hasta ver sobreoscilación.
6. Bajar `pos_gain` hasta que desaparezca.

Para el integrador, la regla del manual es

```
vel_integrator_gain = 0.5 * ancho_de_banda * vel_gain
```

donde el ancho de banda es el del seguimiento ya conseguido: si el sistema se
asienta en 100 ms, son `1/0,1 s = 10 Hz`, y entonces
`vel_integrator_gain = 0.5 * 10 * vel_gain`.

> **Ojo con esa fórmula.** El manual no dice si el ancho de banda va en Hz o en
> rad/s, y entre las dos lecturas hay un factor 2π. En [LEEME.md](LEEME.md) se
> resuelve esa ambigüedad eligiendo `0.25 * wc * vel_gain` (cero del PI en
> `wc/4`), que es la lectura conservadora.

La herramienta que recomienda el manual para ver el resultado es `liveplotter`:

```python
start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate,
                          odrv0.axis0.controller.pos_setpoint])
```

---

## 6. Qué significa todo esto en este robot

- **El método de subir un 30 % hasta que vibre es el que queremos evitar.** Es
  el "prueba y error" al que [LEEME.md](LEEME.md) pone alternativa: identificar
  `J`, `b` y `Tc` y calcular `vel_gain = J * wc`. Este documento sirve para
  saber *sobre qué* actúa cada ganancia; el otro, para ponerles número. Las dos
  vías tienen que dar resultados del mismo orden — si no, algo está mal medido.
- **Dos familias, dos juegos de ganancias.** Las orugas giran libres (planta de
  primer orden, sólo lazo de velocidad); los flippers tienen a la gravedad
  haciendo de muelle y corren la cascada entera. No se comparten ganancias.
- **El techo de ancho de banda no lo pone el motor, lo pone la cascada.** La
  separación sana es corriente (1000 rad/s de fábrica) ≫ velocidad ≫ posición.
  Con el lazo de velocidad en 50-150 rad/s y `pos_gain ≈ wc/4`, cada escalón
  queda unas 4-10 veces por debajo del anterior, que es lo que pide una cascada.
- **Unidades: vueltas, no radianes.** Los números del manual están en `turn` y
  `turn/s`, igual que las tramas CAN. Mientras `J` se identifique en esas mismas
  unidades, las ganancias salen directas, sin `2π` y sin la reducción 8:1 (que
  además sigue **sin verificar en hardware**).
- **`liveplotter` no está disponible aquí.** Va por USB contra `odrivetool`, y
  nosotros hablamos por CAN desde ROS. El equivalente son los CSV de
  `registrar_ensayo.py` y los rosbag de `/joint_states`.
- **El límite de velocidad y el de corriente participan en el control.** Al ser
  saturaciones entre etapas, un límite mal puesto se nota como un lazo que "no
  responde", no como un error. Y con `vel_integrator_gain` alto y una oruga
  bloqueada contra un obstáculo, el integrador se carga (windup) y el robot
  arranca de golpe al liberarse.

---

## 7. Advertencia de versión

Esta página es de la **v0.5.5** de la documentación de ODrive. El firmware de
los GIM6010-8 es una variante de ODrive mantenida por SteadyWin: la estructura
de los lazos y los nombres de los parámetros son los mismos, pero los valores
por defecto y el juego de comandos CAN los fija el manual del motor
(`SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf`, traducido en
`../manual_traducido/`). Ante una discrepancia, manda el manual del motor.
