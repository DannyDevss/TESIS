# Identificación de los motores GIM6010-8 y cálculo de las ganancias PI

Herramientas para sacar `vel_gain`, `vel_integrator_gain` y `pos_gain` de una
**medida**, en vez de por prueba y error. Cubre las dos familias de motores del
robot, que son plantas distintas y necesitan juegos de ganancias distintos:

| | IDs | modo en el robot | planta | qué se identifica |
|---|---|---|---|---|
| Orugas | 1-4 | velocidad | primer orden, gira libre | `J`, `b`, `Tc` |
| Flippers | 5-8 | posición | segundo orden, la gravedad hace de muelle | `J`, `b`, `k`, `Tc` |

```
registrar_ensayo.py   excita un motor en PAR y guarda la respuesta a CSV
identificar.py        ajusta el modelo y calcula las ganancias   (Python)
identificar.m         lo mismo en MATLAB, para las figuras de la memoria
datos/                CSV + JSON de metadatos + PNG de cada ensayo
```

---

## Lo primero: qué NO hace falta identificar

**El lazo de corriente ya está resuelto por la calibración del motor.** El
firmware es ODrive y calcula sus propias ganancias a partir de la resistencia y
la inductancia medidas:

```
Kp = current_control_bandwidth * L
Ki = current_control_bandwidth * R
```

Por eso el botón de calibrar devuelve R y L. El único parámetro que se elige es
el ancho de banda (`控制带宽` en el Motor Wizard, `current_control_bandwidth` en
odrivetool, 1000 rad/s de fábrica). Hacer identificación de sistemas ahí sería
medir dos veces lo mismo, y además no se podría: la telemetría CAN va a 100 Hz y
el lazo de corriente vive tres órdenes de magnitud más arriba.

Lo que sí conviene es **verificar** la calibración. El datasheet da 0,48 Ω y
368 µH *entre fases*; según cómo lo reporte el firmware puede haber un factor 2
(por fase frente a entre fases). No merece la pena perseguir ese factor. La
comprobación que sí importa es otra: **los cuatro motores tienen que devolver R y
L parecidos entre sí**. Si uno se sale un 20-30 %, es conector, cable o
soldadura, no sintonía.

Lo que queda por medir es la parte mecánica, y ahí sólo hay una incógnita que
manda: la inercia `J`. Con ella, la ganancia sale de una fórmula.

---

## Unidades: por qué todo va en crudo

Los CSV guardan revoluciones, rev/s y par **tal como viajan por el bus**, sin
convertir a radianes ni al eje de salida. Dos razones:

1. `RELACION_REDUCCION = 8.0` sigue marcada como *sin verificar en hardware* en
   `src/ugv_bridge/ugv_bridge/protocolo_can.py`. Meter una constante no
   verificada en los datos de identificación los contaminaría sin dejar rastro.
2. No hace falta. `vel_gain` del driver opera en **sus** unidades (par por rev/s
   de error), así que si `J` se identifica en esas mismas unidades la ganancia
   sale directa:

   ```
   vel_gain = J * wc          (wc en rad/s)
   ```

   Sin ningún `2*pi` y sin el 8. Para la memoria, la conversión al SI en el mismo
   eje es `J_SI = J / (2*pi)`.

Con el par pasa algo parecido: el manual dice que en el control normal por CAN la
posición, la velocidad y el par se refieren al **rotor** (sólo el modo MIT usa el
eje de salida), pero la constante de par del datasheet (0,47 N·m/A) no cuadra
del todo con los 8,27/Kv = 0,67 N·m/A que tocarían en el rotor. Da igual para
esto: mientras el par que se manda y la ganancia que se calcula estén en la misma
unidad, el resultado es coherente. Lo que **no** se puede hacer es mezclar estos
números con N·m del datasheet sin resolver antes esa ambigüedad.

> Y ya que se está: la verificación del 8:1 son dos minutos. Con el motor en
> IDLE, girar el eje de SALIDA una vuelta exacta a mano y mirar cuánto cambia
> `pos_estimate`. Si cambia 8, la constante es correcta; si cambia 1, el encoder
> ya mide en la salida. Afecta también a la odometría.

---

## Seguridad

Los ensayos corren el motor en **modo par**: sin lazo de posición ni de
velocidad. El regulador lento que lleva el script es una red, no un freno; si el
proceso muere, lo único que queda es el límite que se fijó en el driver por CAN.

- **Orugas**: el robot LEVANTADO, con la oruga girando en el aire, o el motor
  solo en el banco. Nunca apoyado en el suelo.
- **Flippers**: el recorrido está limitado por topes mecánicos. Antes de empezar
  hay que saber dónde están y dejar el flipper a media carrera. `--limite-pos`
  es el margen, y lo pone quien mira el motor.

Abortan el ensayo, con par 0 + IDLE inmediatos: excursión fuera de
`--limite-pos`, velocidad sobre `--limite-vel`, `axis_error` distinto de 0 en el
heartbeat, 200 ms sin tramas de encoder, y Ctrl+C. El motivo queda anotado en el
JSON.

---

## Procedimiento

### Paso 0 — Probar la cadena entera sin hardware

Hay una planta simulada dentro del script. Antes de energizar nada, conviene
comprobar que el análisis recupera los parámetros que se le metieron:

```bash
python3 identificacion/registrar_ensayo.py --motor 1 --perfil chirp --simular
python3 identificacion/registrar_ensayo.py --motor 1 --perfil prbs  --simular
python3 identificacion/identificar.py 'identificacion/datos/m1_*chirp_sim*.csv' \
    --validar 'identificacion/datos/m1_*prbs_sim*.csv'
```

El informe trae una sección «Contraste con la planta simulada» que compara lo
estimado con lo verdadero. Los órdenes de magnitud a esperar: con la oruga, `J`
con un error del orden del 1 % y validación cruzada por encima del 75 %; con el
flipper, `J` en torno al 10 %, `k` al 15 %, `Tc` al 10 % y validación parecida.
`b` es siempre el peor de todos (puede irse al 50 % en el flipper) porque se
canjea con la fricción de Coulomb; no importa, porque las ganancias salen de `J`.
Si esos números no salen, algo se rompió en el análisis y no tiene sentido
llevarlo al banco.

### Paso 1 — Fricción (sólo orugas; primero, y es el que más ahorra tiempo después)

```bash
python3 identificacion/registrar_ensayo.py --motor 1 --perfil friccion --canal can0
python3 identificacion/identificar.py 'identificacion/datos/m1_*friccion*.csv'
```

Una escalera de par constante; de cada peldaño se toma la velocidad en régimen y
de la nube sale la recta `b*w + Tc`. Lo que interesa de aquí es **Tc**, la
fricción de Coulomb del reductor 8:1: es el par mínimo para que el eje se mueva.
Marca dos cosas:

- el suelo de la zona muerta, que ninguna ganancia arregla;
- la amplitud mínima de la excitación en los pasos siguientes, que tiene que ser
  del orden de **3·Tc** para no perderse dentro del rozamiento.

Este perfil es **sólo para las orugas**. La escalera aplica par constante sin
regulador —tiene que ser así, porque el punto de operación es justo la
incógnita— y en un flipper eso lo lleva contra el tope en el primer peldaño; el
script lo rechaza con un mensaje. En el flipper no hace falta: como su excitación
cruza el cero de velocidad, `Tc` sale identificado en el propio chirp del paso 2.

Cada peldaño tiene que llegar al régimen permanente: `--t-paso` (4 s por defecto)
debe ser varias veces la constante de tiempo `J/b`. El analizador descarta los
peldaños que aún derivan y dice cuántos tiró. Si descarta muchos, subir
`--t-paso`. No es un detalle menor: con peldaños de 1,5 s sobre una planta de
0,6 s de constante de tiempo, el error en `Tc` pasaba del 25 %; con 4 s baja al
1 %.

### Paso 2 — Dinámica (chirp)

```bash
python3 identificacion/registrar_ensayo.py --motor 1 --perfil chirp --canal can0
```

Barrido logarítmico de 1 a 20 Hz. La amplitud se calcula sola: el script hace
antes una sonda que estima `J`, y de ahí sale la amplitud que produce el balanceo
de velocidad pedido. Si el paso 1 dio un `Tc` alto, forzarla con
`--amplitud $(3*Tc)`.

### Paso 3 — Validación (PRBS)

```bash
python3 identificacion/registrar_ensayo.py --motor 1 --perfil prbs --canal can0
```

**Con otra señal.** Un modelo que ajusta al 95 % la señal con la que se ajustó y
al 40 % una distinta no vale, y sólo se ve haciendo esto.

### Paso 4 — Ajuste y ganancias

```bash
python3 identificacion/identificar.py datos/m1_oruga_chirp_....csv \
    --validar datos/m1_oruga_prbs_....csv --bw 100
```

Escupe los parámetros físicos, la calidad del ajuste, una tabla de ganancias para
varios anchos de banda y los comandos de odrivetool listos para copiar. Además
deja un PNG con entrada, respuesta medida contra modelo, y la respuesta en
frecuencia empírica contra la del modelo.

En MATLAB, lo mismo:

```matlab
res = identificar('datos/m1_oruga_chirp_....csv', 'validar', 'datos/m1_oruga_prbs_....csv');
```

No necesita la System Identification Toolbox (el ajuste es un `\`); si está
instalada, contrasta con `tfest` y avisa si los dos caminos discrepan más del
35 %.

### Paso 5 — Repetir para los ocho motores

Los cuatro motores de oruga deberían dar parámetros parecidos entre sí, y los
cuatro de flipper también. **Un motor que se desvía del resto de su familia es un
hallazgo**, no ruido: reductor más apretado, cable peor, rodamiento tocado.

### Paso 6 — Verificar en el motor

Las ganancias que salen son el punto de partida medido, no el final. Con ellas
puestas, un escalón de velocidad y mirar el sobreimpulso en el gráfico (Motor
Wizard o `identificar.py` sobre un ensayo en velocidad). Si oscila, bajar. El
método del manual (subir un 30 % hasta vibración, bajar un 50 %) sigue valiendo
como refinamiento; la diferencia es que ahora se arranca de un número con
fundamento y se hacen dos iteraciones en vez de veinte.

---

## Cómo se eligen las ganancias

Con `J` en unidades del driver y un ancho de banda objetivo `wc` en rad/s:

```
vel_gain            = J * wc
vel_integrator_gain = 0.25 * wc * vel_gain      (cero del PI en wc/4)
pos_gain            = wc / 4                     (sólo flippers)
```

- **`vel_gain = J * wc`**: el lazo abierto es `vel_gain/(J*s)` y cruza 0 dB
  justo en `wc`.
- **El integrador se coloca por margen de fase**, no por receta. El cero del PI
  en `wc/4` quita unos 14° de fase en el cruce, peaje asumible a cambio de matar
  el error estacionario. El manual propone `0,5 * ancho_de_banda * vel_gain` sin
  decir si el ancho de banda va en Hz o en rad/s, y entre las dos lecturas hay un
  factor 2π: la de rad/s deja el cero en `wc/2` (agresivo) y la de Hz en `wc/12`
  (conservador). El informe calcula las tres para comparar.
- **Separación de lazos en cascada**: corriente 1000 rad/s ≫ velocidad 50-150
  rad/s ≫ posición 10-30 rad/s. Y `wc` no debería pasar de una décima parte de
  la frecuencia de muestreo en rad/s: con la telemetría a 100 Hz, unos 60 rad/s
  con holgura. Se puede duplicar por USB con
  `odrv0.axis0.config.can.encoder_rate_ms = 5`.

---

## Decisiones de diseño que no son obvias

**Se excita en par, no en velocidad.** Excitando en velocidad se mide el lazo
cerrado con las ganancias actuales, que son justo lo que se quiere calcular. En
par, el driver sólo cierra el lazo de corriente y lo que se ve es la planta
desnuda.

**Hay un regulador lento sosteniendo el punto de operación.** En par puro, una
oruga en el aire es un integrador: al primer semiciclo lento del chirp se embala
y muere contra el límite de velocidad (pasó en la propia simulación, a los
1,6 s). El flipper tiene el problema simétrico con la gravedad. El regulador
trabaja a 0,4 rad/s, más de una década por debajo de la banda de identificación,
y se le realimenta la velocidad filtrada para que dentro de esa banda casi no
responda. Aun así su par **es** parte de la entrada de la planta, y por eso el
CSV guarda `par_cmd` (total) además de `par_exc`, y el ajuste usa el total.

**El regulador arranca en dos etapas.** El lento tarda más de un minuto en juntar
el par que vence la fricción de arranque, así que primero manda uno rápido y
después le pasa su integrador al lento, que arranca ya con el par de sostén
correcto.

**La oruga se identifica girando, no alrededor de cero.** En el cero la fricción
de Coulomb es una discontinuidad y ningún modelo lineal la describe. Girando
siempre en el mismo sentido es localmente una recta. El precio es que `Tc` deja
de ser identificable en ese ensayo (se confunde con un par constante) — de ahí
que el paso 1 exista y vaya aparte. El analizador lo detecta solo y lo dice.

**La sonda de inercia invierte el par sin que el eje cambie de sentido de giro.**
Con el eje girando en positivo, el Coulomb vale siempre `-Tc` gire el par hacia
donde gire, así que al restar las dos aceleraciones desaparecen la fricción, la
gravedad y cualquier otro par constante: `J = 2A/(a1-a2)`. La versión ingenua
—acelerar en un sentido y luego en el otro, cada uno desde el reposo— **no
sirve**: al invertir el sentido de giro el Coulomb cambia de signo con el par y
deja un sesgo de `A/(A-Tc)`, un 25 % largo con estos motores. Es un error fácil
de cometer y difícil de ver, porque el número que sale parece razonable.

**La amplitud del flipper se mide, no se deduce.** El primer intento fue medir la
rigidez de gravedad `k` con rampas de par estáticas y despejar la amplitud. No
funciona: en una rampa estática, `(T+ - T-) = k*Δx + 2*Tc`, o sea que el Coulomb
no se cancela sino que se **duplica**, porque en las dos rampas el eje se mueve
en sentidos opuestos y el rozamiento se opone a los dos. Con los números del
flipper simulado, la `k` así medida salía 6 veces la real. Lo que se hace ahora
es medir directamente la compliancia a la frecuencia más baja del barrido —dos
ciclos de seno pequeños, pico a pico, y escalar— que mide la planta entera tal
como se la va a excitar.

**El ajuste integra la ecuación en vez de derivarla.** Derivar la velocidad para
tener `w'` amplifica el ruido justo en la banda alta, que es donde el chirp
aporta información. Integrando una vez, todos los regresores son integrales de
señales medidas y la integración promedia el ruido:

```
J*(w-w0) + b*∫w + k*∫(x-x0) + Tc*∫tanh(w/eps) - u_off*t = ∫u
```

`u_off` no sobra: absorbe un par constante no modelado. Sin él, el ajuste lo
reparte entre `J` y `b` y los sesga a los dos.

**El «fit» se da de dos maneras.** La simulación libre de 25 s es dura y algo
injusta con estas plantas: con `b/J` del orden de 2 rad/s la oruga es casi un
integrador, su ganancia en continua es enorme y un error minúsculo en el par
constante se convierte en una deriva grande al cabo de medio minuto. Un modelo
puede describir perfectamente la dinámica y sacar un fit pobre por esa deriva.
Por eso se da también el fit reenganchando el estado medido cada 2 s, que es el
que hay que mirar para decidir.

---

## Cosas del robot que conviene tener presentes

- **La inercia de las orugas cambia con el terreno.** No merece la pena perseguir
  un `J` exacto; sintonizar con margen y **validar con el robot apoyado, no en el
  aire**, que es otro sistema.
- **`pos_gain` casi no importa en las orugas**: el puente las manda en velocidad
  (`CONTROL_VELOCIDAD` + `ENTRADA_PASSTHROUGH` en `driver_movimiento`). Donde sí
  cuenta es en los flippers, que van en posición.
- **Windup**: con `vel_integrator_gain` alto y una oruga bloqueada contra un
  obstáculo, el integrador se carga y da un tirón al liberarse. Con una política
  de RL mandando velocidades eso es un riesgo real; si aparece, bajar el
  integrador antes que subir el proporcional.
- **`input_filter_bandwidth`**, si se usa filtro de entrada: la mitad de la
  frecuencia de comandos. A 100 Hz de bucle, 50.

---

## Transporte

`--canal can0` (SocketCAN, adaptador USB-CAN) es lo preferido: `msg.timestamp` es
la marca de tiempo que pone el **kernel** al recibir la trama, no la que pone el
proceso al leerla. Unos milisegundos de jitter en el eje de tiempos es
exactamente el ruido que peor lleva un ajuste de modelo.

`--pi3hat` funciona y no necesita adaptador, pero entrega todas las tramas de un
ciclo SPI con la misma marca de tiempo, así que el eje de tiempos queda
cuantizado al periodo del bucle. Si se usa, bajar `--rate-hz` y no fiarse de la
banda alta del ajuste.

`--canal vcan0` contra `motor_emulator` sirve para probar el transporte y el
protocolo, pero el emulador no implementa modo par: no da datos útiles. Para
probar el análisis, `--simular`.
