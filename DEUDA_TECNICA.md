# Deuda tecnica

Cosas conocidas que NO estan resueltas, con la evidencia que hay hasta hoy y
lo que ya se descarto. La idea es no volver a probar dos veces la misma
hipotesis muerta.

Formato de cada punto: **sintoma**, **evidencia**, **descartado**, **proximo
paso**. Si algo se resuelve, se borra de aca y se documenta donde vive el
cambio (normalmente un comentario en `config/base.yaml`).

Ultima revision: 1-ago-2026.

---

## 1. El enlace con el ESP32 se cae en plena marcha

Es lo unico que provoco una falla dura de navegacion en tres dias.

**Sintoma.** `SerialTimeoutException` al escribir, `/dev/esp32` desaparece,
3 aperturas fallidas seguidas, reset USB automatico. Detras vienen los
watchdogs por falta de Twist, el EKF tardando 0.327 s por ciclo (contra
~0.04 normal) y slam_toolbox descartando scans por cola llena. La corrida
0/3 del 31-jul a las 21:58 es el ejemplo.

**Evidencia importante: el ESP32-S2 tiene USB NATIVO**, no hay puente
CP2102 ni CH340. Que el dispositivo desaparezca del bus significa que el
CPU se colgo o se reseteo. No es ruido en la linea de datos de un puente:
es el micro cayendose.

**Descartado:**

- *Cable.* El cable actual es de buena calidad y uno NUEVO dio peor
  resultado: ni siquiera enumeraba el ESP32. La hipotesis "cable malo" (que
  fue la causa real del episodio equivalente del 9-jul) ya no explica esto.
- *Pico de corriente por el golpe de PWM 226 en el avance.* Fue la primera
  sospecha al ver la caida durante una prueba con
  `wheel_breakaway_on_drive: true`. Es DEBIL y quedo refutada: el golpe de
  PIVOTE dispara los mismos 226 una vez por giro de extremo, con frecuencia
  parecida, y corrio 203 s sin caerse. Ademas el enlace volvio a caerse
  despues con `on_drive: false`, o sea sin correlacion.

**Hipotesis viva.** Brownout en la alimentacion del ESP32 cuando arrancan
los motores. Encaja con el USB nativo (brownout -> reset del micro ->
desaparece del bus) y con la impresion de Diego de que "suena a voltaje o
picos". El L293 mueve 600 mA - 1 A por canal y tiene apagado termico.

**Proximo paso.**

1. Condensador de bulk (1000 uF electrolitico + 100 nF ceramico) lo mas
   cerca posible de los pines de alimentacion del ESP32.
2. Separar la alimentacion del ESP32 de la de los motores, o al menos
   asegurar que no comparten el retorno de masa por una pista larga.
3. Asegurar/soldar el conector de motores, que esta flojo.

---

## 2. El voltaje de bateria no lo puede leer el software

CORRECCION del 1-ago: la version anterior de este documento decia "no hay
monitoreo de bateria". Es falso. El robot LLEVA un voltimetro de panel de
7 segmentos, visible en las fotos del chasis, marcando p.ej. 11.6 V. Lo que
no existe es lectura POR SOFTWARE: nada de eso llega a un topico ni queda
registrado junto a las mediciones.

**Por que importa.** Ya causo DOS diagnosticos falsos en dos dias. A 11.7 V
el robot anda 13% mas lento (0.1069 contra 0.0928 m/s) y eso se le
atribuyo primero a `base_width` y despues a los tiempos del pasillo. Que el
numero este a la vista en el chasis no sirve si nadie lo anota: en una
prueba automatica de 10 tramos no hay quien lo mire.

**Proximo paso.** Divisor resistivo a un pin ADC del S2 y agregarlo a la
telemetria. Cambia el firmware, o sea que depende del punto 4.

**Mientras tanto:** anotar el voltaje a mano antes de cada tanda de
mediciones. Toda calibracion se hace A BATERIA y desenchufado de la pared
(con el cable de la fuente puesto la velocidad cae de 0.147 a 0.033 m/s
porque el cable retiene el robot; eso invalido calibraciones viejas).

---

## 3. Patinaje residual: los encoders exageran la rotacion

**Sintoma.** En las pruebas de recta del 1-ago los encoders reportaron
entre 1.18x y 2.28x el giro que midio el gyro. Una vez, el 31-jul, con el
SIGNO CAMBIADO: la odometria dijo +25.6 grados a la izquierda y el gyro
-14.8 a la derecha, y el robot efectivamente choco por la derecha.

**Ya corregido lo que era corregible por parametro.** `base_width`
0.145 -> 0.175 y `wheel_radius` 0.035 -> 0.034 bajaron el error de
1.52x/1.66x a ~1.38x. Lo que queda es patinaje mecanico real.

**Consecuencia practica.** La odometria de ruedas NO sirve sola para
orientacion. El gyro y el EKF son los que mantienen el robot apuntado, y
coincidieron entre si en las cuatro corridas donde se los comparo. Nav2
navega bien porque el RPP cierra el lazo sobre el rumbo con el gyro.

**No se arregla con trim ni con un PI de velocidad de rueda.** Un escalar
no puede corregir un error que depende del sentido de marcha, del PWM, y
que cambia de signo entre corridas identicas. Se propuso trim de 0.898,
0.935, 0.948 y 0.922 en distintos momentos: los cuatro se cayeron.

**Proximo paso.** Es mecanico: limpiar las ruedas y revisar juego en los
ejes. Baja prioridad mientras el gyro lo tape.

---

## 4. No se puede recompilar el firmware

`arduino-cli 1.5.1` esta instalado en `/home/diego/tools/bin` (NO esta en
el PATH), con el core `esp32:esp32@3.3.11`. FQBN
`esp32:esp32:lolin_s2_mini`, puerto `/dev/esp32` -> `ttyACM0`.

**Dos bloqueos:**

1. **El `.ino` del proyecto no esta en la Pi.** Lo unico que hay son los
   ejemplos del core. Sin el fuente no se puede recompilar nada.
2. **El S2 tiene USB nativo y hay que entrar al bootloader a mano**:
   mantener `0`/BOOT apretado y pulsar RST. No se puede flashear
   remotamente ni desde un script.

Esto bloquea el punto 2 (voltaje de bateria) y el punto 8 (rampa de PWM).

---

## 5. El robot roza el obstaculo bajo: el planner solo despeja 11 cm

Con la camara funcionando (1-ago) el robot DETECTA el obstaculo bajo y
traza un rodeo, pero la curva sale demasiado poco pronunciada y lo roza.
Se recupera solo, pero lo roza.

**Causa.** `SmacPlanner2D` trata al robot como un PUNTO: no comprueba el
footprint contra el mapa, solo se niega a entrar en celdas con coste >= 253
(*inscribed*). Ese radio lo fija el footprint, 0.11 m de semiancho, asi que
el planificador **solo garantiza despejar 11 cm** del centro de la celda
marcada. El `inflation_radius` de 0.55 no lo empuja lejos, solo encarece el
paso sin prohibirlo:

    a 0.15 m del obstaculo el coste todavia es 224 -> el planner pasa
    a 0.20 m                                    193 -> pasa
    a 0.25 m                                    166 -> pasa

Y encima la camara marca la CARA DELANTERA del objeto, no su volumen, asi
que 11 cm desde esa cara no libran el cuerpo entero.

**Proximo paso.** El boton es `cost_travel_multiplier` (hoy 5.0,
`nav2_params.yaml:188`): sube el peso del coste frente a la distancia, o
sea cuanto le duele al planner rozar la inflacion. Es el mismo parametro
que el 31-jul se subio de 3.0 a 5.0 para centrarlo en el paso estrecho.
OJO: afecta TODAS las rutas, incluida la del paso estrecho que ya cuesta,
asi que se prueba con el pasillo completo, no solo con el obstaculo.

NO tocar `footprint_padding` para esto: agrandaria el radio inscrito y
cerraria el paso estrecho, que tiene 0.40 m de hueco para un robot de
0.26 m (7 cm por lado).

---

## 6. El costmap local acumula marcas con el robot quieto

Medido el 1-ago durante la prueba estatica de la camara: con el robot
INMOVIL, las celdas letales dentro de 1 m subieron de 104 a 147 a lo largo
de unos tres minutos, sin que nadie moviera nada en los laterales.

**Por que importa.** Es la sospecha mas firme de por que una corrida del
pasillo dio 0/3 abortos a las 19:31 y la misma prueba, en las mismas
condiciones, dio 4/4 cuatro minutos despues: el robot habria arrancado
encerrado en su propio costmap. El sintoma que lo delata es el tiempo de
posicionamiento inicial, 17.5 s en la que fallo contra 0.1-3.7 s en las que
salieron bien.

`observation_persistence`, cuyo razonamiento completo esta en el comentario
de la fuente `camera` del costmap local en `nav2_params.yaml`, empuja en la
MISMA direccion: conviene tenerlo presente al tocarlo.

---

## 7. Bugs y ruido conocidos

- **`_reader_loop/read` lanza `TypeError: 'NoneType'`.** Aparece en los
  logs; el manejo de excepciones lo captura (`base_driver.py:531-533`) asi
  que no rompe nada, pero ensucia y esconde el error real.
- **`Inflation layer not found` al arrancar Nav2.** Error de startup, la
  navegacion funciona igual. Sin diagnosticar.
- **`Failed to get parameters: parameter 'ApproachZone.max_points' is not
  initialized`**, cada 15 s. Es RUIDO, no un fallo, y de hecho confirma que
  se esta usando el parametro correcto: Nav2 1.3.12 declara `max_points`
  sin valor por defecto para compatibilidad hacia atras, intenta leerlo, y
  como el config usa `min_points` (el moderno) la lectura falla. El binario
  lo dice: *"max_points parameter was deprecated. Use min_points instead"*.
  Sale de `[rclcpp]`, no del collision_monitor, y cada 15 s clavados, o sea
  que lo dispara alguien enumerando parametros desde afuera --
  probablemente el poll de `foxglove_bridge`. Comprobable: sin Foxglove no
  deberia aparecer.
- **`Message Filter dropping message: frame 'base_laser' ... the timestamp
  on the message is earlier than all the data in the transform cache`.** Es
  el LIDAR, no la camara. Aislado, unas pocas veces por sesion, en los dos
  costmaps. Sin diagnosticar.
- **Comentario contradictorio en `config/base.yaml`.** El bloque sobre el
  PI dice *"Motivo del apagado de PI y trim"*, pero el trim NUNCA se apago:
  `left_feedforward_scale` sigue en 0.915 y esta activo. Cualquiera que lea
  ese comentario va a creer que no hay correccion de trim aplicada. Es solo
  un comentario, pero enganya.

---

## 8. Rampa de PWM: disenada, sin implementar

La idea es reemplazar el golpe de valor fijo (226) por una BUSQUEDA del
minimo PWM que rompe la friccion, subiendo escalones hasta que la
telemetria confirme movimiento. Ventaja: menos pico de corriente que el
golpe fijo, que es justo lo que sospechamos en el punto 1.

Requiere subir `wheel_breakaway_timeout` y dejar que el driver escriba por
serie a 20 Hz durante el golpe (hoy escribe a la frecuencia de Nav2, 10 Hz).

---

## 9. La password de sudo es trivial

Son tres digitos consecutivos. Ya estaba anotada como algo a cambiar en las
notas del proyecto y sigue igual. No se escribe aca: el repo
`github.com/DiegoiSaavedra/ros2_ws` es PUBLICO.

Recordatorio general: revisar credenciales antes de cada push.

---

## Nota sobre `ticks_per_revolution`

No es deuda, es una aclaracion que conviene tener a mano: desde el
1-ago-2026 vale 4363 y **es una constante de calibracion, no un conteo
fisico de ticks**. En la prueba que la fijo, `wheel_radius` y
`ticks_per_revolution` son indistinguibles (solo entra el producto
`2*pi*r/TPR`). El detalle completo esta en el comentario de
`config/base.yaml`.
