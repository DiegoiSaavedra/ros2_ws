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

## 2. No hay monitoreo de bateria

**Por que importa.** Ya causo DOS diagnosticos falsos en dos dias. A 11.7 V
el robot anda 13% mas lento (0.1069 contra 0.0928 m/s) y eso se le
atribuyo primero a `base_width` y despues a los tiempos del pasillo. Sin el
voltaje a la vista no hay forma de saber si una medicion es comparable con
la anterior.

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

Esto bloquea el punto 2 (monitoreo de bateria) y el punto 6 (rampa de PWM).

---

## 5. Bugs menores conocidos

- **`_reader_loop/read` lanza `TypeError: 'NoneType'`.** Aparece en los
  logs; el manejo de excepciones lo captura (`base_driver.py:531-533`) asi
  que no rompe nada, pero ensucia y esconde el error real.
- **`Inflation layer not found` al arrancar Nav2.** Error de startup, la
  navegacion funciona igual. Sin diagnosticar.
- **Comentario contradictorio en `config/base.yaml`.** El bloque sobre el
  PI dice *"Motivo del apagado de PI y trim"*, pero el trim NUNCA se apago:
  `left_feedforward_scale` sigue en 0.915 y esta activo. Cualquiera que lea
  ese comentario va a creer que no hay correccion de trim aplicada. Es solo
  un comentario, pero enganya.

---

## 6. Rampa de PWM: disenada, sin implementar

La idea es reemplazar el golpe de valor fijo (226) por una BUSQUEDA del
minimo PWM que rompe la friccion, subiendo escalones hasta que la
telemetria confirme movimiento. Ventaja: menos pico de corriente que el
golpe fijo, que es justo lo que sospechamos en el punto 1.

Requiere subir `wheel_breakaway_timeout` y dejar que el driver escriba por
serie a 20 Hz durante el golpe (hoy escribe a la frecuencia de Nav2, 10 Hz).

---

## 7. La password de sudo es trivial

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
