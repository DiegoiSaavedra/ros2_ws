"""Funciones puras para validar y limitar comandos de la base móvil."""

import math
from typing import Tuple


DEFAULT_CMD_VEL_TIMEOUT = 0.25
DEFAULT_SERIAL_RX_TIMEOUT = 0.25
MAX_PWM = 255
VELOCITY_EPSILON = 0.005

# Curva medida el 28-jul-2026 con el robot A BATERIA (enchufado a la fuente
# de pared el cable lo retenia y daba 0.033 m/s en vez de 0.147), con
# escalones de ~6 s para que el motor llegue a regimen:
#   PWM   avance        pivote
#    60   -             0.228 rad/s  (solo una rueda)
#    80   0.012 m/s     0.428 rad/s  (solo una rueda en avance)
#   100   0.046 m/s     0.475 rad/s
#   120   0.064 m/s     0.677 rad/s
#   150   0.090 m/s     1.093 rad/s
#   255   0.147 m/s     ~1.57 rad/s
# Por debajo de PWM 100 en avance solo arranca la rueda derecha, asi que la
# velocidad minima ejecutable con las dos ruedas es ~0.046 m/s: pedir menos
# haria que el robot girase en vez de avanzar. El mapeo reparte el rango
# [deadband, 255] sobre [min_wheel_speed, max_wheel_speed] para que ningun
# comando caiga en esa zona muerta.
DEFAULT_DRIVE_DEADBAND = 100.0
DEFAULT_PIVOT_DEADBAND = 80.0
DEFAULT_MIN_WHEEL_SPEED = 0.040
DEFAULT_MAX_WHEEL_SPEED = 0.147


class WheelSpeedPI:
    """PI de velocidad de una rueda con anti-windup y salida limitada."""

    def __init__(
        self,
        kp: float,
        ki: float,
        correction_limit: float,
        integral_limit: float,
    ):
        self.kp = require_finite(kp, 'PI kp')
        self.ki = require_finite(ki, 'PI ki')
        self.correction_limit = require_finite(
            correction_limit, 'PI correction limit')
        self.integral_limit = require_finite(
            integral_limit, 'PI integral limit')
        if self.kp < 0.0 or self.ki < 0.0:
            raise ValueError('las ganancias PI no pueden ser negativas')
        if self.correction_limit <= 0.0 or self.integral_limit <= 0.0:
            raise ValueError('los límites PI deben ser positivos')
        self.integral = 0.0
        self.last_time = None
        self.error = 0.0
        self.proportional = 0.0
        self.integral_term = 0.0
        self.correction = 0.0
        self.saturated = False

    def reset(self) -> None:
        self.integral = 0.0
        self.last_time = None
        self.error = 0.0
        self.proportional = 0.0
        self.integral_term = 0.0
        self.correction = 0.0
        self.saturated = False

    def update(self, target: float, measured: float, now: float) -> float:
        setpoint = require_finite(target, 'PI target')
        feedback = require_finite(measured, 'PI measured')
        current = require_finite(now, 'PI monotonic time')
        error = setpoint - feedback
        self.error = error
        dt = 0.0
        if self.last_time is not None and current >= self.last_time:
            dt = current - self.last_time
        self.last_time = current

        previous_integral = self.integral
        candidate_integral = previous_integral + error * dt
        candidate_integral = max(
            -self.integral_limit,
            min(self.integral_limit, candidate_integral),
        )
        unsaturated = self.kp * error + self.ki * candidate_integral
        correction = max(
            -self.correction_limit,
            min(self.correction_limit, unsaturated),
        )

        # Si la salida saturada empuja en la misma dirección que el error,
        # conserva el integral anterior. Sí permite descargarlo cuando el
        # error cambia de signo.
        saturated_high = unsaturated > self.correction_limit and error > 0.0
        saturated_low = unsaturated < -self.correction_limit and error < 0.0
        if saturated_high or saturated_low:
            self.integral = previous_integral
            unsaturated = self.kp * error + self.ki * self.integral
            correction = max(
                -self.correction_limit,
                min(self.correction_limit, unsaturated),
            )
        else:
            self.integral = candidate_integral
        self.proportional = self.kp * error
        self.integral_term = self.ki * self.integral
        self.correction = correction
        self.saturated = abs(correction) >= self.correction_limit - 1.0e-9
        return correction


def require_finite(value: float, name: str) -> float:
    """Convierte ``value`` a float y rechaza NaN e infinitos."""
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} no es finito: {converted!r}")
    return converted


def positive_timeout(value: float, default: float) -> float:
    """Devuelve un timeout positivo finito o un valor seguro por defecto."""
    try:
        converted = require_finite(value, 'timeout')
    except (TypeError, ValueError, OverflowError):
        return default
    return converted if converted > 0.0 else default


def check_deadband(deadband: float) -> float:
    """Valida un deadband de PWM y lo devuelve como float."""
    finite_deadband = require_finite(deadband, 'deadband')
    if finite_deadband < 0.0 or finite_deadband >= MAX_PWM:
        raise ValueError(f"deadband fuera de rango: {finite_deadband}")
    return finite_deadband


def twist_to_wheel_velocities(
    linear_x: float,
    angular_z: float,
    base_width: float,
    maximum: float,
) -> Tuple[float, float]:
    """Convierte Twist a ruedas y escala ambas juntas si exceden el máximo."""
    vx = require_finite(linear_x, 'linear.x')
    omega = require_finite(angular_z, 'angular.z')
    width = require_finite(base_width, 'base_width')
    limit = require_finite(maximum, 'maximum wheel speed')
    if width <= 0.0:
        raise ValueError(f"base_width debe ser positivo: {width}")
    if limit <= 0.0:
        raise ValueError(f"maximum wheel speed debe ser positivo: {limit}")

    left = vx - omega * (width / 2.0)
    right = vx + omega * (width / 2.0)
    left = require_finite(left, 'left wheel velocity')
    right = require_finite(right, 'right wheel velocity')

    scale = max(1.0, abs(left) / limit, abs(right) / limit)
    scale = require_finite(scale, 'wheel saturation scale')
    return left / scale, right / scale


def velocity_to_pwm(
    v: float,
    deadband: float,
    min_wheel_speed: float,
    max_wheel_speed: float,
) -> int:
    """Mapea velocidad de rueda a PWM sobre el rango realmente ejecutable.

    El rango útil de PWM [deadband, 255] se reparte sobre el rango de
    velocidad que la base puede sostener con las dos ruedas girando,
    [min_wheel_speed, max_wheel_speed]. Un comando por debajo del mínimo se
    ejecuta al mínimo en vez de quedarse en una zona donde solo arranca una
    rueda; cero sigue siendo cero.
    """
    velocity = require_finite(v, 'wheel velocity')
    finite_deadband = check_deadband(deadband)
    floor_speed = require_finite(min_wheel_speed, 'min wheel speed')
    limit = require_finite(max_wheel_speed, 'max wheel speed')
    if limit <= 0.0:
        raise ValueError(f"max_wheel_speed debe ser positivo: {limit}")
    if floor_speed < 0.0 or floor_speed >= limit:
        raise ValueError(
            f"min_wheel_speed debe estar en [0, {limit}): {floor_speed}")
    if abs(velocity) < VELOCITY_EPSILON:
        return 0

    span = float(MAX_PWM) - finite_deadband
    ratio = (abs(velocity) - floor_speed) / (limit - floor_speed)
    ratio = max(0.0, min(1.0, ratio))
    magnitude = finite_deadband + span * ratio
    magnitude = require_finite(magnitude, 'calculated PWM')
    magnitude = min(magnitude, float(MAX_PWM))
    signed = magnitude if velocity > 0.0 else -magnitude
    signed = require_finite(signed, 'signed PWM')
    return int(signed)


def command_timed_out(now: float, last_command: float, timeout: float) -> bool:
    """Comprueba timeout monotónico; cualquier dato temporal inválido falla seguro."""
    try:
        current = require_finite(now, 'monotonic now')
        previous = require_finite(last_command, 'last command time')
        limit = require_finite(timeout, 'command timeout')
    except (TypeError, ValueError, OverflowError):
        return True
    if limit <= 0.0 or current < previous:
        return True
    return current - previous >= limit


def parse_encoder_line(line: str) -> Tuple[int, int]:
    """Valida un paquete completo ``ticks_left,ticks_right``."""
    parts = line.split(',')
    if len(parts) != 2:
        raise ValueError(f"paquete de encoder con {len(parts)} campos")
    try:
        return int(parts[0]), int(parts[1])
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"ticks de encoder inválidos: {line!r}") from exc
