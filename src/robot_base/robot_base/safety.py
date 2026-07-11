"""Funciones puras para validar y limitar comandos de la base móvil."""

import math
from typing import Tuple


DEFAULT_CMD_VEL_TIMEOUT = 0.25
DEFAULT_SERIAL_RX_TIMEOUT = 0.25
MAX_PWM = 255
PWM_SLOPE = 950.0
VELOCITY_EPSILON = 0.005


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


def wheel_speed_limit(deadband: float) -> float:
    """Máxima velocidad que no supera MAX_PWM con la calibración actual."""
    finite_deadband = require_finite(deadband, 'deadband')
    if finite_deadband < 0.0 or finite_deadband >= MAX_PWM:
        raise ValueError(f"deadband fuera de rango: {finite_deadband}")
    return (MAX_PWM - finite_deadband) / PWM_SLOPE


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


def velocity_to_pwm(v: float, deadband: float) -> int:
    """Aplica la calibración existente; cero nunca recibe deadband."""
    velocity = require_finite(v, 'wheel velocity')
    finite_deadband = require_finite(deadband, 'deadband')
    if finite_deadband < 0.0 or finite_deadband >= MAX_PWM:
        raise ValueError(f"deadband fuera de rango: {finite_deadband}")
    if abs(velocity) < VELOCITY_EPSILON:
        return 0

    magnitude = finite_deadband + abs(velocity) * PWM_SLOPE
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
