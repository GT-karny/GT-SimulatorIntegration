# g29_input.py
# Minimal, controller-friendly wrapper around pygame joystick for Logitech wheels.

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import pygame
except ImportError:
    pygame = None


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _deadzone(x: float, dz: float) -> float:
    if abs(x) <= dz:
        return 0.0
    sign = 1.0 if x >= 0.0 else -1.0
    return sign * (abs(x) - dz) / (1.0 - dz)


def _axis_to_01(v: float) -> float:
    # [-1..+1] -> [0..1]
    return _clamp((v + 1.0) * 0.5, 0.0, 1.0)


@dataclass
class G29Mapping:
    """
    Keep this small: only the knobs you realistically tweak per-machine.
    """
    steer_axis: int = 0
    throttle_axis: int = 2
    brake_axis: int = 1

    invert_steer: bool = True
    invert_throttle: bool = False
    invert_brake: bool = False

    steer_deadzone: float = 0.02


class G29:
    """
    Controller-friendly API.

    Usage:
        g29 = G29.auto()                  # simplest
        throttle, brake, steer = g29.read()

    - If pygame is missing or device is not present, read() returns safe defaults.
    - No need to call pygame.init() yourself.
    """

    def __init__(self, mapping: Optional[G29Mapping] = None, device_index: Optional[int] = None):
        self.map = mapping or G29Mapping()
        self._joy = None
        self._ok = False
        self._device_index = device_index

        self._init()

    @staticmethod
    def auto(mapping: Optional[G29Mapping] = None) -> "G29":
        """
        Auto-pick a Logitech/G29 device if present; else pick joystick 0.
        """
        return G29(mapping=mapping, device_index=None)

    def ok(self) -> bool:
        return self._ok and self._joy is not None

    def read(self) -> Tuple[float, float, float]:
        """
        Returns: (throttle[0..1], brake[0..1], steer[-1..1])

        Safe defaults if not available:
            (0.0, 1.0, 0.0)
        """
        if not self.ok():
            return 0.0, 1.0, 0.0

        try:
            pygame.event.pump()
        except Exception:
            return 0.0, 1.0, 0.0

        try:
            # steer
            s = float(self._joy.get_axis(self.map.steer_axis))
            if self.map.invert_steer:
                s = -s
            s = _deadzone(s, self.map.steer_deadzone)
            s = _clamp(s, -1.0, 1.0)

            # pedals (separate axes)
            t_raw = float(self._joy.get_axis(self.map.throttle_axis))
            b_raw = float(self._joy.get_axis(self.map.brake_axis))
            t = _axis_to_01(t_raw)
            b = _axis_to_01(b_raw)

            if self.map.invert_throttle:
                t = 1.0 - t
            if self.map.invert_brake:
                b = 1.0 - b

            return _clamp(t, 0.0, 1.0), _clamp(b, 0.0, 1.0), s

        except Exception as e:
            print(f"[GT-DriveController] G29 read error: {e}")
            return 0.0, 1.0, 0.0

    # ---- internals ----
    def _init(self) -> None:
        if pygame is None:
            print("[GT-DriveController] pygame not installed; G29 disabled")
            return

        try:
            if not pygame.get_init():
                pygame.init()
            if not pygame.joystick.get_init():
                pygame.joystick.init()

            n = pygame.joystick.get_count()
            if n <= 0:
                print("[GT-DriveController] No joystick detected by pygame")
                return

            idx = self._pick_index(n) if self._device_index is None else int(self._device_index)
            idx = 0 if idx < 0 or idx >= n else idx

            self._joy = pygame.joystick.Joystick(idx)
            self._joy.init()

            print(f"[GT-DriveController] Joystick selected: idx={idx}, name={self._joy.get_name()}")
            print(
                f"[GT-DriveController] axes={self._joy.get_numaxes()}, "
                f"buttons={self._joy.get_numbuttons()}, hats={self._joy.get_numhats()}"
            )

            self._ok = True

        except Exception as e:
            print(f"[GT-DriveController] Failed to init joystick: {e}")

    def _pick_index(self, n: int) -> int:
        # Prefer Logitech/G29/G920 if present; else 0
        keywords = ("Logitech", "G29", "G920")
        try:
            for i in range(n):
                name = pygame.joystick.Joystick(i).get_name()
                if any(k in name for k in keywords):
                    return i
        except Exception:
            pass
        return 0
