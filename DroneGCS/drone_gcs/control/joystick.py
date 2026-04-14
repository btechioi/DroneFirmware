"""Joystick Controller for Drone GCS - Gamepad/joystick input handling"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable

try:
    import pygame
    from pygame import joystick as pygame_joystick
    from pygame import event as pygame_event

    joystick = pygame_joystick
    event = pygame_event
    PygameAvailable = True
except ImportError:
    PygameAvailable = False
    pygame = None
    joystick = None
    event = None

logger = logging.getLogger(__name__)


class JoystickAxis(Enum):
    ROLL = 0
    PITCH = 1
    THROTTLE = 2
    YAW = 3
    AUX1 = 4
    AUX2 = 5
    AUX3 = 6
    AUX4 = 7


@dataclass
class JoystickState:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1000
    yaw: int = 1500
    aux1: int = 1000
    aux2: int = 1000
    aux3: int = 1000
    aux4: int = 1000
    armed: bool = False
    buttons: list[bool] = None

    def __post_init__(self):
        if self.buttons is None:
            self.buttons = []


class JoystickController:
    DEAD_ZONE = 0.1
    THROTTLE_CENTER = 1000

    def __init__(self) -> None:
        if not PygameAvailable:
            logger.warning("pygame-ce not available - joystick control disabled")
            return

        pygame.init()
        joystick.init()

        self._joystick: Optional[object] = None
        self._num_joysticks = joystick.get_count()
        self._active = False
        self._last_update = 0

        self._axis_mapping: dict[int, JoystickAxis] = {
            0: JoystickAxis.ROLL,
            1: JoystickAxis.PITCH,
            2: JoystickAxis.THROTTLE,
            3: JoystickAxis.YAW,
        }

        self._button_mapping: dict[int, Callable] = {}
        self._on_state_change: Optional[Callable[[JoystickState], None]] = None

        self._last_state = JoystickState()

        self._expo = 0.2
        self._throttle_curve = True

    @property
    def num_joysticks(self) -> int:
        return self._num_joysticks

    @property
    def is_active(self) -> bool:
        return self._active and self._joystick is not None

    @property
    def joystick_name(self) -> str:
        if self._joystick:
            return self._joystick.get_name()
        return "No joystick"

    def set_state_callback(self, callback: Callable[[JoystickState], None]) -> None:
        self._on_state_change = callback

    def set_axis_mapping(self, axis: int, mapping: JoystickAxis) -> None:
        self._axis_mapping[axis] = mapping

    def set_button_mapping(self, button: int, callback: Callable) -> None:
        self._button_mapping[button] = callback

    def set_expo(self, value: float) -> None:
        self._expo = max(0.0, min(1.0, value))

    def set_throttle_curve(self, enabled: bool) -> None:
        self._throttle_curve = enabled

    def get_joysticks(self) -> list[dict]:
        joysticks = []
        for i in range(self._num_joysticks):
            js = joystick.Joystick(i)
            joysticks.append(
                {
                    "id": i,
                    "name": js.get_name(),
                    "num_axes": js.get_numaxes(),
                    "num_buttons": js.get_numbuttons(),
                    "num_hats": js.get_numhats(),
                }
            )
        return joysticks

    def select_joystick(self, index: int) -> bool:
        if index < 0 or index >= self._num_joysticks:
            return False

        self._joystick = joystick.Joystick(index)
        self._joystick.init()
        self._active = True

        logger.info(f"Selected joystick: {self._joystick.get_name()}")
        logger.info(f"  Axes: {self._joystick.get_numaxes()}")
        logger.info(f"  Buttons: {self._joystick.get_numbuttons()}")
        logger.info(f"  Hats: {self._joystick.get_numhats()}")

        return True

    def disconnect(self) -> None:
        if self._joystick:
            self._joystick.quit()
            self._joystick = None
        self._active = False

    def update(self) -> Optional[JoystickState]:
        if not self._active or not self._joystick:
            return None

        state = JoystickState()

        for axis_idx, axis_mapping in self._axis_mapping.items():
            if axis_idx < self._joystick.get_numaxes():
                raw_value = self._joystick.get_axis(axis_idx)
                value = self._apply_deadzone(raw_value)
                pwm = self._map_to_pwm(value, axis_mapping)

                if axis_mapping == JoystickAxis.ROLL:
                    state.roll = pwm
                elif axis_mapping == JoystickAxis.PITCH:
                    state.pitch = pwm
                elif axis_mapping == JoystickAxis.THROTTLE:
                    state.throttle = pwm
                elif axis_mapping == JoystickAxis.YAW:
                    state.yaw = pwm
                elif axis_mapping == JoystickAxis.AUX1:
                    state.aux1 = pwm
                elif axis_mapping == JoystickAxis.AUX2:
                    state.aux2 = pwm
                elif axis_mapping == JoystickAxis.AUX3:
                    state.aux3 = pwm
                elif axis_mapping == JoystickAxis.AUX4:
                    state.aux4 = pwm

        num_buttons = self._joystick.get_numbuttons()
        state.buttons = [self._joystick.get_button(i) for i in range(num_buttons)]

        if num_buttons >= 4:
            state.aux1 = 2000 if state.buttons[0] else 1000
            state.aux2 = 2000 if state.buttons[1] else 1000
            state.aux3 = 2000 if state.buttons[2] else 1000
            state.aux4 = 2000 if state.buttons[3] else 1000

        if num_buttons >= 7:
            state.armed = state.buttons[6] and state.throttle < 1050
        else:
            state.armed = state.aux1 == 2000 and state.throttle < 1050

        for hat_idx in range(self._joystick.get_numhats()):
            hat = self._joystick.get_hat(hat_idx)
            hat_x, hat_y = hat
            if hat_idx == 0:
                if hat_x < 0:
                    state.aux1 = 1000
                elif hat_x > 0:
                    state.aux1 = 2000

        self._last_state = state

        if self._on_state_change:
            self._on_state_change(state)

        return state

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.DEAD_ZONE:
            return 0.0
        sign = 1 if value > 0 else -1
        adjusted = (abs(value) - self.DEAD_ZONE) / (1.0 - self.DEAD_ZONE)
        return sign * adjusted

    def _apply_expo(self, value: float) -> float:
        return value * (1.0 + self._expo * (value * value - 1.0))

    def _map_to_pwm(self, value: float, axis: JoystickAxis) -> int:
        value = self._apply_expo(value)

        if axis == JoystickAxis.THROTTLE:
            if self._throttle_curve:
                value = (value + 1.0) / 2.0
                value = value**1.5
                value = value * 2.0 - 1.0

            pwm = int(1500 + value * 500)
            return max(1000, min(2000, pwm))
        else:
            pwm = int(1500 + value * 500)
            return max(1000, min(2000, pwm))

    def get_last_state(self) -> JoystickState:
        return self._last_state

    def has_joystick(self) -> bool:
        return self._num_joysticks > 0

    def __del__(self) -> None:
        self.disconnect()


def has_pygame() -> bool:
    return PygameAvailable
