"""Integration Tests for Flight Controller Simulator"""

import unittest
import time
from fc_simulator import (
    FlightControllerSimulator,
    RCChannels,
    Telemetry,
    FlightMode,
    FailSafeState,
    ControlSource,
)


class TestFlightControllerIntegration(unittest.TestCase):
    def test_initial_state(self):
        fc = FlightControllerSimulator()
        self.assertFalse(fc.motors_armed)
        self.assertEqual(fc.current_mode, FlightMode.MANUAL)
        self.assertEqual(fc.failsafe.state, FailSafeState.NONE)

    def test_arm_disarm_sequence(self):
        fc = FlightControllerSimulator()
        fc.arm()
        self.assertTrue(fc.motors_armed)
        fc.disarm()
        self.assertFalse(fc.motors_armed)

    def test_rc_tracking(self):
        fc = FlightControllerSimulator()
        rc = RCChannels(roll=1600, pitch=1400, throttle=1200, yaw=1550)
        fc.receive_rc(rc, 0)
        self.assertEqual(fc.rc.roll, 1600)
        self.assertEqual(fc.rc.pitch, 1400)

    def test_closed_loop_control(self):
        fc = FlightControllerSimulator()
        fc.arm()

        dt = 0.0025
        rc = RCChannels(throttle=1200, roll=1500, pitch=1500, yaw=1500)

        for _ in range(100):
            fc.receive_rc(rc, 0)
            fc.update(dt)

        self.assertLess(fc.imu.roll, 0.5)

    def test_motor_outputs_when_disarmed(self):
        fc = FlightControllerSimulator()
        fc.arm()
        fc.disarm()
        motors = fc.get_motor_outputs()
        self.assertEqual(motors, [1000, 1000, 1000, 1000])

    def test_motor_outputs_when_armed(self):
        fc = FlightControllerSimulator()
        fc.arm()

        rc = RCChannels(throttle=1200)
        fc.receive_rc(rc, 0)

        for _ in range(50):
            fc.update(0.0025)

        motors = fc.get_motor_outputs()
        self.assertTrue(any(m > 1000 for m in motors))

    def test_telemetry_updates(self):
        fc = FlightControllerSimulator()
        fc.arm()
        fc.update(0.0025)
        self.assertEqual(fc.telemetry.armed, fc.motors_armed)

    def test_mode_switch_via_aux(self):
        fc = FlightControllerSimulator()

        rc = RCChannels(aux2=1000)
        fc.receive_rc(rc, 0)
        self.assertEqual(fc.current_mode, FlightMode.MANUAL)

        rc = RCChannels(aux2=2000)
        fc.receive_rc(rc, 0)
        self.assertEqual(fc.current_mode, FlightMode.ALTHOLD)


class TestFlightScenarios(unittest.TestCase):
    def test_hover_scenario(self):
        fc = FlightControllerSimulator()
        fc.arm()

        dt = 0.0025
        rc = RCChannels(throttle=1200, roll=1500, pitch=1500, yaw=1500)

        for _ in range(400):
            fc.receive_rc(rc, 0)
            fc.update(dt)

        motors = fc.get_motor_outputs()
        self.assertTrue(all(1100 <= m <= 1300 for m in motors))

    def test_roll_command(self):
        fc = FlightControllerSimulator()
        fc.arm()

        dt = 0.0025
        rc = RCChannels(throttle=1200, roll=1700, pitch=1500, yaw=1500)

        for _ in range(200):
            fc.receive_rc(rc, 0)
            fc.update(dt)

        self.assertGreater(fc.imu.roll, 0)

    def test_pitch_command(self):
        fc = FlightControllerSimulator()
        fc.arm()

        dt = 0.0025
        rc = RCChannels(throttle=1200, roll=1500, pitch=1700, yaw=1500)

        for _ in range(200):
            fc.receive_rc(rc, 0)
            fc.update(dt)

        self.assertGreater(fc.imu.pitch, 0)

    def test_yaw_command(self):
        fc = FlightControllerSimulator()
        fc.arm()

        dt = 0.0025
        rc = RCChannels(throttle=1200, roll=1500, pitch=1500, yaw=1700)

        for _ in range(200):
            fc.receive_rc(rc, 0)
            fc.update(dt)

        self.assertNotEqual(fc.imu.yaw, 0)


if __name__ == "__main__":
    unittest.main()
