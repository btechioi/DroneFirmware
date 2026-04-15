"""Test RC Protocol"""

import unittest
from fc_simulator import RCChannels, FlightMode, ControlSource, FailSafeState


class TestRCChannels(unittest.TestCase):
    def test_default_values(self):
        rc = RCChannels()
        self.assertEqual(rc.roll, 1500)
        self.assertEqual(rc.pitch, 1500)
        self.assertEqual(rc.throttle, 1000)
        self.assertEqual(rc.yaw, 1500)
        self.assertEqual(rc.aux1, 1000)

    def test_custom_values(self):
        rc = RCChannels(roll=1600, pitch=1400, throttle=1200, yaw=1550)
        self.assertEqual(rc.roll, 1600)
        self.assertEqual(rc.pitch, 1400)
        self.assertEqual(rc.throttle, 1200)
        self.assertEqual(rc.yaw, 1550)

    def test_to_bytes(self):
        rc = RCChannels()
        data = rc.to_bytes()
        self.assertEqual(len(data), 21)
        self.assertEqual(data[0], 0xAA)
        self.assertEqual(data[1], 0x01)
        self.assertEqual(data[2], 8)

    def test_from_bytes(self):
        rc = RCChannels(roll=1600, pitch=1400, throttle=1200, yaw=1550)
        data = rc.to_bytes()
        restored = RCChannels.from_bytes(data)
        self.assertEqual(restored.roll, 1600)
        self.assertEqual(restored.pitch, 1400)
        self.assertEqual(restored.throttle, 1200)
        self.assertEqual(restored.yaw, 1550)

    def test_crc_integrity(self):
        rc = RCChannels(roll=1800, pitch=1200)
        data = rc.to_bytes()
        crc = data[-2] | (data[-1] << 8)
        self.assertEqual(crc, sum(data[:-2]) & 0xFFFF)

    def test_arm_aux_high(self):
        rc = RCChannels(aux1=2000)
        self.assertGreater(rc.aux1, 1500)

    def test_disarm_aux_low(self):
        rc = RCChannels(aux1=1000)
        self.assertLess(rc.aux1, 1200)


class TestRCInput(unittest.TestCase):
    def setUp(self):
        from fc_simulator import FlightControllerSimulator

        self.fc = FlightControllerSimulator()

    def test_arm_sequence(self):
        self.assertFalse(self.fc.motors_armed)
        rc = RCChannels(aux1=2000)
        self.fc.receive_rc(rc, 0)
        self.fc.arm()
        self.assertTrue(self.fc.motors_armed)

    def test_disarm_sequence(self):
        self.fc.arm()
        self.assertTrue(self.fc.motors_armed)
        self.fc.disarm()
        self.assertFalse(self.fc.motors_armed)

    def test_mode_selection(self):
        rc = RCChannels(aux2=1000)
        self.fc.receive_rc(rc, 0)
        self.assertEqual(self.fc.current_mode, FlightMode.MANUAL)

        rc = RCChannels(aux2=2000)
        self.fc.receive_rc(rc, 0)
        self.assertEqual(self.fc.current_mode, FlightMode.ALTHOLD)


class TestRCEnums(unittest.TestCase):
    def test_flight_mode_values(self):
        self.assertEqual(FlightMode.MANUAL, 0)
        self.assertEqual(FlightMode.ALTHOLD, 1)
        self.assertEqual(FlightMode.POSHOLD, 2)

    def test_fail_safe_state_values(self):
        self.assertEqual(FailSafeState.NONE, 0)
        self.assertEqual(FailSafeState.SIGNAL_LOSS, 1)

    def test_control_source_values(self):
        self.assertEqual(ControlSource.RC_RECEIVER, 0)
        self.assertEqual(ControlSource.COMPANION, 1)


if __name__ == "__main__":
    unittest.main()
