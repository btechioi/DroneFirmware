"""Test Telemetry Protocol"""

import unittest
from fc_simulator import Telemetry, FailSafeState, ControlSource


class TestTelemetry(unittest.TestCase):
    def test_default_values(self):
        t = Telemetry()
        self.assertFalse(t.armed)
        self.assertEqual(t.mode, 0)
        self.assertEqual(t.roll, 0.0)

    def test_to_bytes(self):
        t = Telemetry()
        data = t.to_bytes()
        self.assertEqual(data[0], 0x55)
        self.assertEqual(data[1], 0x02)

    def test_armed_flag_in_flags(self):
        t = Telemetry(armed=True)
        data = t.to_bytes()
        flags = data[0]
        self.assertEqual(flags & 1, 1)

    def test_disarmed_flag_in_flags(self):
        t = Telemetry(armed=False)
        data = t.to_bytes()
        flags = data[0]
        self.assertEqual(flags & 1, 0)

    def test_failsafe_encoding(self):
        t = Telemetry(failsafe=FailSafeState.SIGNAL_LOSS)
        data = t.to_bytes()
        flags = data[0]
        self.assertEqual((flags >> 4) & 0x0F, FailSafeState.SIGNAL_LOSS)

    def test_control_source_encoding(self):
        t = Telemetry(control_source=ControlSource.COMPANION)
        data = t.to_bytes()
        self.assertEqual(data[1], ControlSource.COMPANION)

    def test_attitude_encoding(self):
        t = Telemetry(roll=1.57, pitch=-0.78)
        self.assertEqual(int(t.roll * 100), 157)
        self.assertEqual(int(t.pitch * 100), -78)

    def test_altitude_encoding(self):
        t = Telemetry(altitude=10.5)
        self.assertEqual(int(t.altitude * 100), 1050)

    def test_battery_encoding(self):
        t = Telemetry(battery_mv=11800)
        self.assertEqual(t.battery_mv, 11800)


if __name__ == "__main__":
    unittest.main()
