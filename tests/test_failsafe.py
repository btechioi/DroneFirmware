"""Test Failsafe System"""

import unittest
import time
from fc_simulator import FailsafeManager, FailSafeState, RCChannels


class TestFailsafeManager(unittest.TestCase):
    def test_initialization(self):
        fs = FailsafeManager()
        self.assertEqual(fs.state, FailSafeState.NONE)
        self.assertFalse(fs.rc_lost)

    def test_rc_timeout_triggers_failsafe(self):
        fs = FailsafeManager()
        now_ms = 0
        rc = RCChannels()

        fs.signal_rc_received(now_ms)

        time.sleep(0.6)
        now_ms = int(time.time() * 1000)

        fs.update(now_ms, rc)

        self.assertEqual(fs.state, FailSafeState.SIGNAL_LOSS)

    def test_rc_reception_clears_failsafe(self):
        fs = FailsafeManager()
        rc = RCChannels()

        fs.signal_rc_received(0)
        fs.state = FailSafeState.SIGNAL_LOSS

        now_ms = int(time.time() * 1000)
        fs.signal_rc_received(now_ms)

        self.assertEqual(fs.state, FailSafeState.SIGNAL_LOSS)

    def test_control_change_resets_stuck_timer(self):
        fs = FailsafeManager()
        rc = RCChannels(roll=1500)

        fs.signal_rc_received(0)

        rc.roll = 1600
        now_ms = 0
        fs.update(now_ms, rc)

        self.assertEqual(fs._prev_roll, 1600)

    def test_failsafe_reset(self):
        fs = FailsafeManager()
        fs.state = FailSafeState.SIGNAL_LOSS
        fs.rc_lost = True

        fs.reset()

        self.assertEqual(fs.state, FailSafeState.NONE)
        self.assertFalse(fs.rc_lost)

    def test_critical_sensor_not_resettable(self):
        fs = FailsafeManager()
        fs.state = FailSafeState.CRITICAL_SENSOR

        fs.reset()

        self.assertEqual(fs.state, FailSafeState.CRITICAL_SENSOR)

    def test_multiple_rc_updates(self):
        fs = FailsafeManager()
        rc = RCChannels()

        for i in range(100):
            now_ms = i * 100
            fs.signal_rc_received(now_ms)
            fs.update(now_ms, rc)

        self.assertEqual(fs.state, FailSafeState.NONE)

    def test_rc_timeout_threshold(self):
        fs = FailsafeManager()
        fs.rc_timeout_ms = 100

        fs.signal_rc_received(0)
        fs.update(200, RCChannels())

        self.assertEqual(fs.state, FailSafeState.SIGNAL_LOSS)


if __name__ == "__main__":
    unittest.main()
