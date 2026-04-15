"""Test Motor Mixer"""

import unittest
from fc_simulator import MotorMixer


class TestMotorMixer(unittest.TestCase):
    def test_mixer_initialization(self):
        mixer = MotorMixer()
        self.assertEqual(mixer.motor_count, 4)
        self.assertEqual(len(mixer.motor_outputs), 4)

    def test_neutral_throttle(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1000, roll=0, pitch=0, yaw=0)
        self.assertEqual(outputs, [1000, 1000, 1000, 1000])

    def test_roll_right(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=100, pitch=0, yaw=0)
        self.assertEqual(outputs[0], 1600)
        self.assertEqual(outputs[1], 1400)
        self.assertEqual(outputs[2], 1400)
        self.assertEqual(outputs[3], 1600)

    def test_roll_left(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=-100, pitch=0, yaw=0)
        self.assertEqual(outputs[0], 1400)
        self.assertEqual(outputs[1], 1600)
        self.assertEqual(outputs[2], 1600)
        self.assertEqual(outputs[3], 1400)

    def test_pitch_forward(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=0, pitch=100, yaw=0)
        self.assertEqual(outputs[0], 1600)
        self.assertEqual(outputs[1], 1600)
        self.assertEqual(outputs[2], 1400)
        self.assertEqual(outputs[3], 1400)

    def test_yaw_right(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=0, pitch=0, yaw=100)
        self.assertEqual(outputs[0], 1600)
        self.assertEqual(outputs[1], 1400)
        self.assertEqual(outputs[2], 1600)
        self.assertEqual(outputs[3], 1400)

    def test_full_control(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=50, pitch=50, yaw=50)
        self.assertEqual(len(outputs), 4)
        self.assertTrue(all(1000 <= o <= 2000 for o in outputs))

    def test_output_limits(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=2000, roll=500, pitch=500, yaw=500)
        self.assertTrue(all(1000 <= o <= 2000 for o in outputs))

    def test_zero_throttle(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=0, roll=100, pitch=100, yaw=100)
        self.assertTrue(all(1000 <= o <= 2000 for o in outputs))

    def test_negative_values(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1000, roll=-50, pitch=-50, yaw=-50)
        self.assertTrue(all(1000 <= o <= 2000 for o in outputs))

    def test_motor_outputs_clamped(self):
        mixer = MotorMixer()
        outputs = mixer.compute(throttle=1500, roll=1000, pitch=1000, yaw=1000)
        self.assertTrue(all(1000 <= o <= 2000 for o in outputs))


if __name__ == "__main__":
    unittest.main()
