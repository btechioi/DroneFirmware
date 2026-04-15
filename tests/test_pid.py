"""Test PID Controller"""

import unittest
from fc_simulator import PID


class TestPID(unittest.TestCase):
    def test_pid_initialization(self):
        pid = PID(kp=1.0, ki=0.5, kd=0.2)
        self.assertEqual(pid.kp, 1.0)
        self.assertEqual(pid.ki, 0.5)
        self.assertEqual(pid.kd, 0.2)
        self.assertEqual(pid.output_limit, 500.0)

    def test_pid_proportional(self):
        pid = PID(kp=2.0, ki=0.0, kd=0.0, output_limit=100)
        output = pid.compute(error=10.0, dt=0.01)
        self.assertAlmostEqual(output, 20.0, places=1)

    def test_pid_integral_windup(self):
        pid = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=50)
        for _ in range(100):
            pid.compute(error=1.0, dt=0.01)
        self.assertGreater(pid._integral, 40)

    def test_pid_derivative(self):
        pid = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=100)
        pid.compute(error=0.0, dt=0.01)
        output = pid.compute(error=10.0, dt=0.01)
        self.assertAlmostEqual(output, 1000.0, places=0)

    def test_pid_output_limits(self):
        pid = PID(kp=100.0, ki=0.0, kd=0.0, output_limit=50)
        output = pid.compute(error=10.0, dt=0.01)
        self.assertLessEqual(abs(output), 50)

    def test_pid_reset(self):
        pid = PID(kp=1.0, ki=1.0, kd=1.0)
        pid.compute(error=10.0, dt=0.01)
        pid.reset()
        self.assertEqual(pid._integral, 0.0)
        self.assertEqual(pid._prev_error, 0.0)

    def test_pid_full_response(self):
        pid = PID(kp=1.0, ki=0.5, kd=0.2, output_limit=100)
        pid.reset()

        outputs = []
        for i in range(100):
            error = 10.0 - i * 0.1
            output = pid.compute(error=error, dt=0.01)
            outputs.append(output)

        self.assertTrue(all(abs(o) <= 100 for o in outputs))

    def test_pid_zero_error(self):
        pid = PID(kp=1.0, ki=1.0, kd=1.0)
        pid.compute(error=5.0, dt=0.01)
        output = pid.compute(error=0.0, dt=0.01)
        self.assertLess(abs(output), 1.0)

    def test_pid_negative_error(self):
        pid = PID(kp=2.0, ki=0.0, kd=0.0, output_limit=100)
        output = pid.compute(error=-5.0, dt=0.01)
        self.assertAlmostEqual(output, -10.0, places=1)
