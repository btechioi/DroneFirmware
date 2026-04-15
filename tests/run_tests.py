"""Test Runner for Drone Flight Controller Tests"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))


def run_tests():
    loader = unittest.TestLoader()
    suite = loader.discover(".", pattern="test_*.py")

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")

    if result.wasSuccessful():
        print("\nALL TESTS PASSED")
        return 0
    else:
        print("\nSOME TESTS FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(run_tests())
