import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simulate_all


class SimulateAllTests(unittest.TestCase):
    def test_servo_options_exist_in_catalog(self) -> None:
        sweep_input = {
            "shoulder": {"servo_options": ["SG90", "AD002"]},
            "elbow": {"servo_options": ["MG996R"]},
        }

        candidates = [
            simulate_all.servo_candidates(sweep_input, joint)
            for joint in ("shoulder", "elbow")
        ]

        self.assertEqual([item[0]["name"] for item in candidates], ["SG90", "MG996R"])
        self.assertEqual(candidates[0][0]["tau_stall"], 0.118)
        self.assertEqual(candidates[1][0]["tau_stall"], 1.324)


if __name__ == "__main__":
    unittest.main()
