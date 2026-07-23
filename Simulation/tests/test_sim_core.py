import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sim_core


class ResultRowTests(unittest.TestCase):
    def test_build_result_row_orders_success_and_move_time_first(self) -> None:
        row = sim_core.build_result_row(
            {"shoulder": {"position_sequence": [1, 2]}},
            10.0,
            1.2345,
            extra={"shoulder_servo_name": "SG90"},
        )

        self.assertEqual(list(row.keys())[:2], ["success", "move_time_s"])
        self.assertTrue(row["success"])
        self.assertEqual(row["move_time_s"], 1.2345)


if __name__ == "__main__":
    unittest.main()
