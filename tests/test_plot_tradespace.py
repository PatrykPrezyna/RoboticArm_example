import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulation"))

import plot_tradespace


class ParetoFrontierTests(unittest.TestCase):
    def test_pareto_frontier_identifies_non_dominated_points(self) -> None:
        points = [
            (10.0, 20.0, "A"),
            (12.0, 15.0, "B"),
            (15.0, 10.0, "C"),
            (16.0, 14.0, "D"),
            (20.0, 5.0, "E"),
        ]

        frontier = plot_tradespace.pareto_frontier(points)

        self.assertEqual(frontier, [(10.0, 20.0, "A"), (12.0, 15.0, "B"), (15.0, 10.0, "C"), (20.0, 5.0, "E")])


if __name__ == "__main__":
    unittest.main()
