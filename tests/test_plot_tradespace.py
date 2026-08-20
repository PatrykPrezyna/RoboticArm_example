import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

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


class EarlyTradespaceTests(unittest.TestCase):
    def test_enumerates_component_combinations_and_applies_quantity(self) -> None:
        payload = {
            "components": [
                {
                    "name": "Controller",
                    "options": [
                        {"name": "A", "properties": {"mass": 10, "cost": 2}},
                        {"name": "B", "properties": {"mass": 20, "cost": 3}},
                    ],
                },
                {
                    "name": "Servo",
                    "quantity": 2,
                    "options": [
                        {"name": "S", "properties": {"mass": 5, "cost": 4}},
                    ],
                },
                {"name": "Descriptive only"},
            ]
        }

        designs = plot_tradespace.enumerate_early_designs(payload)

        self.assertEqual(len(designs), 2)
        self.assertEqual(designs[0]["concept_id"], "C01")
        self.assertEqual(designs[1]["concept_id"], "C02")
        self.assertEqual(designs[0]["total_mass_g"], 20.0)
        self.assertEqual(designs[0]["total_cost_usd"], 10.0)
        self.assertEqual(designs[1]["total_mass_g"], 30.0)
        self.assertEqual(designs[1]["total_cost_usd"], 11.0)
        self.assertEqual(
            plot_tradespace.early_decision_names(designs),
            ["Controller", "Servo"],
        )

    def test_project_input_produces_sixteen_designs(self) -> None:
        payload = plot_tradespace.load_json(ROOT / "Simple_tradespace" / "inout.json")

        designs = plot_tradespace.enumerate_early_designs(payload)

        self.assertEqual(len(designs), 16)


if __name__ == "__main__":
    unittest.main()
