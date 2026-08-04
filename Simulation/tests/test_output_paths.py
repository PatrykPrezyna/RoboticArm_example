import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import plot_tradespace
except ModuleNotFoundError:  # pragma: no cover - environment-dependent
    plot_tradespace = None
import sim_core


class OutputPathTests(unittest.TestCase):
    def test_default_output_path_uses_timestamped_name(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "output"
            path = sim_core.default_output_path(output_dir)

            self.assertEqual(path.parent, output_dir)
            self.assertTrue(path.name.startswith("sim_result_"))
            self.assertTrue(path.name.endswith(".csv"))

    def test_latest_result_csv_prefers_newest_file(self) -> None:
        if plot_tradespace is None:
            self.skipTest("matplotlib is not installed")

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            older = output_dir / "sim_result_20240101_010101.csv"
            older.write_text("old", encoding="utf-8")
            newer = output_dir / "sim_result_20240102_020202.csv"
            newer.write_text("new", encoding="utf-8")

            self.assertEqual(plot_tradespace.latest_result_csv(output_dir), newer)


if __name__ == "__main__":
    unittest.main()
