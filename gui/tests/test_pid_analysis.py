import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from state import PIDState
from widgets.plots import compute_pid_contributions, compute_pid_terms


class PidAnalysisTests(unittest.TestCase):
    def test_pid_terms_are_computed_for_each_axis(self):
        samples = [
            {"time": 0.0, "error_x": 10.0, "error_y": 2.0},
            {"time": 0.1, "error_x": 7.0, "error_y": 1.0},
            {"time": 0.2, "error_x": 5.0, "error_y": 0.5},
        ]
        pid = PIDState(kp_x=1.0, ki_x=0.5, kd_x=2.0, kp_y=0.8, ki_y=0.2, kd_y=1.0)

        enriched = compute_pid_terms(samples, pid)

        self.assertAlmostEqual(enriched[0]["p_x"], 10.0)
        self.assertIn("i_x", enriched[0])
        self.assertIn("d_x", enriched[0])
        self.assertIn("p_y", enriched[0])
        self.assertAlmostEqual(enriched[0]["p_y"], 1.6)

    def test_steady_state_error_uses_the_recent_tail(self):
        samples = [
            {"time": 0.0, "error_x": 8.0},
            {"time": 0.1, "error_x": 7.0},
            {"time": 0.2, "error_x": 6.0},
            {"time": 0.3, "error_x": 4.0},
            {"time": 0.4, "error_x": 2.0},
            {"time": 0.5, "error_x": 1.0},
            {"time": 0.6, "error_x": 1.0},
            {"time": 0.7, "error_x": 1.0},
        ]

        summary = compute_pid_contributions(samples, PIDState())

        self.assertAlmostEqual(summary["x"]["steady_state_error"], 1.0)
        self.assertGreaterEqual(summary["x"]["p"], 0.0)
        self.assertGreaterEqual(summary["x"]["i"], 0.0)
        self.assertGreaterEqual(summary["x"]["d"], 0.0)


if __name__ == "__main__":
    unittest.main()
