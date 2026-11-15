import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import revisit_count as rc  # noqa: E402

FWD = np.array([0.0, 0.0, 1.0])


class RevisitTest(unittest.TestCase):
    def test_known_two_revisits_sparse(self):
        # A,B,C visited, then A and B again 18/24 s later with the same view: exactly 2 pairs.
        t = np.array([0.0, 6.0, 12.0, 18.0, 24.0])
        p = np.array([[0, 0, 0], [5, 0, 0], [10, 0, 0], [0.1, 0, 0], [5.1, 0, 0]], float)
        view = np.tile(FWD, (5, 1))
        c = rc.count(t, p, view)
        self.assertEqual(c["revisit_pairs"], 2)
        self.assertEqual(c["revisit_events"], 2)

    def test_view_angle_blocks_revisit(self):
        t = np.array([0.0, 10.0])
        p = np.zeros((2, 3))
        v = np.array([FWD, [np.sin(np.radians(40)), 0, np.cos(np.radians(40))]])
        self.assertEqual(rc.count(t, p, v)["revisit_pairs"], 0)

    def test_dense_two_crossings_give_two_events(self):
        # dense 10 Hz path (2 m/s) that crosses its own earlier leg at two different places -> 2 events,
        # although many raw sample pairs.
        wp_t = [0, 15, 20, 25, 35, 40, 47.5]
        wp_x = [0, 30, 30, 20, 20, 10, 10]
        wp_y = [0, 0, 10, 10, -10, -10, 5]
        t = np.arange(0, 47.5, 0.1)
        p = np.stack([np.interp(t, wp_t, wp_x), np.interp(t, wp_t, wp_y), 0 * t], 1)
        view = np.tile(FWD, (len(t), 1))
        c = rc.count(t, p, view)
        self.assertGreater(c["revisit_pairs"], 2)
        self.assertEqual(c["revisit_events"], 2)

    def test_time_gap_must_exceed_T(self):
        t = np.array([0.0, 4.9])
        self.assertEqual(rc.count(t, np.zeros((2, 3)), np.tile(FWD, (2, 1)))["revisit_pairs"], 0)

    def test_approximation_stated(self):
        c = rc.count(np.array([0.0, 10.0]), np.zeros((2, 3)), np.tile(FWD, (2, 1)))
        self.assertIn("APPROXIMATION", c["kf64_basis"])


if __name__ == "__main__":
    unittest.main()
