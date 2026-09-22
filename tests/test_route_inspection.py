import unittest

import numpy as np

from sentinel.route_inspection import assign_inspections, uncertain_route_tasks


class RouteInspectionTests(unittest.TestCase):
    def test_unknown_route_stretches_become_separate_tasks(self):
        observed = np.ones((4, 8), dtype=bool)
        observed[1, 1:3] = False
        observed[1, 5:8] = False
        tasks = uncertain_route_tasks(observed, [(x, 1) for x in range(8)], .5)
        self.assertEqual([len(task.cells) for task in tasks], [2, 3])
        self.assertEqual([task.priority for task in tasks], [1.0, 1.5])

    def test_closest_drone_receives_highest_priority_inspection(self):
        observed = np.ones((5, 10), dtype=bool)
        observed[2, 1:5] = False
        observed[2, 8:10] = False
        tasks = uncertain_route_tasks(observed, [(x, 2) for x in range(10)], 1)
        assignments = assign_inspections(tasks, [(2, 2), (9, 2), (0, 0)], 1)
        self.assertEqual(assignments[0].drone_id, 0)
        self.assertEqual(assignments[1].drone_id, 1)
        self.assertLess(assignments[0].travel_m, 1.2)

    def test_known_route_needs_no_inspection(self):
        observed = np.ones((3, 3), dtype=bool)
        self.assertEqual(uncertain_route_tasks(observed, [(0, 1), (1, 1), (2, 1)], 1), [])


if __name__ == "__main__":
    unittest.main()
