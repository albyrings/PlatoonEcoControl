import unittest
from main import compute_dijkstra

class TestComputeDijkstra(unittest.TestCase):
    def setUp(self):
        self.t_min = [0, 20, 40, 60, 80, 100, 200]
        self.t_max = [0, 30, 50, 70, 90, 110, 200]
        self.distances = [0, 300, 600, 900, 1200, 1550, 1800]

    def test_optimal_times(self):
        optimal_times = compute_dijkstra(self.t_min, self.t_max, self.distances)
        expected_times = [0, 20, 40, 60, 80, 100, 200]  # Example expected times
        self.assertEqual(optimal_times, expected_times)

    def test_no_feasible_path(self):
        t_min = [0, 20, 40, 60, 80, 100, 200]
        t_max = [0, 10, 20, 30, 40, 50, 60]  # No feasible path
        distances = [0, 300, 600, 900, 1200, 1550, 1800]
        optimal_times = compute_dijkstra(t_min, t_max, distances)
        self.assertEqual(optimal_times, [])  # Expecting no feasible path

    def test_single_intersection(self):
        t_min = [0, 20, 200]
        t_max = [0, 30, 200]
        distances = [0, 300, 1800]
        optimal_times = compute_dijkstra(t_min, t_max, distances)
        expected_times = [0, 20, 200]  # Example expected times
        self.assertEqual(optimal_times, expected_times)

if __name__ == '__main__':
    unittest.main()