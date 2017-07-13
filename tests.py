import unittest
from laser import *


def check_vectors(a, b):
    return all([x == y for x, y in zip(a, b)])


def check_vector_list(a, b):
    return all(check_vectors(c, d) for c, d in zip(a, b))


class LaserTestCase(unittest.TestCase):
    """Tests for laser module"""

    def test_vertices_to_target_point(self):
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 0., 1.], [1., 0., 0.]])
        self.assertTrue(check_vectors(target['point'], [0.5, 0., 0.5]))

    def test_vertices_to_target_normal(self):
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 0., 1.], [1., 0., 0.]])
        self.assertTrue(check_vectors(target['normal'], [0., 1., 0.]), msg="Normal is {}".format(target['normal']))

    def test_get_focus(self):
        l = Laser(1., 1., 5.)
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 0., 1.], [1., 0., 0.]])
        self.assertTrue(check_vectors(l.get_focus(target), [0.5, 5., 0.5]))

    def test_vertices_to_target_point_2(self):
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 1., 1.], [1., 1., 0.]])
        self.assertTrue(check_vectors(target['point'], [0.5, 0.5, 0.5]))

    def test_vertices_to_target_normal_2(self):
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 1., 1.], [1., 1., 0.]])
        normal = np.array([-1., 1., 0.])
        normal = normal / np.linalg.norm(normal)

        self.assertTrue(check_vectors(target['normal'], normal),
                        msg="Normal is {}".format(target['normal']))

    def test_get_focus_2(self):
        l = Laser(1., 1., 5.)
        target = vertices_to_target([[0., 0., 0.], [0., 0., 1.], [1., 1., 1.], [1., 1., 0.]])
        focus = np.array([-1., 1., 0.])
        focus = focus / np.linalg.norm(focus)
        focus *= 5.
        focus += [0.5, 0.5, 0.5]

        self.assertTrue(check_vectors(l.get_focus(target), focus),
                        msg="Focus is {}, expected {}".format(l.get_focus(target), focus))

    def test_interpolate_grid(self):
        laser = Laser(3., 3., 5.)
        ass = PathAssessor(laser, trimesh.Trimesh())
        target = {'point': (1.5, 0., 1.5), 'normal': (0., 1., 0.)}
        grid = ass.interpolate_grid(target, 3)
        self.assertTrue(check_vector_list(sorted(grid), sorted([
            [0.5, 0., 0.5], [1.5, 0., 0.5], [2.5, 0., 0.5],
            [0.5, 0., 1.5], [1.5, 0., 1.5], [2.5, 0., 1.5],
            [0.5, 0., 2.5], [1.5, 0., 2.5], [2.5, 0., 2.5],
        ])), msg="len = {}\ngrid = {}".format(len(grid), grid))

    def test_stitch_arrays(self):
        arrays = [[[0, 1], [2, 3]], [[4, 5], [6, 7]], [[8, 9], [10, 11]]]
        self.assertTrue(check_vector_list(stitch_arrays(arrays), [[0, 1, 4, 5, 8, 9], [2, 3, 6, 7, 10, 11]]))




if __name__ == '__main__':
    unittest.main()
