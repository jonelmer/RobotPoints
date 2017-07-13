'''
Contains utilities for calculating the laser's profile and creating a mesh which represents the laser

'''

import trimesh
import trimesh.path
import trimesh.path.entities
import trimesh.points
import trimesh.ray.ray_triangle
import trimesh.visual
import numpy as np
import shapely
import shapely.geometry
import shapely.affinity
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from tqdm import tqdm, trange


class LaserException(Exception):
    pass


class PathFinderException(Exception):
    pass


class Laser(object):

    def __init__(self, width, height, focus_distance):
        self.width = width
        self.height = height
        self.focus_distance = focus_distance

    def get_mesh(self, target, pre=True, post=True):
        image = self.get_image_vertices(target)
        focus = self.get_focus(target)

        if pre:
            mesh_vertices = [focus]
            mesh_vertices += image
            mesh_faces = [(0, 2, 1), (0, 3, 2), (0, 4, 3), (0, 1, 4), (1, 2, 3), (1, 3, 4)]

        if pre and post:
            for i in range(1, len(mesh_vertices)):
                mesh_vertices[i] = 2 * mesh_vertices[i] - focus

        if post and not pre:
            mesh_vertices = []
            mesh_vertices += image
            mesh_vertices += [2 * vertex - focus for vertex in image]

            mesh_faces = [(0, 1, 4), (5, 4, 1),
                          (1, 2, 5), (6, 5, 2),
                          (2, 3, 6), (7, 6, 3),
                          (3, 0, 7), (4, 7, 0),
                          (0, 2, 1), (0, 3, 2),
                          (4, 5, 6), (4, 6, 7)]

        return trimesh.Trimesh(vertices=mesh_vertices, faces=mesh_faces)

    def get_nominal_3D(self, target):
        return trimesh.path.Path3D([trimesh.path.entities.Line((0, 1))],
                                   [target['point'], self.get_focus(target)])

    def get_focus(self, target):
        return target['point'] + target['normal']*self.focus_distance

    def get_image_vertices(self, target):
        # Only consider normals which are in the x-y plane
        if not target['normal'][2] == 0.:
            raise LaserException("Expected the z-normal to be zero, but got {}".format(target['normal'][2]))

        # Rotate about the z axis to get the rightwards vector
        right = np.dot([
            [0., -1., 0.],
            [1.,  0., 0.],
            [0.,  0., 1.]],
            target['normal'])
        right *= self.width/2

        # Up must be +Z
        up = np.array([0., 0., 1.]) * self.height/2

        vertices = [None] * 4

        vertices[0] = target['point'] - right - up
        vertices[1] = target['point'] - right + up
        vertices[2] = target['point'] + right + up
        vertices[3] = target['point'] + right - up

        return vertices


class PathFinder(object):
    def __init__(self, laser, mesh, process=True):
        self.laser = laser
        self.mesh = mesh

        self.layers = None
        self.sections = None
        self.section_points = None

        if process:
            self.process()

    def process(self):
        self._layers()
        self._sections()
        self._points()

    def _layers(self):
        self.layers = np.arange(*self.mesh.bounds[:, 2], step=self.laser.height)

    def _sections(self):
        sections = [None] * len(self.layers)
        for i, z in tqdm(enumerate(self.layers), desc="Sectioning mesh..."):
            sections[i] = self.mesh.section(plane_origin=[0., 0., z], plane_normal=[0., 0., 1.])
        self.sections = sections

    def _points(self):
        section_points = [None] * len(self.layers)

        def next_point(path, index, last_point, distance):
            last = shapely.geometry.Point(last_point)

            search = shapely.geometry.LineString([last_point] + path.coords[index:])

            for i, p in enumerate(search.coords):
                d = shapely.geometry.Point(p).distance(last)
                if d > distance:
                    d_s = last.distance(shapely.geometry.Point(search.coords[i - 1]))
                    print('Distance to previous path point is ', d_s)
                    print('Distance to next path point is ', d)
                    line = shapely.geometry.LineString([search.coords[i - 1], search.coords[i]])

                    intersection = shapely.geometry.\
                        LinearRing(last.buffer(self.laser.width).exterior.coords).intersection(line)

                    print('Distance to intersection is ', last.distance(intersection))

                    if type(intersection) is shapely.geometry.Point:
                        return intersection, i + index
                    else:
                        if len(intersection) is 0:
                            return shapely.geometry.Point(search.coords[i]), i + index
                        raise PathFinderException('Intersection is not a point! {}'.format(intersection))

            raise PathFinderException('No point is at specified distance from start')

        with tqdm(total=len(self.sections), desc="Calculating points...") as pbar:

            for i, [section, z] in enumerate(zip(self.sections, self.layers)):
                flat_section, transform = section.to_planar()

                try:
                    poly = flat_section.polygons_full[0]
                except IndexError:
                    raise PathFinderException("No flat section was found for layer at z={}".format(z))

                ring = shapely.geometry.LinearRing(poly.exterior)
                ring = shapely.affinity.affine_transform(ring, [1, 0, 0, 1, transform[0][-1], transform[1][-1]])

                if not ring.is_ccw:
                    ring.coords = list(ring.coords)[::-1]

                points = []
                points.append(tuple(ring.coords[0]))

                start_point = shapely.geometry.Point(points[0])

                point_index = 0

                while True:
                    if point_index >= len(ring.coords):
                        break

                    try:
                        intersection, point_index = next_point(ring, point_index, points[-1], self.laser.width)
                    except PathFinderException:
                        break

                    intersection = list(intersection.coords)[0]

                    print('Next intersection at', intersection)

                    points.append(intersection)

                    if start_point.distance(shapely.geometry.Point(intersection)) < self.laser.width:
                        print('Close to start point')
                        if len(points) > 2:
                            print('That\'s a wrap!')
                            break

                section_points[i] = [list(p) + [z] for p in points]
                pbar.update()

        self.section_points = section_points

    def get_pointcloud(self):
        if self.section_points is not None:
            return trimesh.points.PointCloud(np.sum(self.section_points))

    def get_squares(self):
        squares = [[None]] * len(self.section_points)

        for i, section_points in enumerate(self.section_points):

            # Clear the current section's list of squares
            squares[i] = []

            for a, b in zip(section_points, section_points[1:]):
                square = []
                square.append(a)
                square.append(a[:2]+[a[2]+self.laser.height])
                square.append(b[:2]+[b[2]+self.laser.height])
                square.append(b)

                squares[i].append(square)

        return squares


class PathAssessor(object):
    def __init__(self, laser, mesh):
        self.laser = laser
        self.mesh = mesh
        self.intersector = trimesh.ray.ray_triangle.RayMeshIntersector(self.mesh)

    def process_layers(self, layers, steps):
        print "Assessing {} layers".format(len(squares))

        pbar = tqdm(total=sum([len(l) for l in layers]), desc="Assessing... ")

        layer_distances = [self.process_squares(layer, steps, pbar=pbar) for layer in layers]

        pbar.close()

        data = layer_distances[0]
        for layer in layer_distances[1:]:
            data += layer

        return data#layer_distances#zip(*layer_distances)


    def process_squares(self, squares, steps, pbar=None):
        distances = [None] * len(squares)

        for i, square in enumerate(squares):
            distances[i] = self.process_square(square, steps, pbar)

        return stitch_arrays(distances)


    def process_square(self, square, steps, pbar):
        target = vertices_to_target(square)
        focus = self.laser.get_focus(target)
        grid = self.interpolate_grid(target, steps)
        distances = [self.distance(point, focus) for point in grid]

        #data = [g + [d] for g, d in zip(grid, distances)]

        if pbar:
            pbar.update()

        return np.reshape(distances, (steps, steps))

    def interpolate_grid(self, target, steps):
        image = self.laser.get_image_vertices(target)
        x = np.linspace(image[2][0], image[0][0], steps+1)
        x += (x[1] - x[0])/2
        x = x[:-1]

        y = np.linspace(image[2][1], image[0][1], steps+1)
        y += (y[1] - y[0]) / 2
        y = y[:-1]

        z = np.linspace(image[0][2], image[2][2], steps+1)
        z += (z[1] - z[0]) / 2
        z = z[:-1]

        points = [[i, j, k] for k in z for i, j in zip(x, y)]

        return points

    def distance(self, point, focus):
        locations = self.intersector.intersects_location([focus], [point - focus])

        dist = 0

        for loc in locations[0]:
            delta = loc - focus
            dist_candidate = np.sum([d**2 for d in delta])**0.5
            if dist_candidate > dist:
                dist = dist_candidate

        return dist - self.laser.focus_distance


def vertices_to_target(vertices):
    # Check that exactly 4 vertices were supplied
    if len(vertices) is not 4:
        raise LaserException("points_to_target requires 4 vertices, but {} were given".format(len(vertices)))

    # Numpyify the points
    vertices = np.array(vertices)

    # Calculate position deltas
    a = vertices[1] - vertices[0]
    b = vertices[2] - vertices[0]

    # The target point is the mean of all vertices
    target_point = sum(vertices) / 4

    # Find the target normal
    target_normal = np.cross(a, b)
    target_normal = target_normal / np.linalg.norm(target_normal)

    return {'point': target_point, 'normal': target_normal}


def vertices_to_3D(vertices, closed=True):
    entities = [trimesh.path.entities.Line((x, x+1)) for x in range(len(vertices) - 1)]

    if closed:
        entities.append(trimesh.path.entities.Line((len(vertices)-1, 0)))

    return trimesh.path.Path3D(entities, vertices)


def squares_to_3D(squares):
    s = [item for sublist in squares for item in sublist]
    return np.sum([vertices_to_3D(item) for sublist in squares for item in sublist])


def set_mesh_color(mesh, color=(0.5, 0.5, 0.5, 0.5)):
    if type(color) is str:
        if color is "red": color = (1, 0, 0)
        if color is "green": color = (0, 1, 0)
        if color is "blue": color = (0, 0, 1)
    mesh.visual = trimesh.visual.create_visual(face_colors=[color] * len(mesh.faces))

def stitch_arrays(arrays):
    # Take a l x m x n array, and smoosh it together, patchwork style

    # Initialise with the first 2D array
    data = [list(a) for a in arrays[0]]

    for array in arrays[1:]:
        # Each array is 2D
        for i, row in enumerate(array):
            data[i].extend(row)

    return data


def equalize_and_mask(arrays):
    if not all([len(arrays) == len(arrays[0]) for array in arrays]):
        # The arrays are not of equal size
        longest = np.max([len(array) for array in arrays])

        for array in arrays:
            while len(array) < longest:
                array.append(np.nan)

    return np.ma.masked_where(np.isnan(arrays), arrays)


def plot_colour_map(data, ticks=1, blanks=True):
    maxima = np.max(data)
    minima = np.min(data)

    x = np.arange(float(len(data[0]) + 1))
    y = np.arange(float(len(data) + 1))
    x /= ticks
    y /= ticks

    extreme = max([maxima, -minima])
    norm = colors.Normalize(vmin=-extreme, vmax=extreme)

    ax = plt.gca()
    plt.plot()

    if blanks:
        ax.set_facecolor((0.8, 0.8, 0.8))

    plt.pcolormesh(x, y, data, cmap='seismic', norm=norm)
    plt.axis('equal')
    plt.colorbar()

    plt.grid(True, which='major', axis='both', linestyle='-', color='k')

    plt.xticks(range(int(max(x)) + 1))
    plt.yticks(range(int(max(y)) + 1))

    plt.title("Deviation from image plane (mm)\n")


def cylinder():
    mesh = trimesh.primitives.Cylinder().smoothed()
    mesh.apply_scale(5)
    laser = Laser(5., 5., 50.)
    path = PathFinder(laser, mesh)

    squares = path.get_squares()

    res = 10
    ass = PathAssessor(laser, mesh)

    scene = mesh.scene()
    scene.add_geometry(squares_to_3D(squares))

    data = equalize_and_mask(ass.process_layers(squares, res))
    plot_colour_map(data, res)

    set_mesh_color(mesh)

    return scene


def cube():
    mesh = trimesh.primitives.Box().smoothed()
    mesh.apply_scale(22)
    laser = Laser(5., 5., 50.)
    path = PathFinder(laser, mesh)

    squares = path.get_squares()

    res = 10
    ass = PathAssessor(laser, mesh)

    scene = mesh.scene()
    scene.add_geometry(squares_to_3D(squares))

    data = equalize_and_mask(ass.process_layers(squares, res))
    plot_colour_map(data, res)

    set_mesh_color(mesh)

    return scene


def shuttle():
    mesh = trimesh.load('Buran.stl')
    mesh.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [0, 1, 0]))
    mesh.apply_scale(0.1)

    laser = Laser(5., 5., 50.)

    path = PathFinder(laser, mesh, False)
    path._layers()
    path.layers = path.layers[1:20]
    path._sections()
    path._points()

    squares = path.get_squares()

    scene = mesh.scene()
    scene.add_geometry(path.get_pointcloud())

    scene.add_geometry(squares_to_3D(squares))

    res = 10
    ass = PathAssessor(laser, mesh)

    data = equalize_and_mask(ass.process_layers(squares, res))
    plot_colour_map(data, res)

    set_mesh_color(mesh)

    return scene


if __name__ == "__main__" or __name__ == "__builtin__":
    mesh = trimesh.load('Part1.stl')
    laser = Laser(5., 5., 50.)
    path = PathFinder(laser, mesh)

    squares = path.get_squares()

    res = 10
    ass = PathAssessor(laser, mesh)

    scene = mesh.scene()
    scene.add_geometry(squares_to_3D(squares))

    data = equalize_and_mask(ass.process_layers(squares, res))
    plot_colour_map(data, res)

    set_mesh_color(mesh)

    #scene.show()

    #n, b, p = plt.hist(data.compressed(), 50)

