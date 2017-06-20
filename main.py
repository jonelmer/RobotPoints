import trimesh
import trimesh.path as path
from shapely.geometry import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D, art3d

mesh_file = 'Part1.stl'
laser_height = 10
laser_width = 10

class PathGeneratorException(Exception):
    pass


def next_intersection(path, search):
    #path.coords = path.coords[5:] + path.coords[:5]
    #path.coords = path.coords[::-1]

    for i in range(len(search.coords)-1):
        line = LineString([search.coords[i+1], search.coords[i]])

        if line.intersects(path):
            intersection = line.intersection(path)
            return list(intersection.coords)[0]


def next_point(path, index, last_point, distance):
    last = Point(last_point)

    search = LineString([last_point] + path.coords[index:])

    for i, p in enumerate(search.coords):
        d = Point(p).distance(last)
        if d > distance:
            d_s = last.distance(Point(search.coords[i - 1]))
            print('Distance to previous path point is ', d_s)
            print('Distance to next path point is ', d)
            line = LineString([search.coords[i - 1], search.coords[i]])

            intersection = LinearRing(last.buffer(laser_width).exterior.coords).intersection(line)
            print('Distance to intersection is ', last.distance(intersection))

            if type(intersection) is Point:
                return intersection, i+index
            else:
                if len(intersection) is 0:
                    return Point(search.coords[i]), i+index
                raise PathGeneratorException('Intersection is not a point! %s' % (intersection))

    raise PathGeneratorException('No point is at specified distance from start')


if __name__ == '__main__' or __name__ == '__builtin__':
    mesh = trimesh.load(mesh_file)
    mesh.apply_transform(trimesh.transformations.scale_matrix(2))
    #mesh.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2, [0, 1, 0]))

    print('Object has extents ', mesh.bounds)

    z_extents = mesh.bounds[:, 2]
    z_extents[0] = z_extents[0] + laser_height/2
    z_levels = np.arange(*z_extents, step=laser_height)

    sections = [None] * len(z_levels)

    for i, z in enumerate(z_levels):
        sections[i] = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])

    combined = np.sum(sections)

    mesh.show()
    combined.show()

    fig = plt.figure()
    ax = Axes3D(fig)

    section_points = [None] * len(z_levels)

    for i, [section, z] in enumerate(zip(sections, z_levels)):
        flat_section, transform = section.to_planar()

        try:
            poly = flat_section.polygons_full[0]
        except IndexError:
            break

        print poly.exterior.length

        ring = LinearRing(poly.exterior)

        #ring = LinearRing(poly.convex_hull.exterior)
        if not ring.is_ccw:
            ring.coords = list(ring.coords)[::-1]

        coords = list(ring.coords)
        x, y = zip(*coords)

        points = []
        points.append(tuple(ring.coords[0]))

        start_point = Point(points[0])
        start_zone = LinearRing(start_point.buffer(laser_width).exterior.coords)
        sx, sy = zip(*list(start_zone.coords))

        index = 0

        while True:
            if index >= len(ring.coords):
                break

            try:
                intersection, index = next_point(ring, index, points[-1], laser_width)
            except None: #PathGeneratorException:
                break

            intersection = list(intersection.coords)[0]

            print('Next intersection at', intersection)

            points.append(intersection)

            if start_point.distance(Point(intersection)) < laser_width:
                print('Close to start point')
                if len(points) > 2:
                    print('That\'s a wrap!')
                    break

        section_points[i] = points

        ax.plot(x, y, z, 'r-', linewidth=1, alpha=0.5)
        ax.plot(sx, sy, z, 'b-', linewidth=1, alpha=0.5)
        px, py = zip(*points)
        ax.plot(px, py, z, 'bx')
        plt.axis('equal')
        #plt.show()

        if __name__ == '__builtin__':
            raw_input()

    print section_points

    for i in range(len(z_levels)-1):
        if section_points[i] is None:
            continue

        for j in range(len(section_points[i])-1):
            try:
                a = section_points[i][j] + (z_levels[i],)
                b = section_points[i+1][j] + (z_levels[i+1],)
                c = section_points[i+1][j+1] + (z_levels[i+1],)
                d = section_points[i][j+1] + (z_levels[i],)
            except (IndexError, TypeError):
                break

            x, y, z = zip(a, b, c, d, a)

            ax.plot(x, y, z, 'k-')

            sq = art3d.Poly3DCollection([zip(x, y, z)])
            sq.set_color('k')
            sq.set_edgecolor('w')
            sq.set_alpha(0.1)
            ax.add_collection3d(sq)

    plt.show()
    raw_input()



