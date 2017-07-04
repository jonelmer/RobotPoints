import trimesh
import trimesh.path as path
import trimesh.path.entities as entities
import trimesh.visual as visual
import trimesh.scene.scene as scene
import trimesh.interfaces.scad as scad
from shapely.geometry import *
from shapely.affinity import affine_transform
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D, art3d

scad._scad_executable = 'C:\\Program Files\\OpenSCAD\\openscad.exe'
scad.exists = True

mesh_file = 'Part1.stl'
laser_height = 10
laser_width = 10
laser_focus = 40

class PathGeneratorException(Exception):
    pass


def laser_profile(vertices):
    profile = []
    mesh_vertices = []
    mesh_faces = []
    vertices = np.array(vertices)

    a = vertices[3] - vertices[0]
    b = vertices[2] - vertices[0]
    c = sum(vertices) / len(vertices)

    normal = np.cross(a, b)
    focus = (normal / np.linalg.norm(normal) * laser_focus) + c

    mesh_vertices.append(focus)

    for p, q in zip(vertices, vertices[1:]):
        profile.append(trimesh.path.Path3D([entities.Line((0, 1)),
                                            entities.Line((1, 2)),
                                            entities.Line((2, 0))],
                                           [2 * p - focus, 2 * q - focus, focus]))
        mesh_vertices.append(2 * p - focus)

    l = len(mesh_vertices)
    for p, q in zip(range(1, l), range(2, l)):
        mesh_faces.append((0, q, p))

    mesh_faces = [(0, 2, 1), (0, 3, 2), (0, 4, 3), (0, 1, 4), (1, 2, 3), (1, 3, 4)]
    #mesh_faces = [mesh_faces[0], mesh_faces[3]]

    return np.sum(profile), trimesh.Trimesh(vertices=mesh_vertices, faces=mesh_faces)




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
    #mesh.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2, [0, 1, 0])

    print('Object has extents ', mesh.bounds)

    z_extents = mesh.bounds[:, 2]
    #z_extents[0] = z_extents[0] + laser_height/2
    z_levels = np.arange(*z_extents, step=laser_height)

    sections = [None] * len(z_levels)

    for i, z in enumerate(z_levels):
        sections[i] = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])

    mesh.visual = visual.create_visual(face_colors=[(0.2, 0.2, 1, 0.5)]*len(mesh.faces))

    sc = scene.Scene()
    #sc.add_geometry(sections[-1])
    sc.add_geometry(np.sum(sections))
    sc.add_geometry(mesh)

    sc.show()

    fig = plt.figure()
    ax = Axes3D(fig)

    section_points = [None] * len(z_levels)

    for i, [section, z] in enumerate(zip(sections, z_levels)):
        flat_section, transform = section.to_planar()

        print(transform)

        try:
            poly = flat_section.polygons_full[0]
        except IndexError:
            break

        print poly.exterior.length

        ring = LinearRing(poly.exterior)
        ring = affine_transform(ring, [1, 0, 0, 1, transform[0][-1], transform[1][-1]])

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
            except PathGeneratorException:
                break

            intersection = list(intersection.coords)[0]

            print('Next intersection at', intersection)

            points.append(intersection)

            if start_point.distance(Point(intersection)) < laser_width:
                print('Close to start point')
                if len(points) > 2:
                    print('That\'s a wrap!')
                    break

        #for i in range(len(points)):
        #    points[i] = (points[i][0] + transform[0][-1], points[i][1] + transform[1][-1])

        section_points[i] = points

        ax.plot(x, y, z, 'r-', linewidth=1, alpha=0.5)
        ax.plot(sx, sy, z, 'b-', linewidth=1, alpha=0.5)
        px, py = zip(*points)
        ax.plot(px, py, z, 'bx')
        plt.axis('equal')
        #plt.show()

        if __name__ == '__builtin__':
            #raw_input()
            pass

    print section_points

    squares = []
    triangle_vertices = []
    triangle_faces = []

    for i in range(len(z_levels)):
        if section_points[i] is None:
            continue

        for j in range(len(section_points[i])-1):
            try:
                a = section_points[i][j] + (z_levels[i],)
                #b = section_points[i+1][j] + (z_levels[i+1],)
                #c = section_points[i+1][j+1] + (z_levels[i+1],)
                b = section_points[i][j] + (z_levels[i]+laser_height,)
                c = section_points[i][j+1] + (z_levels[i]+laser_height,)
                d = section_points[i][j+1] + (z_levels[i],)
            except (IndexError, TypeError):
                break

            x, y, z = zip(a, b, c, d, a)

            #ax.plot(x, y, z, 'k-')

            #sq = art3d.Poly3DCollection([zip(x, y, z)])
            #sq.set_color((0.5, 0.5, 0.5, 0.1))
            #sq.set_edgecolor('w')
            #ax.add_collection3d(sq)

            squares.append(trimesh.path.Path3D([entities.Line((0, 1)),
                                                entities.Line((1, 2)),
                                                entities.Line((2, 3)),
                                                entities.Line((3, 0))], [a, b, c, d]))

            l = len(triangle_vertices)
            triangle_vertices.extend([a, b, c, d])
            triangle_faces.extend([(l+0, l+2, l+1), (l+0, l+3, l+2)])

    #for face in mesh.faces:
        #vertices = [list(mesh.vertices[f]) for f in face]
        #tri = art3d.Poly3DCollection([vertices])
        #tri.set_color((0.2, 0.2, 0.6, 0.1))
        #tri.set_edgecolor((0.2, 0.2, 0.6))
        #ax.add_collection3d(tri)

    #plt.show()

    sc = scene.Scene()
    sc.add_geometry(np.sum(squares))
    sc.add_geometry(mesh)

    new_mesh = trimesh.Trimesh(vertices=triangle_vertices, faces=triangle_faces,
                               face_colors=[(1, 0.2, 0.2, 0.5)]*len(triangle_faces))
    sc.add_geometry(new_mesh)

    laser_outline, laser_mesh = laser_profile(squares[0].discrete[0])

    laser_mesh.visual = visual.create_visual(face_colors=[(0.2, 1, 0.2, 0.2)]*len(mesh.faces))

    sc.add_geometry(laser_mesh)

    sc.show()

    intersection = trimesh.Trimesh(**scad.boolean([mesh, laser_mesh], 'intersection'))
    intersection.show()

    raw_input()



