from Delaunay import Delaunay
import numpy
import matplotlib.pyplot as plt
import sys
from voronoi import Voronoi
import math

#point_file = open(sys.argv[1], "r")
point_file = open("points.txt", "r")
xyPoints = []
xyDup = []
xDup = []
yDup = []


def main_method():
    for curr_point in point_file:
        row = curr_point.split()
        xyPoints.append(numpy.array([float(row[0]), float(row[1])]))
    for i in range(0, len(xyPoints)):
        xyDup.append((xyPoints[i]).tolist())
    for i in range(0, len(xyDup)):
        xDup.append(xyDup[i][0])
        yDup.append(xyDup[i][1])

    delaunay = Delaunay(xyPoints)
    triangles = []
    for i in range(0, len(delaunay.triangle_indices)):
        a = delaunay.all_points[delaunay.triangle_indices[i][0]]
        b = delaunay.all_points[delaunay.triangle_indices[i][1]]
        c = delaunay.all_points[delaunay.triangle_indices[i][2]]
        triangles.append([a.tolist(), b.tolist(), c.tolist()])

    for i in range(0, len(triangles)):
        coordinates = tuple(triangles[i][0]), tuple(triangles[i][1]), tuple(triangles[i][2]), tuple(triangles[i][0])
        x, y = zip(*coordinates)
        plt.plot(x, y)

    # Edges of Original Delaunay
    edge_indices = list(delaunay.edges_of_triangle)
    edges = []
    for i in range(0, len(edge_indices)):
        edge = tuple([(tuple(delaunay.all_points[edge_indices[i][0]].tolist())), tuple((delaunay.all_points[edge_indices[i][1]]).tolist())])
        edges.append(edge)

    # Voronoi

    max_x = numpy.max(xDup)
    max_y = numpy.max(yDup)
    radius = math.sqrt(max_x ** 2 + max_y ** 2)  # Creation of a larger box
    center = numpy.mean(xyDup, axis=0)
    voronoi_xy = Voronoi(center, 50 * radius)

    # Adding Points one by one

    for s in xyDup:
        voronoi_xy.addPoint(s)
    voronoi_circles = voronoi_xy.getCircles()
    voronoi_vertices = []
    for i in range(0,len(voronoi_circles)):
        voronoi_vertices.append(voronoi_circles[i][0].tolist())

    # Add Points to xyDup
    xyDup2 = xyDup[:]
    xDup2 = xDup[:]
    yDup2 = yDup[:]
    for i in range(0, len(voronoi_vertices)):
        xyDup2.append(voronoi_vertices[i])
        xDup2.append(voronoi_vertices[i][0])
        yDup2.append(voronoi_vertices[i][1])

    # Compute DT for DT(VUP)
    xyPoints2 = []
    for k in xyDup2:
        xyPoints2.append(numpy.array(k))

    delaunay2 = Delaunay(xyPoints2)

    # Edges of New Delaunay
    edge_indices_2 = list(delaunay2.edges_of_triangle)
    edges_2 = []
    for i in range(0, len(edge_indices_2)):
        edge = tuple([tuple((delaunay2.all_points[edge_indices_2[i][0]]).tolist()), tuple((delaunay2.all_points[edge_indices_2[i][1]]).tolist())])
        edges_2.append(edge)

    print(edges)
    print(edges_2)
    good_edges = set(edges).intersection(set(edges_2))
    print(good_edges)














    plt.axis('equal')
    plt.show()
    print("Number of Triangles: " + str(len(delaunay.triangle_indices)))
    print("Number of edges:" + str(len(delaunay.edge_to_triangle_indices_Map)))
    print("Number of Vertices:" + str(len(delaunay.all_points)))


if __name__ == '__main__':
    main_method()























