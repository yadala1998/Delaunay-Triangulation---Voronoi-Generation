import numpy as np
import math
import copy

# Source Of Algorithm: http://www.s-hull.org/paper/s_hull.pdf

class Vertex(object):

    def __init__(self, x, y):
        self.xValue = x
        self.yValue = y
        self.faces = []
        self.edges = []

    def getVertexInfo(self):
        return self.xValue, self.yValue

    def addFaceToVertex(self, face_object):
        self.faces.append(face_object)

    def addEdgeToVertex(self, edge_object):
        self.edges.append(edge_object)

    def getFaces(self):
        return self.faces

    def getEdges(self):
        return self.edges


class Edge(object):

    def __init__(self, vertex_1, vertex_2):  # vertex objects not points
        self.vertex_one = vertex_1
        self.vertex_two = vertex_2
        self.faces = []

    def getBothVertices(self):
        return self.vertex_one.getVertexInfo(), self.vertex_two.getVertexInfo()

    def addFaceToEdge(self, face_object):
        self.faces.append(face_object)

    def getFaces(self):
        return self.faces


class Face(object):

    def __init__(self, vertex_1, vertex_2, vertex_3):
        self.vertex_one = vertex_1  # vertex objects
        self.vertex_one = vertex_1  # vertex objects
        self.vertex_two = vertex_2
        self.vertex_three = vertex_3
        self.edges = []

    def getFaceInfo(self):
        return self.vertex_one.getVertexInfo(), self.vertex_two.getVertexInfo(), self.vertex_three.getVertexInfo()

    def addEdge(self, Edge_object):
        self.edges.append(Edge_object)

    def getEdges(self):
        return self.edges


class Delaunay:

    def __init__(self, all_points):

        # data structuedge_set for triangulation
        self.all_points = all_points[:]  # copy
        self.triangle_indices = []  # cells
        self.edge_to_triangle_indices_Map = {}  # edge to triangle(s) map
        self.edges_of_triangle = set()
        self.All_edges = []
        self.All_faces = []
        self.All_Vertices = []

        # compute center of gravity
        center_of_gravity = np.zeros((2,), np.float64)
        for pt in all_points:
            center_of_gravity += pt
        center_of_gravity = center_of_gravity / len(all_points)

        # sort by distance
        def dot_product(pt):
            d = pt - center_of_gravity
            return np.dot(d, d)

        self.all_points.sort(key=dot_product)

        # create first triangle
        index = 0
        break_the_loop = False
        while not break_the_loop and index + 2 < len(all_points):
            area = self.get_area(index, index + 1, index + 2)
            if abs(area) < self.EPS:
                del self.all_points[index]
            else:
                break_the_loop = True

        if index <= len(self.all_points) - 3:
            triangle = [index, index + 1, index + 2]
            self.counter_clockwise_rotation(triangle)
            self.triangle_indices.append(triangle)

            """ 
                 A
                / \ 
               B---C
                
            """
            # Add to the edge list
            edge_AB = (triangle[0], triangle[1])
            self.edges_of_triangle.add(edge_AB)

            # Add to the edge:Triangle map
            edge_AB = self.sortSizeWise(edge_AB[0], edge_AB[1])
            self.edge_to_triangle_indices_Map[edge_AB] = [0, ]

            # Add to the egde list
            edge_BC = (triangle[1], triangle[2])
            self.edges_of_triangle.add(edge_BC)

            # Add to the edge:Triangle map
            edge_BC = self.sortSizeWise(edge_BC[0], edge_BC[1])
            self.edge_to_triangle_indices_Map[edge_BC] = [0, ]

            # Add to the edge list
            edge_CA = (triangle[2], triangle[0])
            self.edges_of_triangle.add(edge_CA)

            # Add to the edge:Triangle Map
            edge_CA = self.sortSizeWise(edge_CA[0], edge_CA[1])
            self.edge_to_triangle_indices_Map[edge_CA] = [0, ]

        else:
            # No Triangle could be constructed
            return

        # add additional all_points
        for i in range(3, len(self.all_points)):
            self.addPoint(i)

        #self.add_To_data_structure()

    # def add_To_data_structure(self):
    #
    #     faces = []
    #     for i in range(0, len(self.triangle_indices)):
    #         a = self.all_points[self.triangle_indices[i][0]]
    #         b = self.all_points[self.triangle_indices[i][1]]
    #         c = self.all_points[self.triangle_indices[i][2]]
    #         faces.append([a.tolist(), b.tolist(), c.tolist()])
    #
    #     for i in range(0, len(faces)):
    #         V1 = Vertex(faces[i][0][0], faces[i][0][1])
    #         V2 = Vertex(faces[i][1][0], faces[i][1][1])
    #         V3 = Vertex(faces[i][2][0], faces[i][2][1])
    #         E1 = Edge(V1, V2)
    #         E2 = Edge(V2, V3)
    #         E3 = Edge(V3, V1)
    #
    #         # duplicate return values
    #         V1_dup = self.duplicateVertexExists(V1, self.All_Vertices)
    #         V2_dup = self.duplicateVertexExists(V2, self.All_Vertices)
    #         V3_dup = self.duplicateVertexExists(V3, self.All_Vertices)
    #         E1_dup = self.duplicateEdgeExists(E1, self.All_edges)
    #         E2_dup = self.duplicateEdgeExists(E2, self.All_edges)
    #         E3_dup = self.duplicateEdgeExists(E3, self.All_edges)
    #
    #         # Adding Faces
    #         self.All_faces.append(Face(V1, V2, V3))
    #
    #         # checking and adding
    #         if not V1_dup[0]:
    #             self.All_Vertices.append(V1)
    #             V1.addFaceToVertex(self.All_faces[i])
    #             V1.addEdgeToVertex(E1)
    #             V1.addEdgeToVertex(E3)
    #         else:
    #             self.All_Vertices[V1_dup[1]].addFaceToVertex(self.All_faces[i])
    #             self.All_Vertices[V1_dup[1]].addEdgeToVertex(E1)
    #             self.All_Vertices[V1_dup[1]].addEdgeToVertex(E3)
    #
    #         if V2_dup[0] == False:
    #             self.All_Vertices.append(V2)
    #             V2.addFaceToVertex(self.All_faces[i])
    #             V2.addEdgeToVertex(E1)
    #             V2.addEdgeToVertex(E2)
    #         else:
    #             self.All_Vertices[V2_dup[1]].addFaceToVertex(self.All_faces[i])
    #             self.All_Vertices[V2_dup[1]].addEdgeToVertex(E1)
    #             self.All_Vertices[V2_dup[1]].addEdgeToVertex(E2)
    #
    #         if V3_dup[0] == False:
    #             self.All_Vertices.append(V3)
    #             V3.addFaceToVertex(self.All_faces[i])
    #             V3.addEdgeToVertex(E2)
    #             V3.addEdgeToVertex(E3)
    #         else:
    #             self.All_Vertices[V3_dup[1]].addFaceToVertex(self.All_faces[i])
    #             self.All_Vertices[V3_dup[1]].addEdgeToVertex(E2)
    #             self.All_Vertices[V3_dup[1]].addEdgeToVertex(E3)
    #
    #         if E1_dup[0] == False:
    #             self.All_edges.append(E1)
    #             E1.addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(E1)
    #         else:
    #             self.All_edges[E1_dup[1]].addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(self.All_edges[E1_dup[1]])
    #
    #         if E2_dup[0] == False:
    #             self.All_edges.append(E2)
    #             E2.addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(E2)
    #         else:
    #             self.All_edges[E2_dup[1]].addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(self.All_edges[E2_dup[1]])
    #
    #         if E3_dup[0] == False:
    #             self.All_edges.append(E3)
    #             E3.addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(E3)
    #         else:
    #             self.All_edges[E3_dup[1]].addFaceToEdge(self.All_faces[i])
    #             self.All_faces[i].addEdge(self.All_edges[E3_dup[1]])
    #
    #     return
    #
    EPS = 1.23456789e-14
    # def duplicateVertexExists(self, Vertex_object, All_Vertices):
    #     for i in range(0, len(All_Vertices)):
    #         if Vertex_object.getVertexInfo() == All_Vertices[i].getVertexInfo():
    #             return True, i
    #     return False, 0
    #
    # # Helper Method for Checking duplicate Edges
    # def duplicateEdgeExists(self, Edge_object, All_edges):
    #     for i in range(0, len(All_edges)):
    #         if (np.array_equal(Edge_object.getBothVertices()[0],
    #                            All_edges[i].getBothVertices()[0]) == True or np.array_equal(
    #                 Edge_object.getBothVertices()[0], All_edges[i].getBothVertices()[1]) == True) \
    #                 and (
    #                 np.array_equal(Edge_object.getBothVertices()[1],
    #                                All_edges[i].getBothVertices()[0]) == True or np.array_equal(Edge_object.getBothVertices()[1], All_edges[i].getBothVertices()[0])):
    #             return True, i
    #     return False, 0

    def get_triangle_indices(self):
        return self.triangle_indices

    def get_edge(self):
        return self.edge_to_triangle_indices_Map.keys()

    def get_area(self, ip0, ip1, ip2):
        d1 = self.all_points[ip1] - self.all_points[ip0]
        d2 = self.all_points[ip2] - self.all_points[ip0]
        return (d1[0] * d2[1] - d1[1] * d2[0])

    def findVisibleEdge(self, ip, edge):
        area = self.get_area(ip, edge[0], edge[1])
        if area < self.EPS:
            return True
        return False

    def counter_clockwise_rotation(self, ips):
        area = self.get_area(ips[0], ips[1], ips[2])
        if area < -self.EPS:
            ip1, ip2 = ips[1], ips[2]
            # swap
            ips[1], ips[2] = ip2, ip1

    def flipOneEdge(self, edge):
        edge_set = set()
        triangles = self.edge_to_triangle_indices_Map.get(edge, [])
        if len(triangles) < 2:
            return edge_set
        else:
            triangle_index_1, triangle_index_2 = triangles
            triangle_1 = self.triangle_indices[triangle_index_1]
            triangle_2 = self.triangle_indices[triangle_index_2]

            # find the opposite vertices, not part of the edge
            opposite_vertex_1 = -1
            opposite_vertex_2 = -1
            for i in range(3):
                if not triangle_1[i] in edge:
                    opposite_vertex_1 = triangle_1[i]
                if not triangle_2[i] in edge:
                    opposite_vertex_2 = triangle_2[i]

            # compute the 2 angles at the opposite vertices
            da1 = self.all_points[edge[0]] - self.all_points[opposite_vertex_1]
            db1 = self.all_points[edge[1]] - self.all_points[opposite_vertex_1]
            da2 = self.all_points[edge[0]] - self.all_points[opposite_vertex_2]
            db2 = self.all_points[edge[1]] - self.all_points[opposite_vertex_2]

            angle1 = abs(math.atan2(self.get_area(opposite_vertex_1, edge[0], edge[1]), np.dot(da1, db1)))
            angle2 = abs(math.atan2(self.get_area(opposite_vertex_2, edge[1], edge[0]), np.dot(da2, db2)))

            # InCircle Test
            if angle1 + angle2 > math.pi * (1.0 + self.EPS):
                new_triangle_1 = [opposite_vertex_1, edge[0], opposite_vertex_2]  # triangle a
                new_triangle_2 = [opposite_vertex_1, opposite_vertex_2, edge[1]]  # triangle b

                # update the triangle data structure
                self.triangle_indices[triangle_index_1] = new_triangle_1
                self.triangle_indices[triangle_index_2] = new_triangle_2

                # remove this edge
                del self.edge_to_triangle_indices_Map[edge]

                # add new edge
                e = self.sortSizeWise(opposite_vertex_1, opposite_vertex_2)
                self.edge_to_triangle_indices_Map[e] = [triangle_index_1, triangle_index_2]

                # modify two edge entries which now connect to a different triangle
                e = self.sortSizeWise(opposite_vertex_1, edge[1])
                v = self.edge_to_triangle_indices_Map[e]
                for i in range(len(v)):
                    if v[i] == triangle_index_1:
                        v[i] = triangle_index_2
                edge_set.add(e)

                e = self.sortSizeWise(opposite_vertex_2, edge[0])
                v = self.edge_to_triangle_indices_Map[e]
                for i in range(len(v)):
                    if v[i] == triangle_index_2:
                        v[i] = triangle_index_1
                edge_set.add(e)
                # Add for possible flipping
                edge_set.add(self.sortSizeWise(opposite_vertex_1, edge[0]))
                edge_set.add(self.sortSizeWise(opposite_vertex_2, edge[1]))

        return edge_set

    def flipEdges(self):
        # start with all the edges
        edgeSet = set(self.edge_to_triangle_indices_Map.keys())
        continueFlipping = True

        while continueFlipping:
            newEdgeSet = set()
            for edge in edgeSet:
                newEdgeSet |= self.flipOneEdge(edge)

            edgeSet = copy.copy(newEdgeSet)
            continueFlipping = (len(edgeSet) > 0)

    def addPoint(self, ip):
        # collection for later updates
        edges_of_triangleToRemove = set()
        edges_of_triangleToAdd = set()

        for edge in self.edges_of_triangle:

            if self.findVisibleEdge(ip, edge):
                # create new triangle
                newTriangle = [edge[0], edge[1], ip]
                newTriangle.sort()
                self.counter_clockwise_rotation(newTriangle)
                self.triangle_indices.append(newTriangle)

                # update the edge to triangle map
                e = list(edge[:])
                e.sort()
                index_of_Triangle = len(self.triangle_indices) - 1
                self.edge_to_triangle_indices_Map[tuple(e)].append(index_of_Triangle)

                # add the two boundary edges
                e1 = [ip, edge[0]]
                e1.sort()
                e1 = tuple(e1)

                e2 = [edge[1], ip]
                e2.sort()
                e2 = tuple(e2)

                v1 = self.edge_to_triangle_indices_Map.get(e1, [])
                v1.append(index_of_Triangle)

                v2 = self.edge_to_triangle_indices_Map.get(e2, [])
                v2.append(index_of_Triangle)
                self.edge_to_triangle_indices_Map[e1] = v1
                self.edge_to_triangle_indices_Map[e2] = v2

                # keep track of the boundary edges to update
                edges_of_triangleToRemove.add(edge)
                edges_of_triangleToAdd.add((edge[0], ip))
                edges_of_triangleToAdd.add((ip, edge[1]))

        # update the boundary edges
        for edge in edges_of_triangleToRemove:
            self.edges_of_triangle.remove(edge)
        for edge in edges_of_triangleToAdd:
            EdgeSorted = list(edge)
            EdgeSorted.sort()
            EdgeSorted = tuple(EdgeSorted)
            if len(self.edge_to_triangle_indices_Map[EdgeSorted]) == 1:
                # only add boundary edge if it does not appear
                # twice in different order
                self.edges_of_triangle.add(edge)

        # recursively flip edges
        flipped = True
        while flipped:
            flipped = self.flipEdges()

    def sortSizeWise(self, i1, i2):
        if i1 < i2:
            return (i1, i2)
        return (i2, i1)




