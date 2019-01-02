import math
import heapq
import matplotlib.pyplot as plt
from matplotlib.path import Path
import time
import os

# Vertex Class Definition
class Vertex():
    def __init__(self, x="", y="", g="", p="", h="", polygon="", vertexNum = "", isConvex = "", neighbors=""):
        self.x = x  # x position of vertex
        self.y = y  # y position of vertex
        self.g = g  # g value - distance from start vertex to current vertex
        self.h = h  # heuristic value
        self.p = p  # parent vertex reference
        self.polygon = polygon
        self.vertexNum = vertexNum
        self.isConvex = isConvex
        self.neighbors = neighbors  # set of neighbor Vertex references

# A* Heuristic Function
def Heuristic(x, y, xgoal, ygoal):
    return ((xgoal - x)**2 + (ygoal - y)**2)**(0.5)

# Cost Function (Straight Line Distance)
def c(s,s_prime):
    return (((s.x - s_prime.x)**2 + (s.y - s_prime.y)**2)**(0.5))

# Take input map file and output vector of walls, polygons, and start/goal locations
def parseFile(fileName):
    fp = open(fileName)
    line = fp.readline()
    line_vector = line.split()
    for i in range(0, len(line_vector)):
        line_vector[i] = eval(line_vector[i])
    walls = line_vector
    line = fp.readline()
    line = fp.readline()
    polygons = []
    while line[0] == '(':
        line_vector = line.split()
        for i in range(0, len(line_vector)):
            line_vector[i] = eval(line_vector[i])
        polygons.append(line_vector)
        line = fp.readline()
    line = fp.readline()
    start_goal = []
    while line != '':
        line_vector = line.split()
        for i in range(0, len(line_vector)):
            line_vector[i] = eval(line_vector[i])
        start_goal.append(line_vector)
        line = fp.readline()
    return [walls, polygons, start_goal]


# Path 'codes' variable generating function with vertex number - 1 parameter n in path
def codes(n):
    code = [Path.MOVETO]
    for i in range(0, n):
        code.append(Path.LINETO)
    return code


# Given a list of polygon vertices, returns a list of polygon Paths
def polygonPath(polygon):
    polygon_verts = polygon
    polygon_verts.append(polygon[0])
    poly = Path(polygon_verts, codes(len(polygon_verts) - 1))
    del polygon_verts[-1]
    return poly

# Given a list of polygon vertices, returns a list of polygon Paths
def linePath(v1, v2):
    line_verts = [v1, v2]
    l = Path(line_verts, [Path.MOVETO, Path.LINETO])
    return l

# Returns angle in radians between 0 and 2pi relative to the
def angle(dy, dx):
    a = math.atan2(dy, dx)
    if a < 0:
        a = a + 2 * math.pi
    return a

# Remove convex vertices from set of V
def removeConvex(S):
    S_prime = []
    for i in range(0, len(S)):
        S_prime.append(S[i])
        S[i].append(S[i][0])
        polygon_path = Path(S[i], codes(len(S[i]) - 1))
        del S[i][-1]
        for j in range(0, len(S[i])):
            if j == len(S[i]) - 1:
                verts = [S[i][j], S[i][1]]
            elif j == len(S[i]) - 2:
                verts = [S[i][j], S[i][0]]
            else:
                verts = [S[i][j], S[i][j + 2]]
            line = Path(verts, codes(1))
            inside = polygon_path.intersects_path(line)
            if inside == False:
                S_prime[i].remove(S[i][j])
    return S_prime

def isBetween(a, b, c):

    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])

    # compare versus epsilon for floating point values, or != 0 if using integers
    if abs(crossproduct) > 0.1:
        # print "cross product is big"
        return False

    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
    if dotproduct < 0:
        # print "dot product is negative"
        return False

    squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
    if dotproduct > squaredlengthba:
        # print "dot product is bigger than squaredlengths"
        return False

    return True

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return False

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

# Checks if a point in inside a polygon
def insidePolygon(v, polygon):
    p = polygonPath(polygon)
    if p.contains_point(v):
        return True
    return False

def growObstacles(S, r):
    S_prime = []
    ConvexSet = []
    for i in range(0, len(S)):
        S_prime.append([])
        ConvexSet.append([])
        for j in range(0, len(S[i])):
            v1 = S[i][j]
            if j == 0:
                v0 = S[i][len(S[i]) - 1]
                v2 = S[i][j + 1]
            elif j == len(S[i]) - 1:
                v0 = S[i][j - 1]
                v2 = S[i][0]
            else:
                v0 = S[i][j - 1]
                v2 = S[i][j + 1]
            dx0 = v0[0] - v1[0]
            dy0 = v0[1] - v1[1]
            dx2 = v2[0] - v1[0]
            dy2 = v2[1] - v1[1]
            theta0 = angle(dy0, dx0)
            theta2 = angle(dy2, dx2)
            theta_avg = (theta0 + theta2) / 2
            if abs(theta2-theta0) < math.pi:
                if theta_avg > math.pi:
                    theta_avg -= math.pi
                else:
                    theta_avg += math.pi


            v1_new = (v1[0] + r * math.cos(theta_avg), v1[1] + r * math.sin(theta_avg))
            vertexConvex = insidePolygon(v1_new, S[i])
            if vertexConvex:
                if theta_avg > math.pi:
                    theta_avg -= math.pi
                else:
                    theta_avg += math.pi
                v1_new = (v1[0] + r * math.cos(theta_avg), v1[1] + r * math.sin(theta_avg))

            v1_new_0 = (v1_new[0] + 1*math.cos(theta_avg + (math.pi/2)), v1_new[1] + 1*math.sin(theta_avg + (math.pi/2)))
            v1_new_2 = (v1_new[0] + 1*math.cos(theta_avg - (math.pi/2)), v1_new[1] + 1*math.sin(theta_avg - (math.pi/2)))

            v0_out1 = (v0[0] + r*math.cos(theta0 - (math.pi/2)), v0[1] + r*math.sin(theta0 - (math.pi/2)))
            v0_out2 = (v1[0] + r * math.cos(theta0 - (math.pi/2)), v1[1] + r * math.sin(theta0 - (math.pi/2)))
            v2_out1 = (v2[0] + r * math.cos(theta2 + (math.pi / 2)), v2[1] + r * math.sin(theta2 + (math.pi / 2)))
            v2_out2 = (v1[0] + r * math.cos(theta2 + (math.pi / 2)), v1[1] + r * math.sin(theta2 + (math.pi / 2)))
            new_v1 = line_intersection((v0_out1,v0_out2), (v1_new_0, v1_new_2))
            new_v2 = line_intersection((v2_out1,v2_out2), (v1_new_0, v1_new_2))
            new_v1 = (round(new_v1[0], 4), round(new_v1[1], 4))
            new_v2 = (round(new_v2[0], 4), round(new_v2[1], 4))

            if vertexConvex:
                S_prime[i].append(new_v2)
                S_prime[i].append(new_v1)
                ConvexSet[i].append(1)
                ConvexSet[i].append(1)
            else:
                S_prime[i].append(new_v1)
                S_prime[i].append(new_v2)
                ConvexSet[i].append(0)
                ConvexSet[i].append(0)

    return [S_prime, ConvexSet]

def mergeSort(alist):
    if len(alist) > 1:
        mid = len(alist) // 2
        lefthalf = alist[:mid]
        righthalf = alist[mid:]

        mergeSort(lefthalf)
        mergeSort(righthalf)

        i = 0
        j = 0
        k = 0
        while i < len(lefthalf) and j < len(righthalf):
            if lefthalf[i][2] > righthalf[j][2]:
                alist[k] = lefthalf[i]
                i = i + 1
            elif lefthalf[i][2] == righthalf[j][2]:
                if lefthalf[i][1] > righthalf[j][1]:
                    alist[k] = lefthalf[i]
                    i = i + 1
                else:
                    alist[k] = righthalf[j]
                    j = j + 1
            else:
                alist[k] = righthalf[j]
                j = j + 1
            k = k + 1

        while i < len(lefthalf):
            alist[k] = lefthalf[i]
            i = i + 1
            k = k + 1

        while j < len(righthalf):
            alist[k] = righthalf[j]
            j = j + 1
            k = k + 1

def edgesIntersect(startVertex1, endVertex1, startVertex2, endVertex2):
    edge1_verts = [startVertex1, endVertex1]
    edge2_verts = [startVertex2, endVertex2]
    edge1 = Path(edge1_verts, codes(len(edge1_verts) - 1))
    edge2 = Path(edge2_verts, codes(len(edge2_verts) - 1))
    if edge1.intersects_path(edge2):
        return True
    else:
        return False


# VISIBILITY GRAPH

for mapNum in range(5,6):
    for start_goal_pair_number in range(5,6):

        start_time = time.time()

        # Initialize polygon paths
        mapname = "map_" + str(mapNum) + ".txt"
        map = parseFile(mapname)
        print "Running: ", mapname
        # DEFINE GRID
        mx = []
        my = []
        for i in range(0, 3):
            mx.append(map[0][i][0])
            my.append(map[0][i][1])

        turtlebot_radius = 0.2
        xstart = min(mx)
        xend = max(mx)
        ystart = min(my)
        yend = max(my)

        # Start and Goal Positions
        sx_start = map[2][start_goal_pair_number][0][0]
        sy_start = map[2][start_goal_pair_number][0][1]
        sx_goal = map[2][start_goal_pair_number][1][0]
        sy_goal = map[2][start_goal_pair_number][1][1]
        startPrint = (sx_start, sy_start)
        goalPrint = (sx_goal, sy_goal)
        print "Start/Goal Pair", start_goal_pair_number, ":", startPrint, "to", goalPrint  # Define S as set of polygon vertices
        print "Creating Reduced Visibility Graph"
        S = map[1]

        # Grow obstacles by size of Turtlebot Robot and get grown vertex set and also convex vertex set
        [S_grown, ConvexSet] = growObstacles(S, turtlebot_radius)

        # Plot original and grown obstacle map
        for i in range(0, len(S)):
            x_coors = []
            y_coors = []
            for j in range(0, len(S[i])):
                x_coors.append(S[i][j][0])
                y_coors.append(S[i][j][1])
            x_coors.append(x_coors[0])
            y_coors.append(y_coors[0])
            plt.plot(x_coors, y_coors, 'bo-')

        for i in range(0, len(S_grown)):
            x_coors = []
            y_coors = []
            for j in range(0, len(S_grown[i])):
                x_coors.append(S_grown[i][j][0])
                y_coors.append(S_grown[i][j][1])
            x_coors.append(x_coors[0])
            y_coors.append(y_coors[0])
            plt.plot(x_coors, y_coors, 'ro-')

        plt.show()

        # Initialize the graph G and add vertices from polygon list
        G = []
        for i in range(0, len(S_grown)):
            for j in range(0, len(S_grown[i])):
                v = S_grown[i][j]
                convex = ConvexSet[i][j]
                G.append(Vertex(x=v[0], y = v[1], p = None, h = 0, polygon = i, vertexNum = j, isConvex = convex, neighbors = set()))

        # # Add edges within polygons, not including ones for convex vertices
        # count = 0
        # for i in range(0, len(S_grown)):
        #     for j in range(0, len(S_grown[i])):
        #         if G[count].isConvex == 0:
        #             if j == len(S_grown[i])-1:
        #                 if G[count-j].isConvex == 0:
        #                     G[count].neighbors.add(G[count-j])
        #                 if G[count - 1].isConvex == 0:
        #                     G[count].neighbors.add(G[count-1])
        #             elif j == 0:
        #                 if G[count + 1].isConvex == 0:
        #                     G[count].neighbors.add(G[count+1])
        #                 if G[count+len(S_grown[i])-1].isConvex == 0:
        #                     G[count].neighbors.add(G[count+len(S_grown[i])-1])
        #             else:
        #                 if G[count - 1].isConvex == 0:
        #                     G[count].neighbors.add(G[count-1])
        #                 if G[count + 1].isConvex == 0:
        #                     G[count].neighbors.add(G[count+1])
        #         count += 1


        # Add start and goal vertices to G
        startVertex = Vertex(x=sx_start, y=sy_start, p=None, h=0, polygon=-1, vertexNum=0, isConvex=0, neighbors=set())
        goalVertex = Vertex(x=sx_goal, y=sy_goal, p=None, h=0, polygon=-2, vertexNum=0, isConvex=0, neighbors=set())
        G.append(startVertex)
        G.append(goalVertex)

        # Add edges in graph as neighbors of vertices
        for i in range(0, len(G)):
            for j in range(0, len(G)):
                if i != j and G[i].isConvex == 0 and G[j].isConvex == 0:
                    intersection_count = 0
                    vertexLine = [(G[i].x, G[i].y), (G[j].x, G[j].y)]
                    roundDigits = 5
                    vertexLine_rounded = [(round(vertexLine[0][0], roundDigits), round(vertexLine[0][1], roundDigits)),(round(vertexLine[1][0], roundDigits), round(vertexLine[1][1], roundDigits))]
                    for a in range(0, len(S_grown)):
                        midPoint = [(G[i].x + G[j].x)/2, (G[i].y + G[j].y)/2]
                        if insidePolygon(midPoint, S_grown[a]):
                            intersection_count += 1
                            # print "case 1 - vertex midpoint in a polygon"
                            continue
                        for b in range(0, len(S_grown[a])):
                            if b == len(S_grown[a])-1:
                                polygonLine = [S_grown[a][b], S_grown[a][0]]
                            else:
                                polygonLine = [S_grown[a][b], S_grown[a][b+1]]

                            intersection_point = line_intersection(vertexLine, polygonLine)
                            polygonLine_rounded = [(round(polygonLine[0][0], roundDigits), round(polygonLine[0][1], roundDigits)), (round(polygonLine[1][0], roundDigits), round(polygonLine[1][1], roundDigits))]


                            # The polygonLine is the vertexLine itself
                            if (vertexLine_rounded[0][0] == polygonLine_rounded[0][0] and vertexLine_rounded[0][1] == polygonLine_rounded[0][1]) and (vertexLine_rounded[1][0] == polygonLine_rounded[1][0] and vertexLine_rounded[1][1] == polygonLine_rounded[1][1]) or (vertexLine_rounded[1][0] == polygonLine_rounded[0][0] and vertexLine_rounded[1][1] == polygonLine_rounded[0][1]) and (vertexLine_rounded[0][0] == polygonLine_rounded[1][0] and vertexLine_rounded[0][1] == polygonLine_rounded[1][1]):
                                continue
                            if intersection_point != False:
                                intersection_point_rounded = [round(intersection_point[0], roundDigits), round(intersection_point[1], roundDigits)]
                            else:
                                intersection_point_rounded = False
                            # print "-------------------------------"
                            # print "intersection=", intersection_point_rounded
                            # print "vertexLine=", [vertexLine_rounded[0][0], vertexLine_rounded[0][1],vertexLine_rounded[1][0], vertexLine_rounded[1][1]]
                            # print "polygonLine=", [polygonLine_rounded[0][0], polygonLine_rounded[0][1],polygonLine_rounded[1][0], polygonLine_rounded[1][1]]

                            # If the lines do not intersect, continue
                            if intersection_point == False:
                                # print "case A - no intersection at all"
                                continue
                            # If the intersection point is the first vertex, continue
                            elif intersection_point_rounded[0] == vertexLine_rounded[0][0] and intersection_point_rounded[1] == vertexLine_rounded[0][1]:
                                # print "case B - intersection at vertex i"
                                continue
                            # If the intersection point is the second vertex, continue
                            elif intersection_point_rounded[0] == vertexLine_rounded[1][0] and intersection_point_rounded[1] == vertexLine_rounded[1][1]:
                                # print "case C - intersection at vertex j"
                                continue
                            # If the intersection point is on the vertexLine and on polygonLine
                            elif isBetween(vertexLine[0], vertexLine[1], intersection_point) and isBetween(polygonLine[0], polygonLine[1], intersection_point):
                                # print "case D - intersection is between vertexLine and polygonLine"
                                intersection_count += 1
                                break
                            # If the intersection point is the vertex of a polygon edge, and its not either the first or second vertex, add intersection
                            elif (intersection_point_rounded[0] == polygonLine_rounded[0][0] and intersection_point_rounded[1] == polygonLine_rounded[0][1]) or (intersection_point_rounded[0] == polygonLine_rounded[1][0] and intersection_point_rounded[1] == polygonLine_rounded[1][1]):
                                # print "case E - intersection is at polygon vertex"
                                intersection_count += 1
                                break
                            else:
                                # print "case F - no intersection"
                                continue
                    if intersection_count == 0:
                        G[i].neighbors.add(G[j])

        # # Print Polygons
        # for i in range(0, len(S_grown)):
        #     x_coors = []
        #     y_coors = []
        #     for j in range(0, len(S_grown[i])):
        #         x_coors.append(S_grown[i][j][0])
        #         y_coors.append(S_grown[i][j][1])
        #     x_coors.append(x_coors[0])
        #     y_coors.append(y_coors[0])
        #     plt.plot(x_coors, y_coors, 'ro-')
        #
        # # Print Edges
        # for v in G:
        #     for n in v.neighbors:
        #         plt.plot([v.x, n.x],[v.y, n.y] , 'bo-')
        #
        # plt.show()


        # Print Visibility Graph

        # Print Edges
        for v in G:
            for n in v.neighbors:
                plt.plot([v.x, n.x],[v.y, n.y] , 'bo-')

        # Print Polygons
        for i in range(0, len(S_grown)):
            x_coors = []
            y_coors = []
            for j in range(0, len(S_grown[i])):
                x_coors.append(S_grown[i][j][0])
                y_coors.append(S_grown[i][j][1])
            x_coors.append(x_coors[0])
            y_coors.append(y_coors[0])
            plt.plot(x_coors, y_coors, 'ro-')
        # plt.show()


        map_time = time.time() - start_time
        print "Generating Visibility Graph Time Taken:", map_time, "sec"
        start_time = time.time()


        # A* ALGORITHM
        print "Starting A*"


        # A* ALGORITHM

        # add A* heuristics data to vertices
        for i in range(0, len(G)):
            G[i].h = Heuristic(G[i].x, G[i].y, goalVertex.x, goalVertex.y)

        def UpdateVertexA_star(s, s_prime, fringe):
            if (s.g + c(s, s_prime) < s_prime.g):
                s_prime.g = s.g + c(s, s_prime)
                s_prime.p = s
                s_prime_tuple = [(val, key) for (val, key) in fringe if key == s_prime]
                if len(s_prime_tuple) == 1:
                    fringe.remove(s_prime_tuple[0])
                    heapq.heapify(fringe)
                heapq.heappush(fringe, (s_prime.g + s_prime.h, s_prime))


        # A* Main
        startVertex.g = 0
        startVertex.p = startVertex
        fringe = []
        heapq.heapify(fringe)
        heapq.heappush(fringe, (startVertex.g + startVertex.h, startVertex))
        closed = set()
        while len(fringe) != 0:
            s = list(heapq.heappop(fringe))[1]
            if s == goalVertex:
                print "***path found***"
                break
            closed.add(s)
            for s_prime in s.neighbors:
                if s_prime not in closed:
                    s_prime_tuple = [(val, key) for (val, key) in fringe if key == s_prime]
                    if len(s_prime_tuple) != 1:
                        s_prime.g = float('inf')
                        s_prime.p = None
                    UpdateVertexA_star(s, s_prime, fringe)

        a_star_time = time.time() - start_time
        print "A* time taken:", a_star_time, "sec"

        # Backtrack from goal to start vertex by using parent attribute of each vertex to get the solution path vertices
        n = goalVertex
        path_flipped = [(n.x,n.y)]
        while n != startVertex:
            n = n.p
            path_flipped.append((n.x,n.y))

        # Flip the path array so it starts at the start and ends at the goal
        path = []
        pathLen = len(path_flipped)
        for i in range(0, pathLen):
            path.append(path_flipped[pathLen - 1 - i])

        # Round path coordinates to round_digits
        round_digits = 4
        for i in range(0, len(path)):
            path[i] = list(path[i])
            path[i][0] = round(path[i][0], round_digits)
            path[i][1] = round(path[i][1], round_digits)
            path[i] = tuple(path[i])
        # print path

        closed_xcoors = []
        closed_ycoors = []

        open_xcoors = []
        open_ycoors = []

        for i in closed:
            closed_xcoors.append(i.x)
            closed_ycoors.append(i.y)

        for i in fringe:
            open_xcoors.append(i[1].x)
            open_ycoors.append(i[1].y)

        AstarExpandednodes = len(open_xcoors)
        print "A* number of expanded nodes: ", AstarExpandednodes

        # Graphing A* Map
        # Make x and y coordinates vectors for matlibplot input from the path vertex tuples
        x_coors = []
        y_coors = []
        for i in range(0, len(path)):
            x_coors.append(list(path[i])[0])
            y_coors.append(list(path[i])[1])

        # Red dots are center points of blocked cells
        # Blue dots are path vertices
        plt.ylim((ystart, yend))   # set the ylim to bottom, top
        plt.xlim((xstart, xend))   # set the xlim to bottom, top
        plt.plot(x_coors,y_coors, 'go-',linewidth=3.0)
        plt.plot(sx_start,sy_start, 'co')
        plt.plot(sx_goal,sy_goal, 'go-')

        directory = "Visibility_A*_results"
        if not os.path.exists(directory):
            os.makedirs(directory)
        fname = "Visibility_A*_results/path" +str(start_goal_pair_number) +".png"
        resultFile = directory + "/result" + str(start_goal_pair_number) + ".txt"
        plt.savefig(fname)
        # plt.show()
        plt.close()

        print "Path coordinates"
        the_file = open(resultFile, "w+")
        the_file.truncate(0)
        for Astarpath in path:
            resultpath = str(Astarpath[0]) + " " + str(Astarpath[1])
            print resultpath
            the_file.write(resultpath + "\n")
        the_file.close()

        totalTime = a_star_time + map_time
        print "Total time taken:", totalTime, "sec"


        # Getting path distance
        # Distance Function (Straight Line Distance)
        def dist(a,b,c,d):
            return (((a - c)**2 + (b - d)**2)**(0.5))
        totalDist = 0;
        for i in range(0, len(x_coors)-1):
            totalDist += dist(x_coors[i], y_coors[i], x_coors[i+1], y_coors[i+1])
        print "Total distance of path:", totalDist, "meters"

        fp = open(str(directory) + "/dataFile.txt", 'a')
        fp.write("Map " + str(mapNum) + ", Start/Goal " + str(start_goal_pair_number) + "\n")
        fp.write("Visibility Graph A* Total Expanded Nodes: " + str(AstarExpandednodes) + "\n")
        fp.write("Visibility Graph A* Total Time: " + str(totalTime) + "\n")
        fp.write("Visibility Graph A* Total Path Distance: " + str(totalDist) + "\n")
        fp.write("\n\n\n")
        fp.close()
