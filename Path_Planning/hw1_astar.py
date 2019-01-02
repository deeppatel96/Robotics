import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import os
import time

# Take input map file and output vector of walls, polygons, and start/goal locations
def parseFile(fileName):

    fp = open(fileName)
    line = fp.readline()
    line_vector = line.split()
    for i in range(0, len(line_vector)):
        line_vector[i] = eval(line_vector[i])
    walls = line_vector
    polygons = []
    line = fp.readline()
    line = fp.readline()
    while line[0] =='(':
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

    fp.close()
    return [walls, polygons, start_goal]

# Vertex Class Definition
class Vertex():
    def __init__(self, x="", y="", g="", p="", h="", neighbors=""):
        self.x = x  # x position of vertex
        self.y = y  # y position of vertex
        self.g = g  # g value - distance from start vertex to current vertex
        self.h = h  # heuristic value
        self.p = p  # parent vertex reference
        self.neighbors = neighbors  # set of neighbor Vertex references


# A* Heuristic Function
def aStarHeuristic(x, y, xgoal, ygoal):
    return 2 ** (0.5) * min(abs(x - xgoal), abs(y - ygoal)) + max(abs(x - xgoal), abs(y - ygoal)) - min(abs(x - xgoal), abs(y - ygoal))

# FDA* Heuristic Function
def fdaStarHeuristic(x, y, xgoal, ygoal):
    return ((xgoal - x)**2 + (ygoal - y)**2)**(0.5)

# Cost Function (Straight Line Distance)
def c(s,s_prime):
    return (((s.x - s_prime.x)**2 + (s.y - s_prime.y)**2)**(0.5))

# Path 'codes' variable genererating function with vertex number parameter n in path
def codes(n):
    c = [Path.MOVETO]
    for i in range(0, n):
        c.append(Path.LINETO)
    return c

# Given a list of polygon vertices, returns a list of polygon Paths
def polygonPath(polygon):
    polygon_verts = polygon
    polygon_verts.append(polygon[0])
    poly = Path(polygon_verts, codes(len(polygon_verts) - 1))
    return poly

for mapNum in range(1, 6):
    for start_goal_pair_number in range(5, 6):
        start_time = time.time()

        # PRE-PROCESSING
        mapname = "map_"+str(mapNum)+".txt"

        # Initialize polygon paths
        map = parseFile(mapname)
        print "Running: ", mapname
        polygons = []
        for i in range(0, len(map[1])):
            polygons.append(polygonPath(map[1][i]))

        # DEFINE GRID
        mx = []
        my = []
        for i in range(0, 3):
            mx.append(map[0][i][0])
            my.append(map[0][i][1])

        turtlebot_radius = 0.20
        xstart = min(mx)
        xend = max(mx)
        ystart = min(my)
        yend = max(my)
        xdist = xend - xstart
        ydist = yend - ystart
        xlen = 220
        ylen = 220
        deltax = xdist / float(xlen)
        deltay = ydist / float(ylen)

        # Start and Goal Positions
        nodes_expanded = []
        length = len(map[2])

        sx_start = map[2][start_goal_pair_number][0][0]
        sy_start = map[2][start_goal_pair_number][0][1]
        sx_goal = map[2][start_goal_pair_number][1][0]
        sy_goal = map[2][start_goal_pair_number][1][1]
        startPrint = (sx_start,sy_start)
        goalPrint = (sx_goal,sy_goal)
        print "Start/Goal Pair", start_goal_pair_number,":", startPrint, "to", goalPrint
        print "Creating Grid Map"

        # Create empty matrix
        M = np.empty((xlen, ylen))

        # Set codes for 4 vertices so it does not have to be called every iteration of the loop
        codes4 = codes(4)

        # Check each cell it is intersecting or filled by an obstacle. Mark cell as 1 if it contains an obstacle
        for y in range(0, ylen):
            for x in range(0, xlen):
                cell_verts = [(x * deltax + xstart, y * deltay + ystart),
                              (x * deltax + xstart, y * deltay + ystart + deltay),
                              (x * deltax + xstart + deltax, y * deltay + ystart + deltay),
                              (x * deltax + xstart + deltax, y * deltay + ystart),
                              (x * deltax + xstart, y * deltay + ystart)]
                cell = Path(cell_verts, codes4)
                count = 0
                for i in range(0, len(polygons)):
                    count += polygons[i].intersects_path(cell, filled=True)
                if count > 0:
                    M[y, x] = 1
                else:
                    M[y, x] = 0

        # Create boundary around obstacles to account for size of robot
        n = 2
        num_iter = int(math.ceil(turtlebot_radius / deltax))
        for i in range(0, num_iter):
            for y in range(0, ylen):
                for x in range(0, xlen):
                    if M[y, x] == n - 1 + i:
                        if y != 0:
                            if M[y - 1, x] == 0:
                                M[y - 1, x] = n + i
                            if x != 0:
                                if M[y - 1, x - 1] == 0:
                                    M[y - 1, x - 1] = n + i
                            if x != xlen - 1:
                                if M[y - 1, x + 1] == 0:
                                    M[y - 1, x + 1] = n + i
                        if y != ylen - 1:
                            if M[y + 1, x] == 0:
                                M[y + 1, x] = n + i
                            if x != 0:
                                if M[y + 1, x - 1] == 0:
                                    M[y + 1, x - 1] = n + i
                            if x != xlen - 1:
                                if M[y + 1, x + 1] == 0:
                                    M[y + 1, x + 1] = n + i
                        if x != 0:
                            if M[y, x - 1] == 0:
                                M[y, x - 1] = n + i
                        if x != xlen - 1:
                            if M[y, x + 1] == 0:
                                M[y, x + 1] = n + i

        # CHANGE ALL OBSTACLES VALUES FROM 1-n TO ONLY 1
        for j in range(0, ylen):
            for i in range(0, xlen):
                if M[j, i] != 0:
                    M[j, i] = 1

        # GROW BOUNDARY OF GRID MAP
        for j in range(0, num_iter):
            for i in range(0, xlen):
                M[j, i] = 1  # grow top boundary by size of robot
                M[ylen - 1 - j, i] = 1  # grow bottom boundary by size of robot
        for j in range(0, ylen):
            for i in range(0, num_iter):
                M[j, i] = 1  # grow left boundary by size of robot
                M[j, xlen - 1 - i] = 1  # grow right boundary by size of robot

        # # PRINT HEAT MAP OF GRID MAP
        # A = np.flipud(M);
        # plt.imshow(M, cmap='hot', interpolation='nearest')
        # plt.show()

        print "Creating Vertex Map"
        # INITIALIZE VERTEX MAP
        N = []
        for j in range(0, ylen + 1):
            N.append([])
            for i in range(0, xlen + 1):
                N[j].append(Vertex(x=i * deltax + xstart, y=j * deltay + ystart, neighbors=set()))

        # ADD NEIGHBOR SETS TO EACH VERTEX
        for j in range(0, ylen + 1):
            for i in range(0, xlen + 1):
                if j == 0:
                    if i == 0:
                        if M[j, i] == 0:
                            N[j][i].neighbors.add(N[j][i + 1])
                            N[j][i].neighbors.add(N[j + 1][i])
                            N[j][i].neighbors.add(N[j + 1][i + 1])
                    elif i == xlen:
                        if M[j, i - 1] == 0:
                            N[j][i].neighbors.add(N[j][i - 1])
                            N[j][i].neighbors.add(N[j + 1][i])
                            N[j][i].neighbors.add(N[[j + 1][i - 1]])
                    else:
                        if M[j, i - 1] == 0:
                            N[j][i].neighbors.add(N[j][i - 1])
                            N[j][i].neighbors.add(N[j + 1][i])
                            N[j][i].neighbors.add(N[[j + 1][i - 1]])
                        if M[j, i] == 0:
                            N[j][i].neighbors.add(N[j][i + 1])
                            N[j][i].neighbors.add(N[j + 1][i])
                            N[j][i].neighbors.add(N[j + 1][i + 1])
                elif j == ylen:
                    if i == 0:
                        if M[j - 1, i] == 0:
                            N[j][i].neighbors.add(N[j - 1][i])
                            N[j][i].neighbors.add(N[j][i + 1])
                            N[j][i].neighbors.add(N[j - 1][i + 1])
                    elif i == xlen:
                        if M[j - 1, i - 1] == 0:
                            N[j][i].neighbors.add(N[j][i - 1])
                            N[j][i].neighbors.add(N[j - 1][i])
                            N[j][i].neighbors.add(N[j - 1][i - 1])

                    else:
                        if M[j - 1, i - 1] == 0:
                            N[j][i].neighbors.add(N[j][i - 1])
                            N[j][i].neighbors.add(N[j - 1][i])
                            N[j][i].neighbors.add(N[j - 1][i - 1])
                        if M[j - 1, i] == 0:
                            N[j][i].neighbors.add(N[j - 1][i])
                            N[j][i].neighbors.add(N[j][i + 1])
                            N[j][i].neighbors.add(N[j - 1][i + 1])
                elif i == 0:
                    if M[j - 1, i] == 0:
                        N[j][i].neighbors.add(N[j - 1][i])
                        N[j][i].neighbors.add(N[j][i + 1])
                        N[j][i].neighbors.add(N[j - 1][i + 1])
                    if M[j, i] == 0:
                        N[j][i].neighbors.add(N[j + 1][i])
                        N[j][i].neighbors.add(N[j][i + 1])
                        N[j][i].neighbors.add(N[j + 1][i + 1])
                elif i == xlen:
                    if M[j - 1, i - 1] == 0:
                        N[j][i].neighbors.add(N[j - 1][i])
                        N[j][i].neighbors.add(N[j][i - 1])
                        N[j][i].neighbors.add(N[j - 1][i - 1])
                    if M[j, i - 1] == 0:
                        N[j][i].neighbors.add(N[j + 1][i])
                        N[j][i].neighbors.add(N[j][i - 1])
                        N[j][i].neighbors.add(N[j + 1][i - 1])
                else:
                    # Bottom left cell open
                    if M[j - 1, i - 1] == 0:
                        N[j][i].neighbors.add(N[j - 1][i])
                        N[j][i].neighbors.add(N[j][i - 1])
                        N[j][i].neighbors.add(N[j - 1][i - 1])
                    # Bottom right cell open
                    if M[j - 1, i] == 0:
                        N[j][i].neighbors.add(N[j - 1][i])
                        N[j][i].neighbors.add(N[j][i + 1])
                        N[j][i].neighbors.add(N[j - 1][i + 1])
                    # Top left cell open
                    if M[j, i - 1] == 0:
                        N[j][i].neighbors.add(N[j + 1][i])
                        N[j][i].neighbors.add(N[j][i - 1])
                        N[j][i].neighbors.add(N[j + 1][i - 1])
                    # Top right cell open
                    if M[j, i] == 0:
                        N[j][i].neighbors.add(N[j + 1][i])
                        N[j][i].neighbors.add(N[j][i + 1])
                        N[j][i].neighbors.add(N[j + 1][i + 1])

        # Find start vertex OR if a vertex at that point does not exist, find closest Vertex in direction of goal & mark that
        # we are not explicitly starting at a vertex on the grid map so we can add the path from the actual start location to
        # the starting vertex on the grid map at the end, when printing the path.
        x_index_start = int((sx_start - xstart) / deltax)
        y_index_start = int((sy_start - ystart) / deltay)
        startAtVertex = True
        if N[y_index_start][x_index_start].x == sx_start and N[y_index_start][x_index_start].y == sy_start:
            startVertex = N[y_index_start][x_index_start]
        else:
            if sx_goal > sx_start:
                x_index_start = int((sx_start - xstart) / deltax + 1)
            if sy_goal > sy_start:
                y_index_start = int((sy_start - ystart) / deltay + 1)
            startVertex = N[y_index_start][x_index_start]
            startAtVertex = False

        # Do the same for the goal vertex
        x_index_goal = int((sx_goal - xstart) / deltax)
        y_index_goal = int((sy_goal - ystart) / deltay)
        goalAtVertex = True
        if N[y_index_goal][x_index_goal].x == sx_goal and N[y_index_goal][x_index_goal].y == sy_goal:
            goalVertex = N[y_index_goal][x_index_goal]
        else:
            if sx_goal < sx_start:
                x_index_goal = int((sx_goal - xstart) / deltax + 1)
            if sy_goal < sy_start:
                y_index_goal = int((sy_goal - ystart) / deltay + 1)
            goalVertex = N[y_index_goal][x_index_goal]
            goalAtVertex = False

        map_time = time.time() - start_time
        print "Pre-Processing & Map Generating Time Taken:", map_time, "sec"

        # A* ALGORITHM
        print "Starting A*"
        # add A* heuristics data to vertices
        for j in range(0, ylen + 1):
            for i in range(0, xlen + 1):
                N[j][i].h = aStarHeuristic(N[j][i].x, N[j][i].y, goalVertex.x, goalVertex.y)

        def UpdateVertexA_star(s, s_prime, fringe):
            if (s.g + c(s, s_prime) < s_prime.g):
                s_prime.g = s.g + c(s, s_prime)
                s_prime.p = s
                s_prime_tuple = [(val, key) for (val, key) in fringe if key == s_prime]
                if len(s_prime_tuple) == 1:
                    fringe.remove(s_prime_tuple[0])
                    heapq.heapify(fringe)
                heapq.heappush(fringe, (s_prime.g + s_prime.h, s_prime))

        start_time = time.time()

        ##A* Main
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


        # Backtrack from goal to start vertex by using parent attribute of each vertex to get the solution path vertices
        n = goalVertex
        pathAstar = [(n.x,n.y)]
        while n != startVertex:
            n = n.p
            pathAstar.append((n.x,n.y))

        # If start vertex was not an actual vertex on the map, add it
        if startAtVertex == False:
            pathAstar.append((sx_start, sy_start))

        # Flip the path array so it starts at the start and ends at the goal
        path = []
        pathLen = len(pathAstar)
        for i in range(0, pathLen):
            path.append(pathAstar[pathLen - 1 - i])

        # If goal vertex was not an actual vertex on the map, add it
        if goalAtVertex == False:
            path.append((sx_goal, sy_goal))

        # Round path coordinates to round_digits
        round_digits = 3
        for i in range(0, len(path)):
            path[i] = list(path[i])
            path[i][0] = round(path[i][0], round_digits)
            path[i][1] = round(path[i][1], round_digits)
            path[i] = tuple(path[i])
        # print path

        # Graphing Map
        # Make x and y coordinates vectors for matlibplot input from the path vertex tuples
        x_coors = []
        y_coors = []
        for i in range(0, len(path)):
            x_coors.append(list(path[i])[0])
            y_coors.append(list(path[i])[1])

        x_box = []
        y_box = []
        for y in range(0, ylen):
            for x in range(0, xlen):
                if M[y][x] != 0:
                    x_box.append(x * deltax + xstart + deltax / 2)
                    y_box.append(y * deltay + ystart + deltay / 2)

        # Red dots are center points of blocked cells
        # Blue dots are path vertices
        plt.ylim((ystart, yend))   # set the ylim to bottom, top
        plt.xlim((xstart, xend))   # set the xlim to bottom, top
        plt.plot(open_xcoors,open_ycoors, 'o', color = "#FFDAB9")
        plt.plot(closed_xcoors,closed_ycoors, 'o', color = "#FFFF00")
        plt.plot(x_box,y_box, 'ro')
        plt.plot(x_coors,y_coors, 'bo-')
        directory = "A*_results"
        if not os.path.exists(directory):
            os.makedirs(directory)
        fname = "A*_results/path" +str(start_goal_pair_number) +".png"
        resultFile = directory + "/result" + str(start_goal_pair_number) + ".txt"
        plt.savefig(fname)
        plt.close()

        print "Path coordinates"
        the_file = open(resultFile, "w+")
        the_file.truncate(0)
        for Astarpath in path:
            resultpath = str(Astarpath[0]) + " " + str(Astarpath[1])
            print resultpath
            the_file.write(resultpath+"\n")
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
        fp.write("A* Total Expanded Nodes: " + str(AstarExpandednodes) + "\n")
        fp.write("A* Total Time: " + str(totalTime) + "\n")
        fp.write("A* Total Path Distance: " + str(totalDist) + "\n")
        fp.write("\n\n\n")
        fp.close()