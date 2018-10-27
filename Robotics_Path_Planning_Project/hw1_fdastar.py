import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import os
import time

# Take input map file and output vector of walls, polygons, and start/goal locations
def parseFile(filename):

    fp = open(filename, "r")
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


for mapNum in range(5,6):
	for start_goal_pair_number in range(5,6):
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
				N[j].append(Vertex(x=i * deltax + xstart, y=j * deltay + ystart, p=None, g=0, neighbors=set()))

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

		# FDA* ALGORITHM
		print "Starting FDA*"
		# add FDA* heuristics data to vertices
		for j in range(0, ylen + 1):
			for i in range(0, xlen + 1):
				N[j][i].h = fdaStarHeuristic(N[j][i].x, N[j][i].y, goalVertex.x, goalVertex.y)

		def grid(x, y):
			x_index = int(x)
			y_index = int(y)
			if len(N[y_index][x_index].neighbors) < 6:
				return True
			return False

		def LineOfSight(s, s_prime_inner):
			x0 = int(float(s.x - xstart) / deltax)
			y0 = int(float(s.y - ystart) / deltay)
			x1 = int(float(s_prime_inner.x - xstart) / deltax)
			y1 = int(float(s_prime_inner.y - ystart) / deltay)
			f = 0
			dy = y1 - y0
			dx = x1 - x0
			if dy < 0:
				dy = -dy
				sy = -1
			else:
				sy = 1
			if dx < 0:
				dx = -dx
				sx = -1
			else:
				sx = 1
			if dx >= dy:
				while x0 != x1:
					f = f + dy
					if f >= dx:
						if grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
							return False
						y0 += sy
						f = f - dx
					if f != 0 and grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
						return False
					if dy == 0 and grid(x0 + ((sx - 1) / 2), y0) and grid(x0 + ((sx - 1) / 2), y0 - 1):
						return False
					x0 += sx

					# Round indices so equalities are not affected by floating point arithmetic/ truncation error
					x0 = round(x0, 3)
					x1 = round(x1, 3)
					y0 = round(y0, 3)

			else:
				while y0 != y1:
					f = f + dx
					if f >= dy:
						if grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
							return False
						x0 += sx
						f -= dy
					if f != 0 and grid(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)):
						return False
					if dx == 0 and grid(x0, y0 + ((sy - 1)/2)) and grid(x0 - 1, y0 + ((sy - 1)/2)):
						return False
					y0 += sy

					# Round indices so equalities are not affected by floating point arithmetic/ truncation error
					y0 = round(y0, 3)
					y1 = round(y1, 3)
					x0 = round(x0, 3)
			return True

		def UpdateVertexFDA_star(s, s_prime, openList):
			if LineOfSight(s.p, s_prime):
				# Path 2
				if (s.p.g + c(s.p, s_prime)) < s_prime.g:
					s_prime.g = s.p.g + c(s.p, s_prime)
					s_prime.p = s.p
					s_prime_tuple_fda = [(val, key) for (val, key) in openList if key == s_prime]
					if len(s_prime_tuple_fda) == 1:
						openList.remove(s_prime_tuple_fda[0])
						heapq.heapify(openList)
					heapq.heappush(openList, (s_prime.g + s_prime.h, s_prime))
			else:
				# Path 1
				if s.g + c(s, s_prime) < s_prime.g:
					s_prime.g = s.g + c(s, s_prime)
					s_prime.p = s
					s_prime_tuple_fda = [(val, key) for (val, key) in openList if key == s_prime]
					if len(s_prime_tuple_fda) == 1:
						openList.remove(s_prime_tuple_fda[0])
						heapq.heapify(openList)
					heapq.heappush(openList, (s_prime.g + s_prime.h, s_prime))

		start_time = time.time()

		# FDA* Main
		expandedNodes = set()
		startVertex.g = 0
		startVertex.p = startVertex
		openList = []
		heapq.heapify(openList)
		heapq.heappush(openList, (startVertex.g + startVertex.h, startVertex))
		closedFDA = set()

		while len(openList) != 0:
			s = list(heapq.heappop(openList))[1]
			if s == goalVertex:
				print "***path found***"
				break
			closedFDA.add(s)
			for s_prime in s.neighbors:
				if s_prime not in closedFDA:
					s_prime_tuple = [(val, key) for (val, key) in openList if key == s_prime]
					if len(s_prime_tuple) != 1:
						s_prime.g = float('inf')
						s_prime.p = None
					UpdateVertexFDA_star(s, s_prime, openList)


		# Print time taken
		fda_star_time = time.time() - start_time
		print "FDA* time taken:", fda_star_time, "sec"

		closed_xcoors = []
		closed_ycoors = []

		open_xcoors = []
		open_ycoors = []
		for i in closedFDA:
			closed_xcoors.append(i.x)
			closed_ycoors.append(i.y)

		for i in openList:
			open_xcoors.append(i[1].x)
			open_ycoors.append(i[1].y)

		FDAExpandednodes = len(open_xcoors)
		print "FDA* number of expanded nodes: ", FDAExpandednodes

		# Backtrack from goal to start vertex by using parent attribute of each vertex to get the solution path vertices
		n = goalVertex
		path = [(n.x,n.y)]
		while n != startVertex:
			n = n.p
			path.append((n.x,n.y))

		# If start vertex was not an actual vertex on the map, add it
		if startAtVertex == False:
			path.append((sx_start, sy_start))

		# Flip the path array so it starts at the start and ends at the goal
		pathFDA = []
		pathLen = len(path)
		for i in range(0, pathLen):
			pathFDA.append(path[pathLen-1-i])

		# If goal vertex was not an actual vertex on the map, add it
		if goalAtVertex == False:
			pathFDA.append((sx_goal, sy_goal))

		# Round path coordinates to round_digits
		round_digits = 3
		for i in range(0, len(pathFDA)):
			pathFDA[i] = list(pathFDA[i])
			pathFDA[i][0] = round(pathFDA[i][0], round_digits)
			pathFDA[i][1] = round(pathFDA[i][1], round_digits)
			pathFDA[i] = tuple(pathFDA[i])

		# Graphing Map
		# Make x and y coordinates vectors for matlibplot input from the path vertex tuples
		x_coors = []
		y_coors = []
		for i in range(0, len(pathFDA)):
			x_coors.append(list(pathFDA[i])[0])
			y_coors.append(list(pathFDA[i])[1])

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

		directory = "FDA*_results"
		if not os.path.exists(directory):
			os.makedirs(directory)
		fname = "FDA*_results/path" +str(start_goal_pair_number) +".png"
		plt.savefig(fname)
		plt.close()

		print "Path coordinates"

		for indexk in range(0, len(x_coors)):
			print(str(x_coors[indexk])+" "+ str(y_coors[indexk]))

		totalTime = fda_star_time+map_time
		print "Total time taken:", totalTime, "sec"

		# Open results file
		resultFile = directory + "/result" + str(start_goal_pair_number) + ".txt"

		fp = open(resultFile, 'a')
		fp.truncate(0)
		for indexk in range(0, len(x_coors)):
			resultpath = str(x_coors[indexk])+" "+ str(y_coors[indexk]) + "\n"
			fp.write(resultpath)
		fp.close()


		# Getting path distance
		# Distance Function (Straight Line Distance)
		def dist(a,b,c,d):
			return (((a - c)**2 + (b - d)**2)**(0.5))
		totalDist = 0;
		for i in range(0, len(x_coors)-1):
			totalDist += dist(x_coors[i], y_coors[i], x_coors[i+1], y_coors[i+1])
		print "Total distance of path:", totalDist, "meters"


		fp = open(str(directory)+"/dataFile.txt", 'a')
		fp.write("Map "+str(mapNum)+", Start/Goal "+ str(start_goal_pair_number)+"\n")
		fp.write("FDA* Total Expanded Nodes: "+str(FDAExpandednodes)+"\n")
		fp.write("FDA* Total Time: "+str(totalTime)+"\n")
		fp.write("FDA* Total Path Distance: "+str(totalDist)+"\n")
		fp.write("\n\n\n")
		fp.close()
