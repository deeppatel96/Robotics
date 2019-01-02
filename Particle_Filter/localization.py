#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import random
import math
import tf
from geometry_msgs.msg import Pose
import rospy


# Global Parameters
N_arr = [10, 25, 50, 100]		# Number of particles
noises = ['low', 'mid', 'high']	# Different noise settings array
worlds = [1,2,3,4,5,6,7]		# Different worlds array
a1 = 0.05						# Motion Model parameter 
a2 = 0.05						# Motion Model parameter 
a3 = 0.05						# Motion Model parameter 
a4 = 0.05						# Motion Model parameter 
line_length = 0.3	 			# Plot orientation line length
_map = None						# Map walls and obstacles
line_segments = None			# Line Segments of Map

zmax = 10
p_rand = 1/zmax
p_hit = 1-p_rand
var = 0.1

# Map Class Definition
class Map():
    def __init__(self, walls=None, obstacles=None):
        self.walls = walls
        self.obstacles = obstacles

# Trajectory Class Definition
class Trajectory():
    def __init__(self, start_x=None, start_y=None, heading=None, distance=None, ground_truth=None, \
    			noisy_heading=None, noisy_distance=None, scan_data=None):
    	self.start_x = start_x
    	self.start_y = start_y
    	self.heading = heading
    	self.distance = distance
    	self.ground_truth = ground_truth
    	self.noisy_heading = noisy_heading
    	self.noisy_distance = noisy_distance
    	self.scan_data = scan_data
        
# Particle Class Definition
class Particle():
    def __init__(self, state=None, weight=None):
        self.state = state
        self.weight = weight

 # State Class Definition
class State():
    def __init__(self, x=None, y=None, theta=None):
        self.x = x
        self.y = y
        self.theta = theta

# Take input map file and output vector of walls, polygons, and start/goal locations
def parseMapFile(fileName):
	
	global _map
	_map = Map()

	fp = open(fileName)
	line = fp.readline()
	line_vector = line.split()
	for i in range(0, len(line_vector)):
		line_vector[i] = eval(line_vector[i])
	_map.walls = line_vector
	_map.obstacles = []
	line = fp.readline()
	line = fp.readline()
	while line != '':
		line_vector = line.split()
		for i in range(0, len(line_vector)):
			line_vector[i] = eval(line_vector[i])
		_map.obstacles.append(line_vector)
		line = fp.readline()
 
	fp.close()


# Take input trajectory file and output 
def parseTrajectoryFile(fileName, isGroundTruth=True):

	_trajectory = Trajectory()
	
	fp = open(fileName)
	
	# Get starting position
	line = fp.readline()
	line = fp.readline()
	line_vector = line.split()
	_trajectory.start_x = eval(line_vector[1])
	line = fp.readline()
	line_vector = line.split()
	_trajectory.start_y = eval(line_vector[1])
	
	line = fp.readline()
	line = fp.readline()
	# Initialize lists for trajectory object
	_trajectory.heading = []
	_trajectory.distance = []
	if isGroundTruth:
		_trajectory.ground_truth = []
	_trajectory.noisy_heading = []
	_trajectory.noisy_distance = []
	_trajectory.scan_data = []
	
	# Loop through all iterations of actions
	while line != '':
		
		# Get heading and distance data
		line_vector = line.split()
		_trajectory.heading.append(eval(line_vector[2]))
		line = fp.readline()
		line_vector = line.split()
		_trajectory.distance.append(eval(line_vector[2]))
		
		# Get ground truth data
		if isGroundTruth:

			pose = Pose()
			line = fp.readline()
			line = fp.readline()
			line = fp.readline()
			line = fp.readline()
			line = fp.readline()
			line_vector = line.split()
			pose.position.x = eval(line_vector[1])
			line = fp.readline()
			line_vector = line.split()
			pose.position.y = eval(line_vector[1])
			line = fp.readline()
			line_vector = line.split()
			pose.position.z = eval(line_vector[1])
			line = fp.readline()
			line = fp.readline()
			line_vector = line.split()
			pose.orientation.x = eval(line_vector[1])
			line = fp.readline()
			line_vector = line.split()
			pose.orientation.y = eval(line_vector[1])
			line = fp.readline()
			line_vector = line.split()
			pose.orientation.z = eval(line_vector[1])
			line = fp.readline()
			line_vector = line.split()
			pose.orientation.w = eval(line_vector[1])
			_trajectory.ground_truth.append(pose)
			
			# Skip twist information in ground truth data
			for _ in range(0,10):
				line = fp.readline()
		
		# Get Noisy Heading and Distance Data
		line = fp.readline()
		line = fp.readline()
		line_vector = line.split()
		_trajectory.noisy_heading.append(eval(line_vector[1]))
		line = fp.readline()
		line = fp.readline()
		line_vector = line.split()
		_trajectory.noisy_distance.append(eval(line_vector[1]))
		
		# Get scan data
		line = fp.readline()
		line = fp.readline()
		line_vector = line.split()[1:]
		
		scan_data_i = []
		for i in range(0, len(line_vector)):
			if 'nan' in line_vector[i]:
				scan_data_i.append(None)
			else:
				scan_data_i.append(eval(line_vector[i].translate(None,',()[]')))
		_trajectory.scan_data.append(scan_data_i)
		
		# Go to get line (will be '' at EOF)
		line = fp.readline()
 
	fp.close()
	return _trajectory


def get_map_line_segments():

	global line_segments
	line_segments = []
	# Get wall line segments
	for i in range(0,len(_map.walls)-1):
		line_segments.append([list(_map.walls[i]), list(_map.walls[i+1])])
	line_segments.append([list(_map.walls[-1]), list(_map.walls[0])])
	
	# Get obstacle line segments
	for i in range(0,len(_map.obstacles)):
		for j in range(0,len(_map.obstacles[i])-1):
			line_segments.append([list(_map.obstacles[i][j]), list(_map.obstacles[i][j+1])])
		line_segments.append([list(_map.obstacles[i][-1]), list(_map.obstacles[i][0])])
		
	return np.array(line_segments)


# Particle Filter Algorithm Definition
# @params	
# St_1 - set of particles at time t-1
# ut - control input at time t
# zt - observation at time t
def particle_filter_SIR(St_1, ut, zt):
	N = len(St_1)

	# Initialize new particle set
	St = []
	
	# Initialize normalization factor
	eta = 0
	
	for i in range(0,N):
	
		# Initialize new particle
		p = Particle()
	
		# Generate sample indices based on previous weights
		particle_index = generate_index(St_1)
		
		# Sample new particle state using sample motion model
		p.state = sample_motion_model(St_1[particle_index].state, ut)
		
		# Compute importance weight
		p.weight = observation_model(zt, p.state)
		
		# Update Normalization Factor
		eta += p.weight
		
		# Insert new particle with state and weight into particle set
		St.append(p)
		
	# Normalize weights of new particles
	if eta == 0:
		for i in range(0,N):
			St[i].weight = 1/N
	else:
		for i in range(0,N):
			St[i].weight = St[i].weight / eta
	
	return St


# Generates index for particle using discrete weight distribution of the particle set
def generate_index(St_1):
	N = len(St_1)
	r = random.random()
	curr = 0
	j = 0
	while curr < r and j<N:
		curr += St_1[j].weight
		j += 1
	return j-1
	
	
# Sample Motion Model
def sample_motion_model(xt_1, ut):
	xt = State()
	
	delta_rot1 = ut[0] + sample_normal_distribution(a1*abs(ut[0]) + a2*ut[1])
	delta_trans = ut[1] + sample_normal_distribution(a3*ut[1] + a4*abs(ut[0]))
	delta_rot2 = 0 + sample_normal_distribution(a2*ut[1])

	xt.x = xt_1.x + delta_trans*math.cos(delta_rot1)
	xt.y = xt_1.y + delta_trans*math.sin(delta_rot1)
	xt.theta = delta_rot1 + delta_rot2
	
	return xt
	
	
# Observation Model
def observation_model(zt, xt):
	
	num_points = len(zt)
	delta_theta = np.deg2rad(60)/ num_points
	q = 1
	for i in range(0, num_points):
		theta = xt.theta + (-num_points/2 + i)*delta_theta
		
		# Ignoring nan for now to see performance without considering
		if zt[i] != None:
			point = [xt.x+zt[i]*math.cos(theta), xt.y+zt[i]*math.sin(theta)]
			d = closest_distance(point)
			prob = (1/(((2*math.pi)**0.5)*var))*math.exp(-(d**2)/(2*(var**2)))
			q = q * (p_hit*prob + p_rand*1/zmax)

	return q
	
# Get closest distance to an obstacle in the map from a point
def closest_distance(point):
	
	distances = []
	for line in line_segments:
		distances.append(point_to_line_dist(point, line))
	
	return min(distances)
	

# Calculates Distance Between a Point and a Line Segment
def point_to_line_dist(point, line):
    """Calculate the distance between a point and a line segment.

    To calculate the closest distance to a line segment, we first need to check
    if the point projects onto the line segment.  If it does, then we calculate
    the orthogonal distance from the point to the line.
    If the point does not project to the line segment, we calculate the 
    distance to both endpoints and take the shortest distance.

    :param point: Numpy array of form [x,y], describing the point.
    :type point: numpy.core.multiarray.ndarray
    :param line: list of endpoint arrays of form [P1, P2]
    :type line: list of numpy.core.multiarray.ndarray
    :return: The minimum distance to a point.
    :rtype: float
    """
    line = np.array(line)
    # unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) + 
        (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
    if is_betw_x and is_betw_y:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist
	

# Sample Normal Distribution Function Definition
def sample_normal_distribution(b):
	
	sum = 0
	for i in range(0,12):
		sum += np.random.uniform(-b,b)
	
	return sum/2
	
	
# Get weighted mean of particle state (x,y,theta)
def mean_particle_state(St):
	N = len(St)
	mean_state = State()
	mean_state.x = 0.0
	mean_state.y = 0.0
	mean_state.theta = 0.0
	for p in range(0, N):
		mean_state.x += float(St[p].state.x)
		mean_state.y += float(St[p].state.y)
		mean_state.theta += float(St[p].state.theta)
	mean_state.x = mean_state.x / N	
	mean_state.y = mean_state.y / N
	mean_state.theta = mean_state.theta / N
	
	return mean_state
	
	
# Quaternion to Euler Angle & Extract Yaw as 2D rotation Angle Theta
def quaternionToTheta(x,y,z,w):
	euler = tf.transformations.euler_from_quaternion((x,y,z,w))
	return euler[2]


# Plot map
def plot_map(world, noise, N):
	x = [_map.walls[0][0], _map.walls[1][0], _map.walls[2][0], _map.walls[3][0]]
	y = [_map.walls[0][1], _map.walls[1][1], _map.walls[2][1], _map.walls[3][1]]
	x_min, x_max, y_min, y_max = min(x), max(x), min(y), max(y)
	for line in line_segments:
		plt.plot([line[0][0], line[1][0]],[line[0][1], line[1][1]], 'k-',LineWidth = 2)
	plt.xlim(x_min,x_max)
	plt.ylim(y_min,y_max)
	plt.title(noise+" noise in world "+str(world)+" with "+str(N)+" particles")
#	plt.show(block=False)

# Plot iterations of particle filter algorithm
def plot_iteration(St, mean_state, ground_truth):
	N = len(St)
	# Plot particles
	x,y = [],[]
	for i in range(0,N):
		x.append(St[i].state.x)
		y.append(St[i].state.y)
	plt.plot(x, y,'r.', MarkerSize=3)
	
	# Plot Mean Particle State
	plt.plot(mean_state.x, mean_state.y, 'r.', MarkerSize=10)
	dx_pm = [mean_state.x, mean_state.x + line_length*math.cos(mean_state.theta)]
	dy_pm = [mean_state.y, mean_state.y + line_length*math.sin(mean_state.theta)]
	plt.plot(dx_pm, dy_pm, 'r-', LineWidth=3)
	
	# Plot Ground Truth State
	plt.plot(ground_truth.position.x, ground_truth.position.y, 'b.', MarkerSize=8)
	
	theta_gt = quaternionToTheta(ground_truth.orientation.x,\
												ground_truth.orientation.y,\
												ground_truth.orientation.z,\
												ground_truth.orientation.w)
	dx_gt = [ground_truth.position.x, ground_truth.position.x + line_length*math.cos(theta_gt)]
	dy_gt = [ground_truth.position.y, ground_truth.position.y + line_length*math.sin(theta_gt)]
	plt.plot(dx_gt, dy_gt, 'b-', LineWidth=2)
	
	# Set options for plot
#	plt.show(block=False)
	
	
# Get Translational & Rotational Distance where state one is a State object and state2 is a Pose object
def distance(state1, state2):
	x = state2.position.x
	y = state2.position.y
#	qx2 = state2.orientation.x
#	qy2 = state2.orientation.y
#	qz2 = state2.orientation.z
#	qw2 = state2.orientation.w
#	theta2 = quaternionToTheta(qx2,qy2,qz2,qw2)
#	d = (((state1.x - x)**2) + ((state1.y - y)**2) + ((state1.theta - theta2)**2))**0.5
	d = (((state1.x - x)**2) + ((state1.y - y)**2))**0.5
	return d
    
    
# Get accuracy between mean particle state and ground truth state
def get_accuracy(mean_state, ground_truth):
	return distance(mean_state, ground_truth)


if __name__ == '__main__':

	# Accuracy for different worlds, noise levels, particle numbers
	world_accuracy = []

	# Iterate through all worlds
	for world in worlds:
		print "load world", world, "map"
	
		# Accuracy for different noise levels, particle numbers
		noise_accuracy = []
	
		# get map file descripor for corresponding world
		map_fn = "../../../turtlebot_maps/map_" + str(world) + ".txt"

		# Parse map file & get line segments -- saved to global variables
		parseMapFile(map_fn)
		get_map_line_segments()
		
		# Iterate through all noise amounts
		for noise in noises:
			print "world", world, "| noise", noise
		
			# Accuracy for different particle numbers
			N_accuracy = []

			# get trajectory file descriptor for corresponding world and noise
			trajectory_fn = "../../../turtlebot_maps/trajectories/"+ noise+ "_world"+str(world)+".txt"

			# Parse trajectory file
			_trajectory = parseTrajectoryFile(trajectory_fn)
	
			# Iterate over different numbers of particles
			for N in N_arr:
				print "world", world, "| noise", noise, "| Particle-Size", N
			
				# Initialize particle number iteration accuracy array
				accuracy = []
				
				# Create initial particles
				St_1 = []
				w = 1.0/float(N)
				for i in range(0, N):
					p = Particle()
					p.state = State()
					p.state.x = _trajectory.start_x
					p.state.y = _trajectory.start_y
					p.state.theta = 0
					p.weight = w
					St_1.append(p)
		
				# Plot map with obstacles
				plot_map(world, noise, N)
		
				# Plot Initial Graph
				mean_state = mean_particle_state(St_1)
				initial_pose = Pose()
				initial_pose.position.x, initial_pose.position.y, initial_pose.position.z = _trajectory.start_x, _trajectory.start_y, 0
				initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w = 0,0,0,1
				plot_iteration(St_1, mean_state, initial_pose)
	
				# Begin Particle Filter Algorithm
				n = len(_trajectory.heading)
				for i in range(0, n):
					print "world", world, "| noise", noise, "| Particle-Size", N, "| Trajectory Point",i,"/",n
					ut = [_trajectory.heading[i], _trajectory.distance[i]]
					zt = _trajectory.scan_data[i]
					St = particle_filter_SIR(St_1, ut, zt)
					mean_state = mean_particle_state(St)
					plot_iteration(St, mean_state, _trajectory.ground_truth[i])
					accuracy.append(get_accuracy(mean_state, _trajectory.ground_truth[i]))
					St_1 = St
	
				# To save the plot
				directory = "graphs"
				if not os.path.exists(directory):
					os.makedirs(directory) 
				plt.savefig(directory+"/"+noise+"_world"+str(world)+"_"+str(N)+"_particle.png")
				plt.show(block=False)
				plt.close()
				
				# Append accuracy of current particle size
				mean_accurary = np.mean(accuracy)
				print mean_accurary
				N_accuracy.append(mean_accurary)
				
			# Append accuracy nested array of current noise level
			noise_accuracy.append(N_accuracy)
		
		# Append accuracy double nested array of current world
		world_accuracy.append(noise_accuracy)
				

	directory = "accuracy"
	if not os.path.exists(directory):
		os.makedirs(directory)   
	with open('accuracy/results.txt', 'w') as f:
		f.write(str(world_accuracy))
		




