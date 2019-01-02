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
import ast

N_arr = [10, 25, 50, 100]	# Number of particles
noises = ['low', 'mid', 'high']	# Different noise settings array
worlds = [1,2,3,4,5,6,7]		# Different worlds array


if __name__ == '__main__':

	with open('accuracy/results.txt', 'r') as f:
		world_accuracy = ast.literal_eval(f.read())
		
	colors = ['ko-','bo-','ro-']
	for i in range(0,len(worlds)):
	
		for j in range(0, len(noises)):
		
			plt.plot(N_arr, world_accuracy[i][j], colors[j], label=noises[j])
		
		plt.title("World "+str(worlds[i]))
		plt.xlabel('N (number of particles)')
		plt.ylabel('Translational + Rotational Distance Error (m)')
        plt.grid(color='k', linestyle='-', linewidth=1)
        plt.legend()
        plt.savefig("accuracy/world"+str(worlds[i])+"_accuracy.png")
        plt.close()
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
