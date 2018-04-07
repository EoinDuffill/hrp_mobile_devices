#!/usr/bin/env python
import rospy
import numpy as np
import sys, os, tty, select
import math, random
import matplotlib.pyplot as plt
import Image
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from am_driver.msg import WheelEncoder
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

class automower_tracking(object):

	
	def init(self):
		#Update rate in Hz
		self.update_rate = 1
		self.update_display_rate = 2
		self.clock_internal = 0
		#State variables set by the experiment/status topic, influences the experiment_running state var
		self.experiment_shutting_down = False
		self.experiment_starting_up = False
		#State variable about experiment current status
		self.experiment_running = False
		self.experiment_run_time = 10
		#Distance between the centers of the rear wheels
		self.wheel_separation_distance = 0.48
		#Setup experiment data publisher
 	   	self.pub_experiment_data = rospy.Publisher('experiment/data', Float32MultiArray, queue_size=1)

		self.file_path = '/home/eoin/ROS/catkin_ws/src/figure'
		
		#Initialise an initial experiment
		self.init_experiment()
		
	def init_experiment(self):
		self.file = open("results", "a")
		
		#initial angle, converted from a quarternion
		self.orientation = 0
		
		#initial x,y
		self.x = 0
		self.y = 0

		print("Starting Experiment...")
		#Write to file with starting co-ords
		self.file.write("\nStart: x = " + str(self.x) + "y = " + str(self.y) + "theta = " + str(self.orientation) + "\n")

		self.wheel_l_accum = 0
		self.wheel_r_accum = 0
		self.prev_tick = None
		#Start time in seconds from simulator
		self.start_time = 0
		#Time since experiment start
		self.time = 0

		#List of points from odometry
		self.x_odom_points = []
		self.y_odom_points = []

		self.experiment_running = True

	def wheel_encoder(self, data):

		if not(self.prev_tick == None):
			#Accumulate encoder data 
			self.wheel_l_accum += (data.header.seq - self.prev_tick)*data.lwheel
			self.wheel_r_accum += (data.header.seq - self.prev_tick)*data.rwheel
			#Update time variable
			self.time = (data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)) - self.start_time
			
		else:
			#Record start time, this is only called on the first iteration
			self.start_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

		#Record previous tick
		self.prev_tick = data.header.seq
	
	#Formula for calculating orientation change over time with wheel encoder data
	def orientation_change(self, e_left, e_right, orientation_start):
		#if encoder values are similar enough ignore the difference
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			return ((e_right-e_left)/self.wheel_separation_distance) + orientation_start
		else:	
			return orientation_start

	#Formula for calculating position change in x,y over time
	def position_change(self, e_left, e_right, x_start, y_start, orientation_start):
		#again ignoring the difference in wheel encoders if similar enough
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			factor = (self.wheel_separation_distance * (e_right + e_left))/(2 * (e_right - e_left))
			trigonometric_input = ((e_right - e_left)/self.wheel_separation_distance) + orientation_start
			X = x_start + (factor * (math.sin(trigonometric_input) - math.sin(orientation_start)))
			Y = y_start - (factor * (math.cos(trigonometric_input) - math.cos(orientation_start)))

			return X, Y
		else:	
			return x_start,y_start

	#function to determine if two numbers are sufficiently close to each other
	def isclose(self, a, b, rel_tol, abs_tol):
		return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

	def publish_experiment_data(self, state):
		#Publish current experiment data for mobile device to subscribe to. 
		mat = Float32MultiArray()
		mat.layout.dim.append(MultiArrayDimension())
		mat.layout.dim[0].label = "data"
		mat.layout.dim[0].size = 7
		mat.layout.dim[0].stride = 1
		mat.layout.data_offset = 0
		mat.data = [0]*7
		#experiement state, 0 = stopped
		mat.data[0] = state
		#time elapsed
		if(state == 0):
			mat.data[1] = self.experiment_run_time
		else:
			mat.data[1] = self.time
		#experiment run time
		mat.data[2] = self.experiment_run_time
		#X, Y and orientation data
		mat.data[3] = self.x
		mat.data[4] = self.y
		mat.data[5] = self.orientation
		#Error value
		mat.data[6] = 0
		self.pub_experiment_data.publish(mat)

	def run(self):
		try:
			self.init()
			
			#Setup subscribers
			rospy.Subscriber("wheel_encoder", WheelEncoder, self.wheel_encoder, queue_size=None)
			rospy.Subscriber('experiment/status', Float32, self.update_status)
			while not rospy.is_shutdown():
				#When experiment is running...
				if(self.experiment_running):
					#If command recieved to shutdown from /experiment/status finish experiment
					if(self.experiment_shutting_down):
						self.finish_experiment()
					#Continue running experiment if time allows
					elif(self.time < self.experiment_run_time):
						self.update()
					#Else the experiment has finished
					else:
						#Reset clock so last update calculates a finishing position and orientation
						self.clock_internal = 0
						self.update()
						self.finish_experiment()
						self.idle()
				#While experiment is fininished wait or start command
				else:
					if(self.experiment_starting_up):
						self.init_experiment()
					else:
						self.idle()

				
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()

	def update(self):
		#write data (odom calculations etc) based on display rate and clock
		if (self.clock_internal % (self.update_display_rate) == 0):
			self.write_data()
			self.publish_experiment_data(1.0)
		
		#Increment internal clock
		self.clock_internal += 1

	def write_data(self):
		#Get initial variables from simulator
		data = rospy.wait_for_message("gazebo/model_states", ModelStates)
		#get the index into the data for the automower
		index = data.name.index("automower")
		
		#Calculate position change, then orientation
		(self.x, self.y) = self.position_change(self.wheel_l_accum, self.wheel_r_accum, self.x, self.y, self.orientation)
		self.orientation = self.orientation_change(self.wheel_l_accum, self.wheel_r_accum, self.orientation)
		self.orientation = ((self.orientation + math.pi) % (2*math.pi)) - math.pi
			
		#print(self.time, self.orientation, self.x, self.y)
		self.file.write("Time = " + str(self.time) + "\n")
		self.file.write("Odometry: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")
		
		#Record x and y, and x and y predictions from odometry
		self.x_odom_points.append(self.x)
		self.y_odom_points.append(self.y)

		#Resets accumalative encoders
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0

	def update_status(self, data):
		#Callback function from subscribed topic about experiment status
		#start new experiment with data.data length
		if(data.data > 0):
			if not(self.experiment_running):
				self.experiment_run_time = data.data
				self.experiment_starting_up = True
		#Stop current experiment	
		if(data.data < 0):
			if(self.experiment_running):
				self.experiment_shutting_down = True
		
		#Reset vars, as experiment has either started up or shut down at this point	
		if(self.experiment_running):
			self.experiment_starting_up = False
		else:
			self.experiment_shutting_down = False

	def finish_experiment(self):
		if(self.experiment_running):
			self.file.close()

			print("Finishing Experiment...")
			#Clear points and axis
			plt.cla()
			plt.clf()
			#Plot odometry calculated points
			plt.plot(self.x_odom_points, self.y_odom_points, 'b--')

			#Set axis based on min and max vals in x and y
			plt.axis([np.amin(self.x_odom_points) - 1, np.amax(self.x_odom_points) + 1, np.amin(self.y_odom_points) - 1, np.amax(self.y_odom_points) + 1])
			plt.savefig(self.file_path+'.png')
			#Save the png as a JPEG
			Image.open(self.file_path+'.png').save(self.file_path+'.jpg','JPEG')
			
			#Running var set to false to signal the end of the experiment			
			self.experiment_running = False
		

	def idle(self):
		#publish experiment data to topic, in this case the previous experiment's data
		self.publish_experiment_data(0.0)


	def fini(self):
		print('Finishing...')

if __name__ == '__main__':
	rospy.init_node('automower_tracking')
	tracker = automower_tracking()
	tracker.run()
