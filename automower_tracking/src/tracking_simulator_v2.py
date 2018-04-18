#!/usr/bin/env python
import rospy
import numpy as np
import sys, os, tty, select
import math, random
import matplotlib.pyplot as plt
import Image
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from am_driver.msg import WheelEncoder
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

class automower_tracking_simulator(object):

	def init(self):
		#Update rate in Hz
		self.update_rate = 50
		#Update figure in hz
		self.figure_update_rate = 1
		self.counter = 0
		#Experiment counter, used to differentiate experiments
		self.experiment_number = 1
		#Experiment type
		self.experiment_type = 1
		#State variables set by the experiment/status topic, influences the experiment_running state var
		self.experiment_shutting_down = False
		self.experiment_starting_up = False
		#State variable about experiment current status
		self.experiment_running = False
		#Distance between the centers of the rear wheels
		self.wheel_separation_distance = 0.48
		#Setup experiment data publisher
 	   	self.pub_experiment_data = rospy.Publisher('experiment/data', Float32MultiArray, queue_size=1)
		#Publisher to navigate automower based on odometry (error calculation experiment)
		self.pub_odom_navigation = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		#init vars
		self.time = 0
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0
		self.prev_tick = None
		self.x = 0
		self.y = 0
		self.orientation = 0

		#Odometry error calculation variable init
		self.experiment_side_length = 3.5
		self.automower_vel = 0.3
		self.automower_rot_vel = 0.3
		self.stage = 0
		self.stage_change = True
		self.stage_init_x = 0
		self.stage_init_y = 0
		self.stage_init_ori = 0
		self.twist = Twist()

		self.file_path = '/home/eoin/ROS/catkin_ws/src/figure'
		
	def init_experiment(self):
		self.file = open("results.txt", "a")

		#Get initial variables from simulator
		data = rospy.wait_for_message("gazebo/model_states", ModelStates)
		#get the index into the data for the automower
		index = data.name.index("automower")
		
		#initial angle, converted from a quarternion
		self.orientation = self.quarternion_to_angle(data.pose[index].orientation.x, data.pose[index].orientation.y,data.pose[index].orientation.z,data.pose[index].orientation.w)
		
		#initial x,y
		self.x = data.pose[index].position.x
		self.gps_x = data.pose[index].position.x
		self.y = data.pose[index].position.y
		self.gps_y = data.pose[index].position.y

		print("Starting Experiment " + str(self.experiment_number)+" of Type: "+ str(self.experiment_type))
		print(data.pose[index])
		#Write to file with starting co-ords
		self.file.write("\nExperiment No."+str(self.experiment_number))
		self.file.write("\nStart: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")

		self.wheel_l_accum = 0
		self.wheel_r_accum = 0
		self.prev_tick = None
		#Start time in seconds from simulator
		self.start_time = 0
		#Time since experiment start
		self.time = 0
		
		#List of points from the simulator
		self.x_points = []
		self.y_points = []

		#List of points from odometry
		self.x_odom_points = []
		self.y_odom_points = []

		#init error calc experiment vars
		if(self.experiment_type > 1):
			self.stage = 0
			self.stage_change = True
			self.stage_init_x = 0
			self.stage_init_y = 0
			self.stage_init_ori = 0
			self.twist = Twist()
				

		self.experiment_running = True
		self.experiment_number += 1



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
		
	#Function to take quarternion of automower orientation in 3D space and convert to an angle from a bird eye view
	def quarternion_to_angle(self,x, y, z, w):
		ysqr = y * y

		t1 = +2.0 * (w * z + x * y)
		t2 = +1.0 - 2.0 * (ysqr + z * z)
		Z = math.atan2(t1, t2)
	
		return Z
	
	#Formula for calculating orientation change over time with wheel encoder data
	def orientation_change(self, e_left, e_right, orientation_start):
		#if encoder values are similar enough ignore the difference
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			return (((e_right-e_left)  )/self.wheel_separation_distance) + orientation_start
		else:	
			return orientation_start

	#Formula for calculating position change in x,y over time (Odometry formulae)
	def position_change(self, e_left, e_right, x_start, y_start, orientation_start):
		#again ignoring the difference in wheel encoders if similar enough
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			factor = (self.wheel_separation_distance * (e_right + e_left))/(2 * (e_right - e_left))
			trigonometric_input = ((  (e_right - e_left))/self.wheel_separation_distance) + orientation_start
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
		mat.layout.dim[0].size = 6
		mat.layout.dim[0].stride = 1
		mat.layout.data_offset = 0
		mat.data = [0]*6
		#experiement state, 0 = stopped
		mat.data[0] = state
		#time elapsed
		if(state == 0):
			mat.data[1] = 0
		else:
			mat.data[1] = self.time
		#X, Y and orientation data
		mat.data[2] = self.x
		mat.data[3] = self.y
		mat.data[4] = self.orientation
		#Error value
		mat.data[5] = 0
		self.pub_experiment_data.publish(mat)

	def run(self):
		try:
			self.init()
			
			#Setup subscribers
			rospy.Subscriber("wheel_encoder", WheelEncoder, self.wheel_encoder, queue_size=None)
			rospy.Subscriber('experiment/status', Float32, self.update_status)
			r = rospy.Rate(self.update_rate)
			while not rospy.is_shutdown():
				#When experiment is running...
				if(self.experiment_running):
					#If command recieved to shutdown from /experiment/status finish experiment
					if(self.experiment_shutting_down):
						self.finish_experiment()
					#Loop experiment until cancelled
					else:
						self.update()
					r.sleep()
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
	
		#write data (odom calculations etc)
		self.write_data()
		self.publish_experiment_data(1.0)
		
		#Error experiment
		if(self.experiment_type > 1):
			self.odom_navigation()

		self.counter += 1

	def odom_navigation(self):
		#If new stage (new rotation or forward phase) reset pos/orientation vars
		if(self.stage_change):
			self.stage += 1
			self.stage_init_x = self.x
			self.stage_init_y = self.y
			self.stage_init_ori = self.orientation
			#reset automower command
			if(self.stage % 2 == 1):
				self.twist.linear.x = self.automower_vel
				self.twist.angular.z = 0
				self.pub_odom_navigation.publish(self.twist)
				#update figure only in start of rotation (not needed for than just the start)
				self.plot_figure()
			else:
				self.twist.linear.x = 0
				#Clockwise or anti-clockwise
				if(self.experiment_type == 2):
					self.twist.angular.z = -self.automower_rot_vel
				else:
					self.twist.angular.z = self.automower_rot_vel
				self.pub_odom_navigation.publish(self.twist)
			self.stage_change = False
			

		if(self.stage % 2 == 1):
			#print(self.distance_calc(self.stage_init_x, self.stage_init_y, self.x, self.y))
			distance = self.distance_calc(self.stage_init_x, self.stage_init_y, self.x, self.y)
			
			#Update figure while moving forward, but not too close to rotation point 
			if(distance < self.experiment_side_length - 0.25):
				self.plot_figure()
			#End of state if distance travelled is distance reached
			if(distance > self.experiment_side_length):
				print(self.distance_calc(self.stage_init_x, self.stage_init_y, self.x, self.y), self.experiment_side_length)
				#reset command
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.pub_odom_navigation.publish(self.twist)
				self.stage_change = True
		else:
			#End of rotation stage if rotated 90 degrees, within a margin e.g. 0.2
			if(abs(self.angle_diff(self.stage_init_ori, self.orientation)) > math.radians(90 - 0.2)):
				print(abs(self.angle_diff(self.stage_init_ori, self.orientation)), math.radians(90))
				#reset command
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.pub_odom_navigation.publish(self.twist)
				self.stage_change = True
		
		#End of experiment (8 stages, 4 straights, 4 corners)
		if(self.stage > 8):
			#reset command
			self.twist.linear.x = 0
			self.twist.angular.z = 0
			self.pub_odom_navigation.publish(self.twist)
			self.stage = 0
			self.experiment_shutting_down = True

	#Compute distance, pythag on delta X and Y
	def distance_calc(self, x_start, y_start, x, y):
		return math.sqrt((x_start - x)**2 + (y_start - y)**2)

	#Difference in angles in range -pi -> pi, 
	def angle_diff(self, theta1, theta2):
		temp = theta1 - theta2
		
		if( temp > math.radians(180) ):
			temp -= math.radians(360)
		elif(temp < math.radians(-180)):
			temp += math.radians(360)
		return temp

	def write_data(self):
		#Get initial variables from simulator
		data = rospy.wait_for_message("gazebo/model_states", ModelStates)
		#get the index into the data for the automower
		index = data.name.index("automower")
		
		#gps update simulator
		self.gps_x = data.pose[index].position.x + random.uniform(-1, 1)
		self.gps_y = data.pose[index].position.y + random.uniform(-1, 1)

		#Calculate position change, then orientation
		(self.x, self.y) = self.position_change(self.wheel_l_accum, self.wheel_r_accum, self.x, self.y, self.orientation)
		self.orientation = self.orientation_change(self.wheel_l_accum, self.wheel_r_accum, self.orientation)
		self.orientation = ((self.orientation + math.pi) % (2*math.pi)) - math.pi
			
		#print(self.time, self.orientation, self.x, self.y)
		self.file.write("Time = " + str(self.time) + "\n")
		self.file.write("Odometry: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")
		self.file.write("Actual	: x = " + str(data.pose[index].position.x) + " y = " + str(data.pose[index].position.y) + " theta = " + str(self.quarternion_to_angle(data.pose[index].orientation.x, data.pose[index].orientation.y,data.pose[index].orientation.z,data.pose[index].orientation.w)) + "\n\n")

		#print actual position info to screen
		#print(data.pose[index])	
		#print(self.orientation)	

		#Record x and y, and x and y predictions from odometry
		self.x_odom_points.append(self.x)
		self.y_odom_points.append(self.y)

		self.x_points.append(data.pose[index].position.x)
		self.y_points.append(data.pose[index].position.y)

		#Resets accumalative encoders
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0

	def update_status(self, data):
		#Callback function from subscribed topic about experiment status
		#start new experiment with data.data length
		if(data.data > 0):
			if not(self.experiment_running):
				self.experiment_type = data.data
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
			self.plot_figure()

			#reset odom controller
			if(self.experiment_type > 1):
				self.twist.linear.x = 0
				self.twist.angular.z = 0
				self.pub_odom_navigation.publish(self.twist)
				self.stage = 0
			
			#Running var set to false to signal the end of the experiment			
			self.experiment_running = False

	def plot_figure(self):
		#Clear points and axis
		plt.cla()
		plt.clf()
		#Plot actual x co-ords
		plt.plot(self.x_points, self.y_points, 'k')
		#Plot odometry calculated points
		plt.plot(self.x_odom_points, self.y_odom_points, 'b--')

		#concatenate all x and all y points for min and max calculations for the graphs axis
		all_x_points = np.concatenate([self.x_points,self.x_odom_points])
		all_y_points = np.concatenate([self.y_points,self.y_odom_points])
		
		#axis sqaured off, uniform in x and y
		plt.axis('equal')
		#Set axis based on min and max vals in x and y
		plt.axis([np.amin(all_x_points) - 1, np.amax(all_x_points) + 1, np.amin(all_y_points) - 1, np.amax(all_y_points) + 1])
		#Save figure to file
		plt.savefig(self.file_path+'.png')
		

	def idle(self):
		#publish experiment data to topic, in this case the previous experiment's data
		self.publish_experiment_data(0.0)

	def fini(self):
		
		print('Finishing...')

if __name__ == '__main__':
	rospy.init_node('automower_tracking_simulator')
	tracker = automower_tracking_simulator()
	tracker.run()
