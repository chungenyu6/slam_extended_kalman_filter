#!/usr/bin/env python3
"""
 *
 * Chung-En Yu <chungenyu6@gmail.com>
 * May 2022
 * University of Minnesota, Electrical Engineering
 *
"""
"""
*
* Variables in following code can be refer to
* the notations in my notes, so it would be 
* more clear to look into the algorithm
*
"""

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from scipy.stats.distributions import chi2
from sympy import *
from math import atan2
import math
import rospy
import numpy as np
import sys
import random
import time


class TurtlebotLocalization:

	def __init__(self):		

		# Name & initiate your node
		rospy.init_node("turtlebot_drive", anonymous=False)
	
		# EKFpropagate() parameters
		self.x_hat = np.zeros(3)			# state estimate ( state = [x,y,phi] )
		self.u = np.zeros(2)				# inputs = linear, angular speed ( u = [v,omega] )
		self.x_n = np.zeros(2)				# state noise of [1] linear [2] angular speed
		self.dt = 0
		self.P = np.zeros((3,3))			# covariance of state estimate
		self.Q = np.zeros((3,3))			# covariance of state noise
		self.Phi = np.zeros((3,3))			# partial derivative of state_func w.r.t. state 
		self.G = np.zeros((3,3))			# partial derivative of state_func w.r.t. noise
		self.z = 0							# distance-only measurement ( only has x,y )
		self.z_n = 0						# measurement noise (x, y)
		self.p_R = np.zeros(2)				# current position of robot
		self.H = np.zeros((1,3))			# partial derivative of measurement_func w.r.t. measurement 
		self.R = np.zeros((1,1))				# convariance of measurement noise
		
		# System parameters
		self.updateFreq = 200 				# update every 200 timesteps
		self.landmarks = [ [-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 0], [0, 1], [1, -1], [1, 0], [1, 1] ]	# 9 known landmarks
		self.counter = 0
		
		# User interface
		print("[INFO] System activated")
		
		# The main loop will run at a rate of 10Hz, i.e., 10 times per second
		self.rate = rospy.Rate(10)
		
	
	""" Do jacobian to get Phi, G """
	def state_pde(self):
	
		# Define variables as symbols, more intuitive to do the math
		x, y, dt, v, noise_v, phi, omega, noise_omega =				\
		symbols('x y dt v noise_v phi omega noise_omega', real = True)
		
		# State equations
		x_func = x + dt * ( v + noise_v ) * cos(phi)
		y_func = y + dt * ( v + noise_v ) * sin(phi)
		phi_func = phi + dt * ( omega + noise_omega )
		
		# Partial state functions w.r.t state	
		Phi_func = Matrix( [x_func, y_func, phi_func] )
		Phi_jacob = Phi_func.jacobian( [x, y, phi] )	
		
		# Subsitute function variables with real value
		self.Phi = Phi_jacob.subs([(x, self.x_hat[0]), (y, self.x_hat[1]), (phi, self.x_hat[2]), (dt, self.dt), 
									(v, self.u[0]), (omega, self.u[1]), (noise_v, 0), (noise_omega, 0)])
	     					
		# Partial state functions w.r.t noise
		G_func = Matrix( [x_func, y_func, phi_func] )
		G_jacob = G_func.jacobian( [noise_v, noise_v, noise_omega] )

		# Subsitute function variables with real value
		self.G = G_jacob.subs([(x, self.x_hat[0]), (y, self.x_hat[1]), (phi, self.x_hat[2]), (dt, self.dt), 
								(v, self.u[0]), (omega, self.u[1]), (noise_v, 0), (noise_omega, 0)])
	
		
	""" Propagate x, P """
	def EKFpropagate(self):
	
		# Propagate state estimate : x_hat_next = f(x_hat, u, 0)
		self.x_hat[0] = self.x_hat[0] + self.dt * self.u[0] * math.cos(self.x_hat[2])
		self.x_hat[1] = self.x_hat[1] + self.dt * self.u[0] * math.sin(self.x_hat[2])
		self.x_hat[2] = self.x_hat[2] + self.dt * self.u[1]
		
		# Propagate state covariance : P_next
		self.P = (self.Phi @ self.P @ self.Phi.T) + (self.G @ self.Q @ self.G.T)
		
		# Check values
#		print(" ===== EKF Propagate ===== ")
#		print("[INFO] State estimates : \n", self.x_hat)
#		print("[INFO] State estimates covariance : \n", self.P)
				
		
	""" distance-only measurement residual ( z add on noise at here ) """
	def measurement_residual(self, num_landmarks):
	
		posi_diff = np.zeros(2)											# position difference ( landmark - robot )
		z_hat = 0														# measurement estimates
		z = 0
		r = 0
		for i in range(0, 2):
			posi_diff[i] = self.landmarks[num_landmarks][i] - self.x_hat[i]
			
		z_hat = np.sqrt( (posi_diff[0]**2) + (posi_diff[1]**2) )
		z = z_hat + self.z_n						 					# real-world measurement has noise
		r = z - z_hat												# this case : r = measurement noise
		
		return r
				
	
	""" Update x, P, z, r, H, S, K """
	def EKFupdate(self, num_landmarks):
				
		# Initialize ( since only have x, y measurement, only consider these 2 )
		r = 0		
		r = self.measurement_residual(num_landmarks)				 
		H_diff_norm = np.zeros(2)			# dominator
		H_diff = np.zeros(2)				# numerator
		S = np.zeros((1,1))				 
		K_tmp = np.zeros((1,3))				 
		K = np.zeros(3)
				
		# Get H : partial derivative of measurement_func w.r.t. measurement
		for i in range(0, 2):			# only (x,y) are measurements
			H_diff_norm[i] = np.linalg.norm( self.landmarks[num_landmarks][i] - self.x_hat[i] )
			H_diff[i] = self.landmarks[num_landmarks][i] - self.x_hat[i]
			self.H[0][i] = - H_diff[i] / H_diff_norm[i]

		# Get S, K
		S = self.H @ self.P @ self.H.T + self.R
		K_tmp = self.P @ self.H.T @ S
		for i in range(0,3):
			K[i] = K_tmp[i]
		
		# Update x, P
		self.x_hat = self.x_hat + K * r
		self.P = self.P - ( K_tmp @ self.H @ self.P )
						
		# Check values
		print(" ===== EKF Update ===== ")
		print("[INFO] EKF estimated robot position :\n", self.x_hat)
#		print("[INFO] EKF estimated robot covariance :\n", self.P)
#		print("H:\n", self.H)	
#		print("R:\n", self.R)		
#		print("S:\n", S)
#		print("K:\n", K)	
#		print("r:\n", r)	
	
	
	def odom_callback(self, msg):		
	
		# Absolute robot position	
		self.p_R[0] = msg.pose.pose.position.x	
		self.p_R[1] = msg.pose.pose.position.y
#		print("[INFO] Abosolute robot position: \t", self.p_R)
		
		
	def vel_callback(self, msg):
	
		# Absolute robot speed
		self.u[0] = msg.linear.x
		self.u[1] = msg.angular.z
#		print("[INFO] Absolute robot speed: \t", self.v, ",\t", self.omega)
										
	
	def activate(self):		
	
		# Initialize local variables		
		std_x_n_v = 0.01									# standard deviation of noise of v
		std_x_n_omega = np.pi/60
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
		self.x_hat[0] = -2					# initialize robot pose
		self.x_hat[1] = -0.5				# initialize robot pose
		self.Q[0,0] = std_x_n_v * std_x_n_v
		self.Q[1,1] = std_x_n_v * std_x_n_v
		self.Q[2,2] = std_x_n_omega * std_x_n_omega
		self.P[0,0], self.P[1,1], self.P[2,2] = 0.01, 0.01, np.pi/90		
				
		# Quit if ROS is not OK, that is, the master is dead
		while not rospy.is_shutdown():				
			
			# Fetch sensor data ( velocity of robot )
			self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
			self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
			
			# Random noise in control inputs ( simulate real world noise in control signal )
			self.x_n[0] = np.random.normal(std_x_n_v)			
			self.x_n[1] = np.random.normal(std_x_n_omega)
			
			# Get parameters for EKF equations
			now = rospy.get_rostime()			# current time
			self.dt = now.secs/1000				# time instance (delta_t) 
			self.state_pde()
			
			# Get R ( every time measure different landmark has different noise )
			std_z_n = 0.05								# standard deviation of z
			self.R[0][0] = std_z_n*std_z_n				# covariance of z_n
			self.z_n = np.random.normal(0,std_z_n)		# random noise for sensor measurement

			# EKF propagate
			self.EKFpropagate()
			
			# EKF update 
			# Every timestep use 9 landmarks to do EKF update
			if (self.counter % self.updateFreq == 0):
				for i in range(0, len(self.landmarks)):
					self.EKFupdate(i)
			self.counter = self.counter + 1
							
			# Sleep for as long as needed to achieve the loop rate.
			self.rate.sleep()	


if __name__ == "__main__":
	try:
		localization = TurtlebotLocalization()
		localization.activate()
	except rospy.ROSInterruptException:
		pass
