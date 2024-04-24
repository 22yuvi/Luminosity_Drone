#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from luminosity_drone.msg import Biolocation
from pid_tune.msg import PidTune
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import math
import rospy
import time

import rospy
from swift_msgs.msg import swift_msgs

#image processing
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,25]
		self.led_point_hover=[0,0,25]
		 # Define the square spiral path
		self.setpoint_list = [[0,0,25]]
		self.setpoint = self.setpoint_list.pop(0)

        # Spiral parameters
		self.step_size = 4  # Step size between consecutive points
		self.inc_size = 4
		self.flag_found= 0
		self.enter=0
		self.reached_center=0
		self.centroid_list = []
		self.dotcnt = 0
		self.cX, self.cY = 0, 0
		self.end=0
		self.toler = 0.4
		# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500

		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters


		self.Kp = [12*0.6, 32*0.3, 1440*0.3]
		self.Ki = [0, 0, 0]
		self.Kd = [12*0.5, 28*0.3, 703*0.3]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.alt_error=0.0 # Variable for calculating errors for PID
		self.prev_alt_error=0.0
		self.sum_alt_error=0.0
		self.error_pitch=[0.0, 0.0]
		self.error_roll=[0.0, 0.0]
		self.prev_error=[0.0,0.0]
		self.sum_error=[0.0,0.0]
		self.min_throttle= 1000
		self.max_throttle= 2000


		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.033 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)





	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.pitch_roll_pid)
		rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)
		# rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		# rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

		# self.img = np.empty([])
		self.bridge = CvBridge()
		self.rate = rospy.Rate(10)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.3 # This is just for awrite a wrapper over the existing PID control system, written in Task 1 to fly the Swift drone through a list of set points in the simulation environment in Gazebo. n example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0004
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch): 
		self.Ki[1] = pitch.Ki * 0.0004
		self.Kd[1] = pitch.Kd * 0.3
		self.Kp[1] = pitch.Kp * 0.3

	def pitch_roll_pid(self,roll):
		self.Ki[0] = roll.Ki * 0.0008
		self.Kd[0] = roll.Kd * 0.3
		self.Kp[0] = roll.Kp * 0.06
	
	def led_detect(self,image):
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)

		# threshold the image to reveal light regions in the blurred image
		thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

		# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
		eroded = cv2.erode(thresh, (5,5), iterations=3)
		dilated = cv2.dilate(eroded, (7,7), iterations=2)
		
		# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
		labels = measure.label(dilated, connectivity=2, background=0)
		mask = np.zeros(dilated.shape, dtype="uint8")
		

		# loop over the unique components
		for label in np.unique(labels):
		
			# if this is the background label, ignore it
			if label == 0:
				continue
			self.dotcnt = label
			# otherwise, construct the label mask and count the number of pixels
			labelMask = np.zeros(dilated.shape, dtype="uint8")
			labelMask[labels == label] = 255
			numPixels = cv2.countNonZero(labelMask)
			# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
			if numPixels > 40:
				mask = cv2.add(mask, labelMask)
				# cv2.imshow(f"Mask{label}", mask)
				# rospy.loginfo("Masking done")

		# find the contours in the mask, then sort them from left to right
		self.cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		# rospy.loginfo(f"LED Trial {len(cnts)}")
		if len(self.cnts)!=0:
			self.cnts = contours.sort_contours(self.cnts)[0]
		

		# Initialize lists to store centroid coordinates and area
		
		area_list = []

		# Loop over the contours
		for i, c in enumerate(self.cnts):
			# Calculate the area of the contour
			area = float(cv2.contourArea(c))
			# Calculate the centroid of the contour
			M = cv2.moments(c)
			if M["m00"] != 0:
				self.cX = float(M["m10"] / M["m00"])
				self.cY = float(M["m01"] / M["m00"])
			else:
				self.cX, self.cY = 0, 0

			# Append centroid coordinates and area to the respective lists
			
			area_list.append(area)
			if self.flag_found==0:
				self.setpoint_list=[]
				self.flag_found=1
				self.setpoint_list.append([self.drone_position[0],self.drone_position[1],25])
				self.next_setpoint()
				# rospy.loginfo(f"flag value in 1: {self.flag_found}")
			else:
				if self.flag_found==1:
					self.centroid_list.append([self.cX, self.cY])
					# rospy.loginfo(f"Centroid: {self.cX, self.cY}")
					self.setpoint_list=[]
					self.setpoint_list.append([self.drone_position[0]+(self.cX-250)/100, self.drone_position[1]+(self.cY-250)/100, self.drone_position[2]])
					self.next_setpoint()
				if self.is_at_center()==1 :
					# rospy.loginfo(f"Drone Position {self.drone_position[0], self.drone_position[1], self.drone_position[2]}")
					self.flag_found=2
					if self.reached_center==0 :
						self.pub_Biolocation()
						self.reached_center = 1
						self.setpoint_list=[]
						self.toler = 0.2
						self.setpoint_list.append([11,11,34])
						self.next_setpoint()
			self.led_point_hover=self.drone_position


	def image_callback(self, data):
		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		image=self.img
		self.led_detect(image)
		# cv2.imshow("camera", self.img)
		# cv2.waitKey(1)
	
	def pub_Biolocation(self):
		self.bioLoc = Biolocation()
		if self.dotcnt == 2:
			self.bioLoc.organism_type = 'alien_a'
		elif self.dotcnt== 3:
			self.bioLoc.organism_type = 'alien_b'
		elif self.dotcnt == 4:
			self.bioLoc.organism_type = 'alien_c'
		self.bioLoc.whycon_x = self.drone_position[0]
		self.bioLoc.whycon_y = self.drone_position[1]
		self.bioLoc.whycon_z = 0
		rospy.loginfo(f"Biolocation: {self.bioLoc}")
		self.biolocation_pub.publish(self.bioLoc)


	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
		
        # Calculate error for the altitude (z-axis)
		error_z = self.setpoint[2]+0.4 - self.drone_position[2]

		# Calculate PID output for altitude (z-axis)
		self.alt_error = -error_z
		self.cmd.rcThrottle = int(1588 + self.alt_error * self.Kp[2] - (self.alt_error - self.prev_alt_error) * self.Kd[2] + self.sum_alt_error * self.Ki[2])

		# Calculate error for roll (x-axis)
		error_roll = self.setpoint[0] - self.drone_position[0]

		# # Calculate PID output for roll (x-axis)
		self.cmd.rcRoll = int(1500 + error_roll * self.Kp[0] - (error_roll - self.prev_error[0]) * self.Kd[0] + self.sum_error[0] * self.Ki[0])

		# # Calculate error for pitch (y-axis)
		error_pitch = self.setpoint[1] - self.drone_position[1]

		# # Calculate PID output for pitch (y-axis)
		self.cmd.rcPitch = int(1500 - error_pitch * self.Kp[1] + (error_pitch - self.prev_error[1]) * self.Kd[1] - self.sum_error[1] * self.Ki[1])

		# Ensure that the outputs are limited within a reasonable range
		self.limit_output()

		# Update previous errors and sum of errors
		self.prev_error[0] = error_roll
		self.prev_error[1] = error_pitch
		self.prev_alt_error = error_z
		self.sum_error[0] += error_roll
		self.sum_error[1] += error_pitch
		self.sum_alt_error += error_z

		# rospy.loginfo(f"Drone Position {self.drone_position[0], self.drone_position[1], self.drone_position[2]}")

		# Publish the control commands and error values
		self.publish_commands()
		if self.is_at_setpoint()==1:
			if self.flag_found==0:
				# Switch to the next setpoint
				# Generate the square spiral path
				if self.inc_size == 0:
					inc_size = 2
				if self.enter==0:
					self.setpoint[0] =self.setpoint[0]+ self.step_size
					self.setpoint_list.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
					self.enter+=1
				elif self.enter==1:
					self.enter+=1
					self.setpoint[1] = self.setpoint[1]+ self.step_size
					self.setpoint_list.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
					self.step_size = self.step_size + self.inc_size

				elif self.enter==2:
					self.setpoint[0] -= self.step_size
					self.setpoint_list.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
					self.enter+=1
				elif self.enter==3:
					self.setpoint[1] -= self.step_size
					self.setpoint_list.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
					self.inc_size = self.inc_size - 2
					self.step_size = self.step_size + self.inc_size
					self.enter=0
				self.next_setpoint()
			elif self.reached_center==1:
				self.setpoint_list.append([11,11,37])
				self.reached_center = 2
				self.next_setpoint()
				self.next_setpoint()


			
			
	def is_at_setpoint(self):
			tolerance = self.toler

			return abs(self.drone_position[0] - self.setpoint[0]) < tolerance and \
				abs(self.drone_position[1] - self.setpoint[1]) < tolerance and \
				abs(self.drone_position[2] - self.setpoint[2]) < tolerance

	def is_at_center(self):
			tolerance = 20

			return abs(self.cX - 250) < tolerance and \
				abs(self.cY - 250) < tolerance 


	def next_setpoint(self):
		# Move to the next setpoint in the list
		if len(self.setpoint_list) > 0:
			self.setpoint = self.setpoint_list.pop(0)
			# rospy.loginfo(f"Setpoint {self.drone_position[0], self.drone_position[1], self.drone_position[2]}")
			self.pid()
		else:
			# self.setpoint_list.append([11,11,37])
			# self.next_setpoint()
			# while not self.is_at_setpoint()==0:
			# 	pass
			
			rospy.loginfo("Reached the last setpoint.")
			self.disarm()
			raise rospy.exceptions.ROSInterruptException("ROS shutdown request")


	def limit_output(self):
		# Limit roll, pitch, and throttle outputs to reasonable values
		self.cmd.rcRoll = max(min(self.cmd.rcRoll, 2000), 1000)
		self.cmd.rcPitch = max(min(self.cmd.rcPitch, 2000), 1000)
		self.cmd.rcThrottle = max(min(self.cmd.rcThrottle, 2000), 1000)

	def publish_commands(self):
		# Publish the control commands and error values
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.alt_error)
		self.pitch_error_pub.publish(self.prev_error[1])
		self.roll_error_pub.publish(self.prev_error[0])





if __name__ == '__main__':
    	
	t = time.time()
	while time.time() - t < 5:
    		pass

	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()

		r.sleep()
