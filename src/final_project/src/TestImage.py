#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import math
from collections import deque

class ImageCapture:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.frame = None
	self.bridge = CvBridge()
        self.capturing = True
	self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image)
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
	self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)


	self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
	self.rate = rospy.Rate(100)

	self.state = 0


	self.kp = 1
	self.kd = 5

	self.delta_error = 0
	self.error = 1

	self.frame_number = 0

	self.mvmt_msg = Twist()

	self.directions = deque()
	self.directions.append(0) # 0 = straight, 1 = left, -1 = right
	self.directions.append(0)
	self.directions.append(-1)

	self.current_turn = 0

	self.stopSignCascade = cv2.CascadeClassifier("stop_sign.xml")

	
	self.obstacle = False
	self.at_stopsign = True
	self.at_intxn = False
	self.turning = False


    def scale(self, percent):
	scale_percent = percent # percent of original size
        width = int(self.frame.shape[1] * scale_percent / 100)
        height = int(self.frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        self.frame = cv2.resize(self.frame, dim, interpolation = cv2.INTER_AREA)

    def scan_callback(self, data):
	num_measurements = len(data.ranges)
	midpoint = int(num_measurements/2)
	front_scan_1 = data.ranges[0:30]
	front_scan_2 = data.ranges[num_measurements-30:num_measurements]

	front_scan = front_scan_1 + front_scan_2

	for i in front_scan:
		if i < 0.5 and not i == "inf":
			self.obstacle = True
			#self.state = 0
			return

	self.obstacle = False

	#print self.at_intxn
	#if not self.at_intxn and not self.turning and not self.visibleStopSigns:
	#	self.state = 1


    def camera_callback(self, data):
	try:
		self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

	# analyse stop signs
	stopSigns = self.stopSignCascade.detectMultiScale(
		gray,
		scaleFactor=1.25,
		minNeighbors=5,
		minSize=(75,75),
		flags=cv2.CASCADE_SCALE_IMAGE
	)
	
	if len(stopSigns) > 0:
		# debugging only
		"""for (x, y, w, h) in stopSigns:
			cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)"""
		self.at_stopsign = True
		#self.state = 0
		print("Stop sign has been detected: " + str(time.time()))
		return
	
	self.at_stopsign = False
	
	self.frame = self.frame[300:480, 0:640]
	# process the image into lines and then detect main line)
	lbound = np.array([0,0,0], dtype = "uint8")
	ubound = np.array([50,50,50], dtype = "uint8")
	mask = cv2.inRange(self.frame, lbound, ubound)
	edges = cv2.Canny(mask, 50, 150, apertureSize = 3)
	lines = cv2.HoughLines(edges, 1, np.pi/180, 75) # might look into detecting largest line
		
	if self.turning:
		return
	
	# finds the x-coord of the middle of the frame
	middle_x = int(self.frame.shape[1]/2)
	if lines is not None:
		for rho, theta in lines[0]:
			line = self.generateLine(rho, theta)
			x1,y1 = line[0]
			x2,y2 = line[1]
		cv2.line(self.frame, (x1,y1), (x2,y2), (255,255,255), 3)

		calculated_error = self.calc_error(x1, y1, x2, y2, middle_x)/200.0
		#self.renderText("Error: " + str(calculated_error))
		self.delta_error = min(calculated_error - self.error, 100)
		self.error = calculated_error

		for line in lines:
			for rho, theta in line:
				curr_line = self.generateLine(rho, theta)
				x1,y1 = curr_line[0]
				x2,y2 = curr_line[1]
			if np.absolute(y1 - y2) < 150:
				#self.state = 2
				self.at_intxn = True
				#print ("Found perp line")
				return
			#else:
				#print("NOT FOUND")
		self.at_intxn = False

	else:
		self.state = 0


	# setup the ground truth line
	cv2.line(self.frame, (middle_x, 0), (middle_x, self.frame.shape[0]), (255,255,255),2)
	# for visualization and report information

	# Display the resulting frame
	#self.render(self.frame)

    def calc_error(self, x1, y1, x2, y2, middle_x):
	if x2-x1 == 0.0:
		return middle_x - x1

	slope = (y2-y1)/(x2-x1)
	b = y2 - slope*x2
	if slope == 0:
		c_x1 = 0
		c_x2 = self.frame.shape[1]
	else:
		c_x1 = (0-b)/slope
		c_x2 = (self.frame.shape[0]-b)/slope
	x_dev = 0
	x_dev += middle_x - c_x1
	x_dev += middle_x - c_x2

	return x_dev/2.0

    def follow_line(self):
	proportional_component = self.kp * self.error
	derivative_component = self.kd * self.delta_error
	error_correction = proportional_component + derivative_component
	angular_cap = 5.5
	self.mvmt_msg.linear.x = 0.1
	self.mvmt_msg.angular.z = error_correction
	#self.renderText("Command: " + str(error_correction))

	#max(min(error_correction, angular_cap), -1 * angular_cap)
        #print("angular change z: %f" % msg.angular.z)

	# self.cmd_pub.publish(self.mvmt_msg)

    def make_turn(self):
	if self.turning:
		time = 4
		if self.current_turn == 0:
			time = 4.5
		if rospy.get_time() - self.turn_start_time > time:
			self.turning = False
			self.at_intxn = False
			#self.state = 1
		else:
			self.mvmt_msg.angular.z = self.current_turn * np.pi/8
			self.mvmt_msg.linear.x = 0.1
	elif self.directions:
		print("Popping")
		#print(self.state)
		self.current_turn = self.directions.popleft()
		self.turning = True
		self.turn_start_time = rospy.get_time()

    def stop_robot(self):
	#print "stopping"
	self.mvmt_msg.linear.x = 0.0
	self.mvmt_msg.angular.z = 0.0
	#self.cmd_pub.publish(self.mvmt_msg)


    def calc_state(self):
	self.state = 1
	if self.obstacle or self.at_stopsign:
		self.state = 0
		return

	if self.at_intxn:
		self.state = 2


    def start(self):
	rospy.Rate(0.4).sleep()
	while not rospy.is_shutdown():
		self.calc_state()
		# track the line
		#print(self.state)
		if self.state == 0:
			self.stop_robot()
		elif self.state == 1:
			self.follow_line()
		elif self.state == 2:
			self.make_turn()
		self.cmd_pub.publish(self.mvmt_msg)
		self.rate.sleep()

    def sigmoid(self, t):
	return (2 / (1 + (math.exp(-1 * t)))) - 1

    def generateLine(self, rho, theta):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        return [(x1, y1), (x2, y2)]

    def render(self, image):
        cv2.imshow('frame', image)

	cv2.imwrite("frames/frame_%d.jpg" % self.frame_number, image)

	self.frame_number += 1

	cv2.waitKey(1)

    def renderText(self, text, position=(10, 25), color=(0, 255, 0)):
        cv2.putText(self.frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    def stop(self):
        self.camera.release()

if __name__ == '__main__':
	rospy.init_node('image_capture')
	capture = ImageCapture()
	capture.start()
	capture.stop()
