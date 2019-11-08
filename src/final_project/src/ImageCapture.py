#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import math

class ImageCapture:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.frame = None
	self.bridge = CvBridge()
        self.capturing = True
	self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image)
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

	self.state = 0

	self.cmd_pub = rospy.Publisher("/cmd_vel/", Twist)
	self.rate = rospy.Rate(1000)
	self.linear_vel = 0.5

	self.kp = 5
	self.kd = 1

	self.delta_error = 0
	self.error = 0

    def scale(self, percent):
	scale_percent = percent # percent of original size
        width = int(self.frame.shape[1] * scale_percent / 100)
        height = int(self.frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        self.frame = cv2.resize(self.frame, dim, interpolation = cv2.INTER_AREA)


    def callback(self, data):
	try:
		self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	# scale the image down for faster processing
	self.scale(50)

	# process the image into lines and then detect main line
	gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
	retval, filtered = cv2.threshold(edges, 127,255, cv2.THRESH_BINARY)
	lines = cv2.HoughLines(filtered, 1, np.pi/180, 5) # might look into detecting largest line
	self.error = float("inf")

	# display the filtered image
	self.frame = filtered

	if lines is not None:
		for rho,theta in lines[0]:
			line = self.generateLine(rho, theta)

			x1, y1 = line[0]
			x2, y2 = line[1]

			cv2.line(self.frame, (x1, y1), (x2, y2),(255,255,255),2)
			middle_x = int(self.frame.shape[1] / 2)

			# setup the ground truth line
			cv2.line(self.frame, (middle_x, 0), (middle_x, self.frame.shape[0]),(255,255,255),2)
			gradient = float("-inf")
			intercept = float("inf")

			if x2 - x1 != 0:
				gradient = float(y2 - y1) / float(x2 - x1)

			if gradient == 0:
				self.error = 1
			else:
				intercept = y2 - (gradient * x2)
				calculated_error = self.sigmoid(1 / gradient)# min(100, abs(math.atan2(gradient)))

				self.delta_error = min(calculated_error - self.error, 100)
				self.error = min(calculated_error, 100)

			# self.renderText("y = " + str(gradient) + "x + " + str(intercept))

	print(self.error)

	# self.renderText("Error: " + str(self.error), (10, self.frame.shape[0] - 100), (255, 0, 0))

	# Display the resulting frame
	self.render(self.frame)

    def follow_line(self):
	msg = Twist()

	proportional_component = self.kp * self.error
	derivative_component = seld.kd * self.delta_error
	error_correction = proportional_component + derivative_component
	angular_cap = 5.5

	msg.angular.z = max(min(error_correction, angular_cap), -1 * angular_cap)
	self.cmd_pub.publish(msg)

    def start(self):
	while not rospy.is_shutdown():
		# track the line
		self.follow_line()
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
