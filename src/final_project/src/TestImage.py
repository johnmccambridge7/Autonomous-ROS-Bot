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

# Start and end nodes, and starting directions
START = 0
END = 15
DIRECTION = 0

# -----------------------------------------------------------------------
# START OF BFS PATH PLANNING

# Node class for intersections
class Node:
	def __init__(self, name):
		self.name = name
		self.previous = None
		self.up = None
		self.right = None
		self.down = None
		self.left = None


# Making the map
def make_map():
	# Make all the nodes first
	node_list = []
	node_list.append(Node("A"))
	node_list.append(Node("B"))
	node_list.append(Node("C"))
	node_list.append(Node("D"))
	node_list.append(Node("E"))
	node_list.append(Node("F"))
	node_list.append(Node("G"))
	node_list.append(Node("H"))
	node_list.append(Node("I"))
	node_list.append(Node("J"))
	node_list.append(Node("K"))
	node_list.append(Node("L"))
	node_list.append(Node("M"))
	node_list.append(Node("N"))
	node_list.append(Node("O"))
	node_list.append(Node("P"))

	# Make the connections between nodes
	node_list[0].right = Node_List[1]
	node_list[1].right = Node_List[2]
	node_list[2].right = Node_List[3]
	node_list[4].right = Node_List[5]
	node_list[5].right = Node_List[6]
	node_list[7].right = Node_List[8]
	node_list[8].right = Node_List[9]
	node_list[9].right = Node_List[10]
	node_list[10].right = Node_List[11]
	node_list[12].right = Node_List[13]
	node_list[13].right = Node_List[14]
	node_list[14].right = Node_List[15]

	node_list[1].left = Node_List[0]
	node_list[2].left = Node_List[1]
	node_list[3].left = Node_List[2]
	node_list[5].left = Node_List[4]
	node_list[6].left = Node_List[5]
	node_list[8].left = Node_List[7]
	node_list[9].left = Node_List[8]
	node_list[10].left = Node_List[9]
	node_list[11].left = Node_List[10]
	node_list[13].left = Node_List[12]
	node_list[14].left = Node_List[13]
	node_list[15].left = Node_List[14]

	node_list[4].up = Node_List[0]
	node_list[5].up = Node_List[1]
	node_list[7].up = Node_List[4]
	node_list[8].up = Node_List[5]
	node_list[9].up = Node_List[6]
	node_list[10].up = Node_List[2]
	node_list[11].up = Node_List[3]
	node_list[12].up = Node_List[7]
	node_list[13].up = Node_List[8]
	node_list[14].up = Node_List[9]
	node_list[15].up = Node_List[11]

	node_list[0].down = Node_List[4]
	node_list[1].down = Node_List[5]
	node_list[2].down = Node_List[10]
	node_list[3].down = Node_List[11]
	node_list[4].down = Node_List[7]
	node_list[5].down = Node_List[8]
	node_list[6].down = Node_List[9]
	node_list[7].down = Node_List[12]
	node_list[8].down = Node_List[13]
	node_list[9].down = Node_List[14]
	node_list[11].down = Node_List[15]

	return node_list


# Expanding a node
def expand(node, frontier, seen):
	next_node = node.up
	if next_node != None and next_node not in seen:
		frontier.append(next_node)
		seen.append(next_node)
		next_node.previous = node

	next_node = node.right
	if next_node != None and next_node not in seen:
		frontier.append(next_node)
		seen.append(next_node)
		next_node.previous = node

	next_node = node.down
	if next_node != None and next_node not in seen:
		frontier.append(next_node)
		seen.append(next_node)
		next_node.previous = node

	next_node = node.left
	if next_node != None and next_node not in seen:
		frontier.append(next_node)
		seen.append(next_node)
 		next_node.previous = node

# Convert Global direction to local robot turns
def convert_direction(result, robot_direction):
	# Convert to local direction
	for i in range(len(result)):
		temp = result[i]
		result[i] = (result[i] - robot_direction) % 4
		robot_direction = temp

	# Convert directions to be consitent with robot directions
	# 0 = straight, 1 = left, -1 = right
	for i in range(len(result)):
		if result[i] == 1:
			result[i] = -1

		if result[i] == 3:
			result[i] = 1

	# Make the list into a deque
	final = deque()
	for i in range(len(result)):
		final.append(result[i])

	return final


# What to do if solved
def solved(node, robot_direction):
	reorder = []

	while node is not None:
		reorder.insert(0, node)
		node = node.previous

	# We now have the path from node to node
	# Now, convert this to left and right turns

	result = []
	for i in range(len(reorder)):
		if i != len(reorder)-1:
 			if reorder[i].up == reorder[i + 1]:
				result.append(0)

			if reorder[i].right == reorder[i + 1]:
				result.append(1)

			if reorder[i].down == reorder[i + 1]:
				result.append(2)

			if reorder[i].left == reorder[i + 1]:
				result.append(3)

	return convert_direction(result, robot_Direction)


# Check if it is Goal
def is_goal(end, node):
	if end.name == node.name:
		return True
	return False


# Search algorithm
def bfs(start, end, direction):
	# Initialize some variables
	frontier = []
	seen = []
	frontier.append(start)
	seen.append(start)

	# Infinite loop, main
	while True:
		# If frontier is empty, there is no solution
		if frontier == []:
			print("IMPOSSIBLE")
			return
		else:
			# Choose first element in Frontier and check if goal is reached
			node = frontier.pop(0)
			if is_goal(end, node):
				return solved(node, direction)
			else:
				expand(node, frontier, seen)


# END OF BFS PATH PLANNING
# --------------------------------------------------------------------------------------------


# START OF ROBOT MOTION
# --------------------------------------------------------------------------------------------


# Class for the self-driving robot
class AutoDriver:

    # Initializes the robot
    def __init__(self, path):
        self.camera = cv2.VideoCapture(0)
        self.frame = None
	self.bridge = CvBridge()
        self.capturing = True
	self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image)
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
	self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

	# Publisher and message to move robot
	self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
	self.mvmt_msg = Twist()

	self.rate = rospy.Rate(100)

	self.state = 0 # Current state of the robot in FSM

	# Gains for PD
	self.kp = 0.5
	self.kd = 3

	# Error values for PD
	self.delta_error = 0
	self.error = 1

	# Queue of directions for path following
	self.directions = path

	# HTML used to train robot for stop sign detection
	self.stopSignCascade = cv2.CascadeClassifier("stop_sign.xml")

	# Flags for state determination
	self.obstacle = False
	self.at_stopsign = True
	self.at_intxn = False
	self.turning = False
	self.current_turn = 0


    # Callback for the LIDAR scanner
    def scan_callback(self, data):

	# Use just the front 60 degrees of scan measurements
	num_measurements = len(data.ranges)
	midpoint = int(num_measurements/2)
	front_scan_1 = data.ranges[0:30]
	front_scan_2 = data.ranges[num_measurements-30:num_measurements]
	front_scan = front_scan_1 + front_scan_2

	# Checks if an obstacle is in front of the robot
	for i in front_scan:
		if i < 0.5 and not i == "inf":
			self.obstacle = True
			return

	self.obstacle = False


    # Callback for the RGB camera
    def camera_callback(self, data):
	try:
		self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	# Convert frame to gray
	gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

	# Analyze stop signs
	stopSigns = self.stopSignCascade.detectMultiScale(
		gray,
		scaleFactor=1.25,
		minNeighbors=5,
		minSize=(75,75),
		flags=cv2.CASCADE_SCALE_IMAGE
	)
	
	# Checks if a stop sign is detected
	if len(stopSigns) > 0:
		# debugging only
		"""for (x, y, w, h) in stopSigns:
			cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)"""
		self.at_stopsign = True
		#self.state = 0
		print("Stop sign has been detected: " + str(time.time()))
		return
	
	self.at_stopsign = False
	
	# Crop the frame and filter into black and white for line detection
	self.frame = self.frame[300:480, 0:640]
	lbound = np.array([0,0,0], dtype = "uint8")
	ubound = np.array([50,50,50], dtype = "uint8")
	mask = cv2.inRange(self.frame, lbound, ubound)
	edges = cv2.Canny(mask, 50, 150, apertureSize = 3)
	lines = cv2.HoughLines(edges, 1, np.pi/180, 75) # Finds the lines

	# Prevents line detection if currently turning
	if self.turning:
		return
	
	# Finds the x-coord of the middle of the frame
	middle_x = int(self.frame.shape[1]/2)

	# If lines found, calculate error for PD
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

		# Checks to see if robot is at an intersection by finding horizontal lines
		for line in lines:
			for rho, theta in line:
				curr_line = self.generateLine(rho, theta)
				x1,y1 = curr_line[0]
				x2,y2 = curr_line[1]
			if np.absolute(y1 - y2) < 150: # Checks if the line is horizontal within a threshold
				self.at_intxn = True
				return
	
		self.at_intxn = False

	else:
		# If no lines found, stop robot
		self.state = 0


	# Draws the ground truth line
	cv2.line(self.frame, (middle_x, 0), (middle_x, self.frame.shape[0]), (255,255,255),2)

	# Display the resulting frame (Comment this out to make robot run faster)
	#self.render(self.frame)


    # Calculates the error for the PD
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

	# Finds average deviation
	x_dev = 0
	x_dev += middle_x - c_x1
	x_dev += middle_x - c_x2

	return x_dev/2.0


    # PD line follower method
    def follow_line(self):
	proportional_component = self.kp * self.error
	derivative_component = self.kd * self.delta_error
	error_correction = proportional_component + derivative_component
	angular_cap = 5.5
	self.mvmt_msg.linear.x = 0.1
	self.mvmt_msg.angular.z = error_correction
	#self.renderText("Command: " + str(error_correction))


    # Method to make left, right, and straight turns
    def make_turn(self):
	if self.turning:
		time = 4 # Time taken to turn
		if rospy.get_time() - self.turn_start_time > time:
			self.turning = False
			self.at_intxn = False
			#self.state = 1
		else:
			self.mvmt_msg.angular.z = self.current_turn * np.pi/6
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
        x2 = int(x0 - 1000*(-b))Make_map
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

	# Making the map and finding path
	node_list = make_map()
	path = bfs(node_list[START], node_list[END], DIRECTION) # (Start node, End node, Direction)

	robot = AutoDriver(path)
	robot.start()
	robot.stop()
