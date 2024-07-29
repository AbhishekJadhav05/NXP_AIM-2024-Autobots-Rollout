#20 -> car width
#72 -> track width without lines
#80 -> track width

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import math

from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan

#OURS
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped


QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.25
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 0.25
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25
THRESHOLD_RAMP_MIN = 0.75
THRESHOLD_RAMP_MAX = 1.2

TIME_PERIOD = 1/30

#Min - 0.6179950833320618 and Max - 0.9302666783332825
#Min - 0.4310002624988556 and Max - 1.9826102256774902
class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		self.status = [0, 0, 0]
		self.prevSpeed, self.prevTurn = 0.75, 0
		self.min, self.max = 10, 0
		self.obs = 0

		# Subscription to get Pose
		self.subscription_pose = self.create_subscription(
			PoseWithCovarianceStamped,
			'/pose',
			self.pose_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for edge vectors.
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False

	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)


	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""

	def edge_vectors_callback(self, message):
		speed = SPEED_MAX
		turn = TURN_MIN

		vectors = message
		half_width = vectors.image_width / 2

		# NOTE: participants may improve algorithm for line follower.
		if (vectors.vector_count == 0):  # none.
			speed = SPEED_50_PERCENT
			turn = self.prevTurn*0.95
			#print("ZERO (0) Vectors formed")

		if (vectors.vector_count == 1):  # curve.
			# Calculate the magnitude of the x-component of the vector.
			deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
			turn = deviation * 2 / vectors.image_width
			#turn = self.prevTurn*0.2 + turn*0.8
			
			speed = speed * (np.abs(math.cos(turn))**(1/3))
			#print("ONE (1) Vector formed")
			#speed = 0.05

		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
			middle_x = (middle_x_left + middle_x_right) / 2

			deviation = half_width - middle_x
			turn = deviation / half_width
			#print(f"t - {turn}, p - {self.prevTurn} and oni san {turn*0.7+self.prevTurn*0.3}")			
			#turn = turn*0.8 + self.prevTurn*0.2
			speed = speed * (np.abs(math.cos(turn)) **(1/5))
			#print("TWO (2) Vectors formed.")

		#smoothening it put
		#speed = speed*0.5 + self.prevSpeed*0.5

		if (self.traffic_status.stop_sign is True):
			speed = SPEED_MIN
			print("stop sign detected")

		if self.ramp_detected is True:
			# TODO: participants need to decide action on detection of ramp/bridge.
			speed = 0.6
			'''change it to reduce speed close to the ramp'''
			print("ramp/bridge detected")

		if self.obstacle_detected is True:
			# TODO: participants need to decide action on detection of obstacle.
			speed = SPEED_25_PERCENT
			turn = -1*self.obs*0.95 + turn*0.05
			print("obstacle detected")

		#While goind down/ after ramp to avoid bouncing of buggs
		if self.prevSpeed < 0.75 and speed > 0.59 and self.ramp_detected is False:
			#print("RAMP CASE ")
			speed = 0.999*self.prevSpeed + 0.001*speed

		self.prevSpeed = speed
		self.prevTurn = turn
		self.rover_move_manual_mode(speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None

		range_1=message.ranges
		#range_difference =  [range_1[i+1] - range_1[i] for i in range(len(range_1) - 1)]
        
		#Can also be for obsracles -> need to differentiate between them smhow
		for distance in range_1:
			if distance < 2:
				self.ramp_detected = True
				break
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.

		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)	#75.96

		flag1, flag2 = False, False
		self.ramp_detected = False

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		
		#backRanges = message.ranges[0:int(length / 4), int(3 * length / 4) : int(length)]
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]

		
		# process front ranges.
		angle1 = theta - PI / 2
		#angles=[]
		for i in range(len(front_ranges)):
			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				self.obsTurn = angle1 + np.arctan(0.05/front_ranges[i])*np.sign(angle1)
				#angles.append(angle)
				flag1 = True
				return
			#angles.append(angle1)
			angle1 += message.angle_increment

		# if flag1 is True:
		# 	closestRange = min(front_ranges)
		# 	index = front_ranges.index(closestRange)
		# 	angleAvoidance = angles[index]
		# 	angleSafe = np.arctan(0.05/min(list(front_ranges)))
		# 	print(f"Meow - {angleAvoidance} and meow {angleSafe} and lallal {min(list(front_ranges))}")
		# 	angle1 = angleAvoidance + angleSafe*np.sign(angleAvoidance)
		# 	self.obs = angle1

		#angle1 = 

		#print(angles)

		# process side ranges.
		side_ranges_left.reverse()
		#angles = []
		for side_ranges in [side_ranges_left, side_ranges_right]:
			angle2 = 0.0
			for i in range(len(side_ranges)):

				if (side_ranges[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
					self.obstacle_detected = True
					self.obsTurn = angle2 + np.arctan(0.05/side_ranges[i])*np.sign(angle2)
					return
					#flag2 = True
				#angles.append(angle2)
				angle2 += message.angle_increment

		# if flag2 is True:
		# 	side_ranges_both = []
		# 	side_ranges_both.extend(side_ranges_left)
		# 	side_ranges_both.extend(side_ranges_right)
		# 	closestRangeSide = min(side_ranges_both)
		# 	index = side_ranges_both.index(closestRangeSide)
		# 	angleAvoidance = angles[index]
		# 	angleSafe = np.arctan(0.05/min(list(side_ranges_both)))
		# 	print(f"Meow - {angleAvoidance} and meow {angleSafe}")
		# 	angle2 = angleAvoidance + angleSafe*np.sign(angleAvoidance)
		# 	self.obs = angle2

		
		#angleOfAvoidRight = np.arctan(10/min(list(side_ranges_right)))
		#angleOfAvoidLeft = np.arctan(10/min(list(side_ranges_left)))
		
		#Considering both angles
		#if angle != theta - PI / 2 and angle2 != 0:
		#	self.obs = angle1*0.6 + angle2*0.4
		# if flag2 is True and flag1 is True:
		# 	print("Both flag true condition.")
		# 	self.obs = angle1*0.7 + angle2*0.3
		# 	return
		
		# if flag1 is True or flag2 is True:
		# 	return

		self.obstacle_detected = False

		# RAMP
		l = len(front_ranges)
		new_front_ranges = front_ranges[int(l/6):int(5*l/6)]

		for i in range(len(new_front_ranges)):
			if (new_front_ranges[i] < THRESHOLD_RAMP_MAX and new_front_ranges[i] > THRESHOLD_RAMP_MIN):
				self.ramp_detected = True
				# if new_front_ranges[i] < self.min:
				# 	self.min = new_front_ranges[i]
				# elif new_front_ranges[i] > self.max:
				# 	self.max = new_front_ranges[i]
				# print(f"Min - {self.min} and Max - {self.max} and  current {new_front_ranges[i]}")
				return

		self.ramp_detected = False

	def pose_callback(self, Message):
		self.status = [Message.pose.pose.position.x, Message.pose.pose.position.y, 0]
		x = Message.pose.pose.orientation.x
		y = Message.pose.pose.orientation.y
		z = Message.pose.pose.orientation.z
		w = Message.pose.pose.orientation.w

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = np.arctan2(t3, t4)
		self.status[2] = yaw_z


def main(args=None):
	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
