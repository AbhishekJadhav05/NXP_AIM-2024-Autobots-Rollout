
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

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0
TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.6
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 0.75
THRESHOLD_OBSTACLE_HORIZONTAL = 0.45
THRESHOLD_RAMP_MIN = 0.9 #0.7
THRESHOLD_RAMP_MAX = 1.1

SAFE_DISTANCE = 0.3
SAFE_DISTANCE_STRAIGHT = 0.25
#Min - 0.6179950833320618 and Max - 0.9302666783332825
#Min - 0.4310002624988556 and Max - 1.9826102256774902
class LineFollower(Node):
    """ Initializes line follower node with the required publishers and subscriptions.
        Returns:
            None
    """
    def __init__(self):
        super().__init__('line_follower')
        self.prevSpeed, self.prevTurn = SPEED_MAX, 0.0
        self.min, self.max = 10, 0.0
        self.obs = 0.0
        self.speed, self.turn = 0.0, 0.0

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
        
        p_turn = 0.0
        kP_base = 0.7
        kD_base = 0.35
        
        # NOTE: participants may improve algorithm for line follower.
        
        if (vectors.vector_count == 1):  # curve.
            # Calculate the magnitude of the x-component of the vector.
            deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
            p_turn = deviation / half_width
            speed = SPEED_50_PERCENT * (np.abs(math.cos(p_turn))**(1/2))
            #speed = speed * (np.abs(math.cos(turn))**(1/2))
            #print("ONE (1) Vector formed")

        if (vectors.vector_count == 2):  # straight.
            # Calculate the middle point of the x-components of the vectors.
            middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
            middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
            middle_x = (middle_x_left + middle_x_right) / 2
            deviation = half_width - middle_x
            p_turn = deviation / half_width
            speed = speed * (np.abs(math.cos(p_turn))**(1/3))
            #speed = SPEED_MAX
            #print("TWO (2) Vectors formed.")
        
        deviation_magnitude = abs(p_turn)
        kP = kP_base * (0.9 + deviation_magnitude)
        kD = kD_base * (0.7 + deviation_magnitude)
        derivative_turn = (turn - self.prevTurn)

        turn = kP * p_turn + kD * derivative_turn

        if (vectors.vector_count == 0):  # none.
            speed = 0.4

            turn = self.prevTurn*0.9
            #print("ZERO (0) Vectors formed")

        if self.ramp_detected is True:
            # TODO: participants need to decide action on detection of ramp/bridge.
            speed = 0.55
            '''change it to reduce speed close to the ramp'''
            print("ramp/bridge detected")

        if self.prevSpeed < 0.85 and speed > 0.54 and self.obstacle_detected is False:
            speed = 0.995*self.prevSpeed + 0.005*speed

        if self.obstacle_detected is True:
            # TODO: participants need to decide action on detection of obstacle.
            speed = 0.45
            turn = -0.9*self.obs + turn*0.1
            #print("obstacle detected") 
        #While goind down/ after ramp to avoid bouncing of buggs
        
        if (self.traffic_status.stop_sign is True):
            speed = self.prevSpeed*0.9
            turn = turn*0.7
            if self.prevSpeed < 0.15:
                speed = SPEED_MIN
            print("stop sign detected")
        
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
        theta = math.atan(shield_vertical / shield_horizontal)  #75.96
        self.ramp_detected = False
        angles = []
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
        angleFront = theta - PI / 2
        for i in range(len(front_ranges)):
            #
            if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
                #print("FRONT",min(front_ranges))
                self.obstacle_detected = True
                angleAvoidance = angleFront
                angleSafe = np.arctan(SAFE_DISTANCE_STRAIGHT/front_ranges[i])
                angleFront = angleAvoidance + np.abs(angleSafe)*np.sign(angleAvoidance) 
                '''+ np.abs(angleSafe)'''
                #self.obs = angleFront
                #angles.append(angleFront)
                print('Front')
                break
            angleFront += message.angle_increment

        
        angleFront2 = PI / 2 - theta
        front_ranges.reverse()
        for i in range(len(front_ranges)):
            if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
                self.obstacle_detected = True
                if angleFront*angleFront2>0:
                    if angleFront > 0:
                        angleFront = angleFront2
                        angleSafe = np.arctan(SAFE_DISTANCE_STRAIGHT/front_ranges[i])
                    else:
                        angleFront = angleFront
                else:
                    angleFront += (angleFront2*0.9)
                angleFront += np.abs(angleSafe)*np.sign(angleFront)
                self.obs = angleFront
                angles.append(angleFront)
                break
            angleFront2 -= message.angle_increment


        close = []
        # process side Left
        #side_ranges_left.reverse()
        angleLeft = 0.0
        for i in range(len(side_ranges_left)):
            if (side_ranges_left[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                #print("LEFT",min(side_ranges_left))
                self.obstacle_detected = True
                angleAvoidance = angleLeft
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_left[i])
                angleLeft = angleAvoidance + np.abs(angleSafe)*np.sign(angleAvoidance)
                angleLeft = theta - angleLeft
                self.obs = angleLeft
                angles.append(angleLeft)
                close.append(side_ranges_left[i])
                print('Left')
                break
            angleLeft += message.angle_increment
        
        # process side Right
        angleRight = 0.0
        side_ranges_right.reverse()
        for i in range(len(side_ranges_right)):
            if (side_ranges_right[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                #print("RIGHT",min(side_ranges_right))
                self.obstacle_detected = True
                angleAvoidance = angleRight
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_right[i])
                angleRight = angleAvoidance + np.abs(angleSafe)*np.sign(angleAvoidance)
                angleRight = - theta + angleRight
                self.obs = angleRight
                angles.append(angleRight)
                close.append(side_ranges_right[i])
                print('Right')
                break
            angleRight += message.angle_increment
        
        if len(angles) == 3:
            print('3')
            if close[0] < close[1]:
                angle = angles[1] + 0.9*angles[2]
            else:
                angle = 0.9*angles[1] + angles[2]
            self.obs = angles[0]*0.5 + angle*0.5
            return
        
        if len(angles) == 2 and angles[0] == angleFront:
            print('2 w front')
            self.obs = np.dot(angles, [1,1])
            return
        
        elif len(angles) == 2:
            print('2 sides')
            if close[0] < close[1]:
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_right[i])
                self.obs = np.dot(angles, [1, 0.9]) + np.abs(angleSafe)*np.sign(angleAvoidance)
            else:
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_left[i])
                self.obs = np.dot(angles, [0.9, 1]) + np.abs(angleSafe)*np.sign(angleAvoidance)
            return
        if len(angles) == 1:
            print('1')
            return
        
        self.obstacle_detected = False
        # RAMP
        for i in range(len(front_ranges)):
            if (front_ranges[i] < THRESHOLD_RAMP_MAX and front_ranges[i] > THRESHOLD_RAMP_MIN):
                self.ramp_detected = True
                return
            
        self.ramp_detected = False

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
