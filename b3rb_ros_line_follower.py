
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
SPEED_MAX = 1.65
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 0.8
THRESHOLD_OBSTACLE_HORIZONTAL = 0.45
THRESHOLD_RAMP_MIN = 0.9 #0.7
THRESHOLD_RAMP_MAX = 1.1

SAFE_DISTANCE = 0.175
SAFE_DISTANCE_STRAIGHT = 0.175
#Min - 0.6179950833320618 and Max - 0.9302666783332825
#Min - 0.4310002624988556 and Max - 1.9826102256774902
class LineFollower(Node):
    """ Initializes line follower node with the required publishers and subscriptions.
        Returns:
            None
    """
    def __init__(self):
        super().__init__('line_follower')
        self.prevSpeed, self.prevTurn = SPEED_MAX, 0
        self.min, self.max = 10, 0
        self.obs = 0
        self.speed, self.turn = 0.0, 0.0

        # Subscription for LIDAR data.
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)

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
        kP_base = 0.95
        kD_base = 0.5
        
        # NOTE: participants may improve algorithm for line follower.
        
        if (vectors.vector_count == 0):  # none.
            speed = 0.4
            p_turn = self.prevTurn*0.9


        if (vectors.vector_count == 1):  # curve.
            # Calculate the magnitude of the x-component of the vector.
            deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
            p_turn = deviation  / half_width
            speed = SPEED_75_PERCENT * (np.abs(math.cos(p_turn))**(1/2))
            #speed = speed * (np.abs(math.cos(turn))**(1/2))
            #print("ONE (1) Vector formed")

        if (vectors.vector_count == 2):  # straight.
            # Calculate the middle point of the x-components of the vectors.
            middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
            middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
            middle_x = (middle_x_left + middle_x_right) / 2
            deviation = half_width - middle_x
            p_turn = deviation  / half_width
            speed = speed * (np.abs(math.cos(p_turn))**(1/4))
            #speed = SPEED_MAX
            #print("TWO (2) Vectors formed.")

        if self.obstacle_detected is True:
            # TODO: participants need to decide action on detection of obstacle.
            speed = 0.45
            p_turn = -0.95*self.obs + p_turn*0.05 
            
            # if self.closest >= 0.6:
            # # Maintain cruising speed
            #     speed = SPEED_50_PERCENT
            # elif self.closest <= 0.2:
            #     # Reduce to baseline velocity
            #     SPEED_25_PERCENT
            # else:
            #     # Linear interpolation between cruise speed and baseline speed
            #     speed = SPEED_25_PERCENT*0.8 + (SPEED_50_PERCENT - SPEED_25_PERCENT) * \
            #             ((self.closest - 0.2) / (0.6 - 0.2))
            '''
            if distance_to_obstacle >= self.d_max:
            # Maintain cruising speed
            return self.v_cruise
            elif distance_to_obstacle <= self.d_min:
                # Reduce to baseline velocity
                return self.v_baseline
            else:
                # Linear interpolation between cruise speed and baseline speed
                velocity = self.v_baseline + (self.v_cruise - self.v_baseline) * \
                        ((distance_to_obstacle - self.d_min) / (self.d_max - self.d_min))
                return velocity
            '''

        deviation_magnitude = abs(p_turn)
        kP = kP_base * (1 + deviation_magnitude)
        kD = kD_base * (0.7 + deviation_magnitude)
        derivative_turn = (turn - self.prevTurn)

        turn = kP * p_turn + kD * derivative_turn
           

        #make it less sensitive - can define new variables to make it look clean
        #speed = (kP) * speed + kD * (speed - self.prevSpeed)
        #speed = abs(speed)
            #print("ZERO (0) Vectors formed")

        if self.ramp_detected is True:
            # TODO: participants need to decide action on detection of ramp/bridge.
            speed = 0.55
            '''change it to reduce speed close to the ramp'''
            print("ramp/bridge detected")

        if self.prevSpeed < 0.8 and speed > 0.54 and self.obstacle_detected is False:
            speed = 0.995*self.prevSpeed + 0.005*speed

        
            #print("obstacle detected") 
        #While goind down/ after ramp to avoid bouncing of buggs
        
        if (self.traffic_status.stop_sign is True):
            speed = self.prevSpeed*0.95
            turn = turn*0.7
            if self.prevSpeed < 0.15:
                speed = SPEED_MIN
            print("stop sign detected")
        
        self.prevSpeed = speed
        self.prevTurn = turn
        print(f"Turn : {turn} and speed : {speed}")
        self.rover_move_manual_mode(float(speed), float(turn))
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
        self.ramp_detected, self.obstacle_detected = False, False

        angles = []
        self.closest = 2
        
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
            if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
                #print("FRONT",min(front_ranges))
                self.obstacle_detected = True
                angleSafe = np.abs(np.arctan(SAFE_DISTANCE_STRAIGHT/front_ranges[i]))*np.sign(angleFront)
                angleFront = angleSafe#np.abs(angleSafe)*np.sign(angleFront) 
                print('Front')
                print(angleFront)
                if self.closest > front_ranges[i]:
                    self.closest = front_ranges[i]
                break
            angleFront += message.angle_increment

        
        angleFront2 = PI / 2 - theta
        front_ranges.reverse()
        for i in range(len(front_ranges)):
            if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
                self.obstacle_detected = True
                print(angleFront2)
                angleFront2 = np.abs(np.arctan(SAFE_DISTANCE_STRAIGHT/front_ranges[i]))*np.sign(angleFront2)
                if angleFront*angleFront2>0:
                    if angleFront > 0:
                        angleFront = angleFront2
                        angleSafe = np.arctan(SAFE_DISTANCE_STRAIGHT/front_ranges[i])
                        #angleFront = min(angleFront, angleFront2)
                        #angleFront = (PI/2 - theta) - angleFront
                    else:
                        angleFront = angleFront
                        #angleFront = max(angleFront, angleFront2)
                        #angleFront = -(PI/2 - theta) + angleFront
                else:
                    angleFront += angleFront2
                #angleFront += np.abs(angleSafe)*np.sign(angleFront)
                print(angleFront)
                self.obs = angleFront
                angles.append(angleFront)
                if self.closest > front_ranges[i]:
                    self.closest = front_ranges[i]
                break
            angleFront2 -= message.angle_increment


        close = []
        # process side Left
        side_ranges_left.reverse()
        angleLeft = 0.0
        for i in range(len(side_ranges_left)):
            if (side_ranges_left[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                #print("LEFT",min(side_ranges_left))
                self.obstacle_detected = True
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_left[i])
                angleLeft = theta - angleLeft

                angleLeft = np.abs(angleSafe)*np.sign(angleLeft)
                
                self.obs = angleLeft
                angles.append(angleLeft)
                close.append(side_ranges_left[i])
                print('Left')
                print(angleLeft)
                if self.closest > side_ranges_left[i]:
                    self.closest = side_ranges_left[i]
                break
            angleLeft += message.angle_increment
        
        # if min(front_ranges) < THRESHOLD_OBSTACLE_HORIZONTAL:
        #     self.obstacle_detected = True
        #     angleFront = np.abs(np.arctan(SAFE_DISTANCE/min(front_ranges))) * np.sign(angleFront)
        #     self.obs = angleFront
        #     angles.append(angleFront)
        #     if self.closest > min(front_ranges):
        #         self.closest = min(front_ranges)

        # if min(side_ranges_left) < THRESHOLD_OBSTACLE_HORIZONTAL:
        #     self.obstacle_detected = True
        #     angleLeft = np.arctan(SAFE_DISTANCE/min(side_ranges_left))
        #     angleLeft = np.abs(angleLeft)
        #     self.obs = angleLeft
        #     close.append(min(side_ranges_left))
        #     if self.closest > min(side_ranges_left):
        #             self.closest = min(side_ranges_left)
        #     angles.append(angleLeft)

        # if min(side_ranges_right) < THRESHOLD_OBSTACLE_HORIZONTAL:
        #     self.obstacle_detected = True
        #     angleRight = np.arctan(SAFE_DISTANCE/min(side_ranges_right))
        #     angleRight = -1*np.abs(angleRight)
        #     self.obs = angleRight
        #     close.append(min(side_ranges_right))
        #     if self.closest > min(side_ranges_right):
        #             self.closest = min(side_ranges_right)
        #     angles.append(angleRight)

        # process side Right
        angleRight = 0.0
        side_ranges_right.reverse()
        for i in range(len(side_ranges_right)):
            if (side_ranges_right[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
                #print("RIGHT",min(side_ranges_right))
                self.obstacle_detected = True
                angleSafe = np.arctan(SAFE_DISTANCE/side_ranges_right[i])
                angleRight = - theta + angleRight

                angleRight = np.abs(angleSafe)*np.sign(angleRight)
                
                self.obs = angleRight
                angles.append(angleRight)
                close.append(side_ranges_right[i])
                print('Right')
                print(angleRight)
                if self.closest > side_ranges_right[i]:
                    self.closest = side_ranges_right[i]
                break
            angleRight += message.angle_increment
        
        if len(angles) == 3:
            print('3')
            if close[0] < close[1]:
                angleSafe = np.arctan(0.05/close[0])
                angle = angles[1] + 0.9*angles[2]
                #angle += np.abs(angleSafe)*np.sign(angle)
            else:
                angleSafe = np.arctan(0.05/close[1])
                angle = 0.9*angles[1] + angles[2]
                #angle += np.abs(angleSafe)*np.sign(angle)

            if angle*angles[0]>0:
                self.obs = angles[0] + angle
            else:
                self.obs = angles[0]*0.9 + angle
            print(f"Final {self.obs}")
            return
        
        if len(angles) == 2 and angles[0] == angleFront:
            print('side w front')
            if angles[0]*angles[1]>0:
                self.obs = np.dot(angles, [1,1])
            else:
                self.obs = angles[0]*0.9 + angles[1]
            print(f"Final {self.obs}")
            return
        
        elif len(angles) == 2:
            print('2 sides')
            if close[0] < close[1]:
                angleSafe = np.arctan(0.05/close[0])
                self.obs = np.dot(angles, [1,0.9])
                #self.obs += np.abs(angleSafe)*np.sign(self.obs)
            else:
                angleSafe = np.arctan(0.05/close[1])
                self.obs = np.dot(angles, [0.9, 1]) 
                #self.obs += np.abs(angleSafe)*np.sign(self.obs)
            print(f"Final {self.obs}")
        
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
