import time
import math
import rospy
import string

from math import floor
from utils import print_error, print_warning, print_ok

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

NUM_QUADRANTS = 4
TIMEOUT = 10

objective_position = [-18, 21]

class Controller:
    name: string = None
    parameters = []
    score = -1

    # variables odometry
    odom_updated = False
    odom_position: Pose = None
    odom_velocity: Twist = None

    # variables sensor
    sensor_updated = False
    base_scan_ranges = []
    # quadrant_x_lowest_value_element
    quadrant_one_lve = 0
    quadrant_two_lve = 0
    quadrant_three_lve = 0
    quadrant_four_lve = 0

    def __init__(self, name = None, parameters = None):
        # for multiple instances of this node use the anonymous parameter true
        # rospy.init_node('controller')

        # initialize robot variables
        if name == None:
            print_error('Name of robot is empty')
        elif name == '':
            self.name == name
        else:
            self.name = '/' + name

        if len(parameters) == 0:
            print_error('Parameters are empty')
        else:
            self.parameters = parameters

    def execute(self):
        rospy.Subscriber('/odom', Odometry, callback=self.update_odom)
        rospy.Subscriber('/base_scan', LaserScan, callback=self.update_base_scan)

        while not(self.odom_updated and self.sensor_updated):
            pass

        start_time = time.time()

        while not rospy.is_shutdown():
            # TODO replace the position of the robot and of the objective_position by the distance
            result =    (self.parameters[0] * self.odom_position.position.x + \
                        self.parameters[0] * self.odom_position.position.y + \
                        self.parameters[1] * objective_position[0] + \
                        self.parameters[1] * objective_position[1] + \
                        self.parameters[2] * self.quadrant_one_lve + \
                        self.parameters[3] * self.quadrant_two_lve + \
                        self.parameters[4] * self.quadrant_three_lve + \
                        self.parameters[5] * self.quadrant_four_lve) % 2

            pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            msg = Twist()
            msg.linear.x = 1
            msg.angular.z = result - 1

            pub.publish(msg)

            if (time.time() - start_time) > TIMEOUT:
                delta_x = self.odom_position.position.x - objective_position[0]
                delta_y = self.odom_position.position.y - objective_position[1]
                distance = math.sqrt(delta_x*delta_x + delta_y*delta_y)

                if distance > 10:
                    distance = 10

                self.score = -0.1 * (time.time() - start_time) + 10 - distance + 5
                if distance < 0.1:
                    print_ok('BOAAAAAAA FILHAO ' + str(self.score))
                else:
                    print_warning('timeout ' + str(self.score))
                break
        
        return self.score

    def update_odom(self, data):
        self.odom_position = data.pose.pose
        self.odom_velocity = data.twist.twist
        self.odom_updated = True

    def update_base_scan(self, data):
        self.base_scan_ranges = data.ranges
        quad_len = floor(len(self.base_scan_ranges)/NUM_QUADRANTS)
        self.quadrant_one_lve = min(self.base_scan_ranges   [0 : quad_len])
        self.quadrant_two_lve = min(self.base_scan_ranges   [quad_len : 2*quad_len])
        self.quadrant_three_lve = min(self.base_scan_ranges [2*quad_len : 3*quad_len])
        self.quadrant_four_lve = min(self.base_scan_ranges  [3*quad_len : 4*quad_len])
        self.sensor_updated = True
