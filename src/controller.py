import time
import math

import numpy
import rospy
import string

from math import floor
from utils import print_error, print_warning, print_ok

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

NUM_QUADRANTS = 5
TIMEOUT = 1000

objective_position = [0.0, 0.0]

class Controller:
    info = None

    name: string = None
    parameters = []
    score = -1

    # variables odometry
    odom_updated = False
    odom_position: Pose = None
    odom_velocity: Twist = None

    last_odom_position: Pose = None

    # variables sensor
    sensor_updated = False
    base_scan_ranges = []
    # quadrant_x_lowest_value_element
    quadrant_one_lve = 0
    quadrant_two_lve = 0
    quadrant_three_lve = 0
    quadrant_four_lve = 0
    quadrant_five_lve = 0

    def __init__(self, info):
        # initialize robot variables
        if info['name'] == None:
            print_error('Name of robot is empty')
        elif info['name'] == '':
            self.name == info['name']
        else:
            self.name = '/' + info['name']

        if len(info['parameters']) == 0:
            print_error('Parameters are empty')
        else:
            self.parameters = info['parameters']

        self.info = info

    def execute(self):
        self.prepare_execute()

        start_time = time.time()

        rate = rospy.Rate(1)

        started_movings = False

        while not rospy.is_shutdown():
            distance_from_objective = math.dist([self.odom_position.position.x, self.odom_position.position.y],
                        [objective_position[0], objective_position[1]])

            delta_position = math.dist([self.odom_position.position.x, self.odom_position.position.y],
                        [self.last_odom_position.position.x, self.last_odom_position.position.y]) \
                            if self.last_odom_position != None else 0.0

            if delta_position > 0.0:
                started_movings = True

            pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

            msg = Twist()
            msg.linear.x = 1
            msg.angular.z = self.predict_function(distance_from_objective) - 1

            pub.publish(msg)

            delta_time = time.time() - start_time

            # (started_movings and delta_position == 0.0)
            if delta_time > 10:

                self.score = self.fitness_function(delta_time, distance_from_objective)

                if distance_from_objective < 0.1:
                    print_ok('BOAAAAAAA FILHAO ' + str(self.score))
                    break
                else:
                    print_warning('timeout ' + str(self.score))
                break

            rate.sleep()
        
        self.info['score'] = self.score

    def prepare_execute(self):
        # declare subscribers
        rospy.Subscriber(self.name + '/odom', Odometry, callback=self.update_odom)
        rospy.Subscriber(self.name + '/base_scan', LaserScan, callback=self.update_base_scan)

        while not(self.odom_updated and self.sensor_updated):
            pass

    def predict_function(self, distance_from_objective):
        vector_direction = numpy.subtract([objective_position[0], objective_position[1]], [self.odom_position.position.x, self.odom_position.position.y])
        return (self.parameters[0] * distance_from_objective + \
                self.parameters[1] * vector_direction[0] + \
                self.parameters[1] * vector_direction[1] + \
                self.parameters[2] * self.quadrant_three_lve) % 2

    def fitness_function(self, delta_time, distance_from_objective):
        return -0.1 * delta_time + 10 - distance_from_objective + 50

    def update_odom(self, data):
        self.last_odom_position = self.odom_position
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
        self.quadrant_five_lve = min(self.base_scan_ranges  [4*quad_len : 5*quad_len])
        self.sensor_updated = True
