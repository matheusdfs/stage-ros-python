import rospy
import string

from math import floor
from utils import print_error

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

NUM_QUADRANTS = 4

class Controller:
    name: string = None

    # variables odometry
    odom_position: Pose = None
    odom_velocity: Twist = None

    # variables sensor
    base_scan_ranges = []
    # quadrant_x_lowest_value_element
    quadrant_one_lve = 0
    quadrant_two_lve = 0
    quadrant_three_lve = 0
    quadrant_four_lve = 0

    def __init__(self, name = None):
        # for multiple instances of this node use the anonymous parameter true
        rospy.init_node('controller')

        # initialize robot variables
        if name == None:
            print_error('Name of robot is empty')
        elif name == '':
            self.name == name
        else:
            self.name = '/' + name

        self.execute()

    def execute(self):
        rospy.Subscriber('/odom', Odometry, callback=self.update_odom)
        rospy.Subscriber('/base_scan', LaserScan, callback=self.update_base_scan)

        while not rospy.is_shutdown():
            pass

    def update_odom(self, data):
        self.odom_position = data.pose.pose
        self.odom_velocity = data.twist.twist

    def update_base_scan(self, data):
        self.base_scan_ranges = data.ranges
        quad_len = floor(len(self.base_scan_ranges)/NUM_QUADRANTS)
        self.quadrant_one_lve = min(self.base_scan_ranges   [0 : quad_len])
        self.quadrant_two_lve = min(self.base_scan_ranges   [quad_len : 2*quad_len])
        self.quadrant_three_lve = min(self.base_scan_ranges [2*quad_len : 3*quad_len])
        self.quadrant_four_lve = min(self.base_scan_ranges  [3*quad_len : 4*quad_len])
