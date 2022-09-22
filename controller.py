from tokenize import String
import rospy

from utils import print_error

from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan

class Controller:
    name: String = None
    odom_data: Odometry = None
    base_scan_data: LaserScan = None

    def __init__(self, name = None):
        # for multiple instances of this node use the anonymous parameter true
        rospy.init_node('controller')

        # initialize robot variables
        if name == None:
            print_error('Name of robot is empty')
        else:
            self.name = '/' + name

        self.execute()

    def execute(self):
        odom_data = rospy.Subscriber(self.name + '/odom', Odometry)
        base_scan_data = rospy.Subscriber(self.name + '/base_scan', LaserScan)
        pass