import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from services import stageros_reset_world
from controller import Controller

if __name__ == '__main__':
    try:
        stageros_reset_world()
        control = Controller('robot_0')
    except rospy.ROSInterruptException:
        pass