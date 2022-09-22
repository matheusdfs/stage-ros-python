import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def callback(data):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
    rate = rospy.Rate(10) # 10hz

    msg = Twist()
    msg.linear.x = 1.0

    pub.publish(msg)
    

def launch():
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()


def reset_world():
    rospy.wait_for_service('reset_positions')
    try:
        add_two_ints = rospy.ServiceProxy('reset_positions', Empty)
        resp1 = add_two_ints()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        

if __name__ == '__main__':
    try:
        rospy.init_node('control', anonymous=True)
        reset_world()
        launch()
    except rospy.ROSInterruptException:
        pass