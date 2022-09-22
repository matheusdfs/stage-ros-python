import rospy

from std_srvs.srv import Empty

# Node + Service_name = Function_name

def stageros_reset_world():
    execute_service('reset_positions', Empty)

def execute_service(service_name, type):
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, type)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%service_name)