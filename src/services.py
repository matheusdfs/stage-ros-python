import rospy

from std_srvs.srv import Empty

# Node + Service_name = Function_name

def stageros_reset_world():
    execute_service('reset_positions', Empty)

def execute_service(service_name, type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, type)
        service()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%service_name)