import rospy
from services import stageros_reset_world
from controller import Controller

if __name__ == '__main__':
    try:
        stageros_reset_world()
        control = Controller('')
    except rospy.ROSInterruptException:
        pass