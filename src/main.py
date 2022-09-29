import rospy
import numpy
import random
import threading

from controller import Controller
from services import stageros_reset_world

NUM_GENES = 6
NUM_INPUTS = 6

def initialize_random_array():
    return numpy.random.random(NUM_INPUTS)

def execute_robots():
    stageros_reset_world()
    workers = []
    for i in range(NUM_GENES):
        control = Controller(robots_info[i]) 
        x = threading.Thread(target=control.execute)
        x.start()
        workers.append(x)

    for w in workers:
        w.join()

if __name__ == '__main__':
    #TODO: implement PSO
    #TODO: take the score off the controller
    #TODO: add the direction vector as a parameter in the predict_function (review all the parameters in the predict_function)
    #TODO: get the actual position of the red block in the world and put as objective

    robots_info = []

    iteration_n = 0

    for i in range(NUM_GENES):
        robots_info.append({'name': 'robot_' + str(i), 'score': -1, 'parameters': initialize_random_array()})

    try:
        rospy.init_node('default')
        
        while 1:
            print('Iteration ' + str(iteration_n))
            execute_robots()
            iteration_n += 1
    except rospy.ROSInterruptException:
        pass
