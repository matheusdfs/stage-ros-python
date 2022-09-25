import rospy
import numpy
import random
import threading

from controller import Controller
from services import stageros_reset_world

NUM_GENES = 4
NUM_INPUTS = 5

def initialize_random_array():
    return numpy.random.random(NUM_INPUTS)

def get_position_best_score(score_array):
    pos_bs = 0
    for i in range(NUM_GENES):
        if score_array[i] > score_array[pos_bs]:
            pos_bs = i

    return pos_bs

if __name__ == '__main__':
    # initialize the parameter array for 5 robots
    # for each parameter of a robot
        # execute each robot
        # evaluate the parameters
        # select, mutate and cross the best parameters

    score_array = []
    parameters_array = []

    num_generation = 0

    for i in range(NUM_GENES):
        score_array.append(-1)
        parameters_array.append(initialize_random_array())

    try:
        rospy.init_node('default')
        while 1:
            print('Generation ' + str(num_generation))
            stageros_reset_world()
            workers = []
            for i in range(NUM_GENES):
                control = Controller('robot_' + str(i), parameters_array[i]) 
                x = threading.Thread(target=control.execute)
                x.start()
                workers.append(x)
                # control = Controller('robot_' + str(i), parameters_array[i])
                # score_array[i] = control.execute()

            for w in workers:
                w.join()

            bs = get_position_best_score(score_array)
            print('best parent in this generation is: ' + str(bs))

            #select
            parameters_array[0] = parameters_array[bs]

            #mutate
            for i in range(NUM_GENES - 1):
                parameters_array[i + 1] = parameters_array[0]
                parameters_array[i + 1][random.randint(0, NUM_INPUTS - 1)] = random.random()

            num_generation += 1
    except rospy.ROSInterruptException:
        pass
