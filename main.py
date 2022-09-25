import rospy
import numpy
import random
import threading

from controller import Controller
from services import stageros_reset_world

NUM_GENES = 4
NUM_INPUTS = 6

def initialize_random_array():
    return numpy.random.random(NUM_INPUTS)

def get_position_best_score(score_array):
    pos_bs = 0
    for i in range(NUM_GENES):
        if score_array[i]['score'] > score_array[pos_bs]['score']:
            pos_bs = i

    return pos_bs

if __name__ == '__main__':
    # TODO: try to use the ros functionality to increase the performance of the program

    # initialize the parameter array for 5 robots
    # for each parameter of a robot
        # execute each robot
        # evaluate the parameters
        # select, mutate and cross the best parameters

    robots_info = []

    # dict = {'name': '', 'score': 0, 'parameters': []}

    num_generation = 0

    for i in range(NUM_GENES):
        robots_info.append({'name': 'robot_' + str(i), 'score': -1, 'parameters': initialize_random_array()})

    try:
        rospy.init_node('default')
        while 1:
            print('Generation ' + str(num_generation))
            stageros_reset_world()
            workers = []
            for i in range(NUM_GENES):
                control = Controller(robots_info[i]) 
                x = threading.Thread(target=control.execute)
                x.start()
                workers.append(x)
                # control = Controller('robot_' + str(i), parameters_array[i])
                # score_array[i] = control.execute()

            for w in workers:
                w.join()

            bs = get_position_best_score(robots_info)
            print('best parent in this generation is: ' + str(bs))

            #select
            robots_info[0] = robots_info[bs]

            #mutate
            for i in range(NUM_GENES - 1):
                robots_info[i + 1]['score'] = -1
                robots_info[i + 1]['parameters'] = robots_info[0]['parameters']
                robots_info[i + 1]['parameters'][random.randint(0, NUM_INPUTS - 1)] = random.random()

            num_generation += 1
    except rospy.ROSInterruptException:
        pass
