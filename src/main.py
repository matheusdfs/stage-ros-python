import math
import rospy
import threading

from controller import Controller
from services import stageros_reset_world

import random
import numpy
import pyswarms.backend as P

from pyswarms.backend.topology import Star

NUM_SWARMS = 6
NUM_PARAMETERS = 3

def execute_robots(robots_info):
    stageros_reset_world()
    workers = []
    for i in range(NUM_SWARMS):
        control = Controller(robots_info[i]) 
        x = threading.Thread(target=control.execute)
        x.start()
        workers.append(x)

    for w in workers:
        w.join()

if __name__ == '__main__':
    #TODO: take the score off the controller
    #TODO: get the actual position of the red block in the world and put as objective

    robots_info = []

    current_scores = []
    best_scores = []

    for i in range(NUM_SWARMS):
        robots_info.append({'name': 'robot_' + str(i), 'score': -1, 'parameters': []})

    my_topology = Star()
    parameters_pso = {'c1': 0.6, 'c2': 0.3, 'w': 0.4}
    swarm = P.create_swarm(n_particles=NUM_SWARMS, dimensions=NUM_PARAMETERS, options=parameters_pso)

    iterations = 100
    try:
        rospy.init_node('default')
        
        for i in range(iterations):
            print('Iteration ' + str(i))
            # 1.update the parameters value in robots_info
            for j in range(NUM_SWARMS):
                robots_info[j]['parameters'] = swarm.position[j]

            # 2.execute the robots
            execute_robots(robots_info)

            # 3.update the swarms variables
            # 3.1 update the personal best
            current_scores = [d.get('score') for d in robots_info]

            if i == 0:
                best_scores = current_scores

            swarm.current_cost =  numpy.array(current_scores)
            swarm.pbest_cost = numpy.array(best_scores)
            swarm.pbest_pos, swarm.pbest_cost = P.compute_pbest(swarm) # Update and store

            # 3.2 update the global best
            if numpy.min(swarm.pbest_cost) < swarm.best_cost:
                swarm.best_pos, swarm.best_cost = my_topology.compute_gbest(swarm)

            # Part 3: Update position and velocity matrices
            # Note that position and velocity updates are dependent on your topology
            swarm.velocity = my_topology.compute_velocity(swarm)
            swarm.position = my_topology.compute_position(swarm)

    except rospy.ROSInterruptException:
        pass
