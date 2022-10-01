import random
import numpy
import pyswarms.backend as P

from pyswarms.backend.topology import Star

# https://pyswarms.readthedocs.io/en/latest/examples/tutorials/custom_optimization_loop.html

def PSO():
    my_topology = Star()
    my_options = {'c1': 0.6, 'c2': 0.3, 'w': 0.4}
    my_swarm = P.create_swarm(n_particles=6, dimensions=3, options=my_options)

    print('The following are the attributes of our swarm: {}'.format(my_swarm.__dict__.keys()))

    iterations = 100
    for i in range(iterations):
        # Part 1: Update personal best
        print(my_swarm.position[0][1])
        my_swarm.current_cost = f(my_swarm.position) # Compute current cost
        my_swarm.pbest_cost = f(my_swarm.pbest_pos)  # Compute personal best pos
        my_swarm.pbest_pos, my_swarm.pbest_cost = P.compute_pbest(my_swarm) # Update and store

        # Part 2: Update global best
        # Note that gbest computation is dependent on your topology
        if numpy.min(my_swarm.pbest_cost) < my_swarm.best_cost:
            my_swarm.best_pos, my_swarm.best_cost = my_topology.compute_gbest(my_swarm)

        # Let's print our output
        if i%20==0:
            print('Iteration: {} | my_swarm.best_cost: {:.4f}'.format(i+1, my_swarm.best_cost))

        # Part 3: Update position and velocity matrices
        # Note that position and velocity updates are dependent on your topology
        my_swarm.velocity = my_topology.compute_velocity(my_swarm)
        my_swarm.position = my_topology.compute_position(my_swarm)

    print('The best cost found by our swarm is: {:.4f}'.format(my_swarm.best_cost))
    print('The best position found by our swarm is: {}'.format(my_swarm.best_pos))

def f(pos):
    return random.randint(0, 200)

if __name__ == '__main__':
    PSO()