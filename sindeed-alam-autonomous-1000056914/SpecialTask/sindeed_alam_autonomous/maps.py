import numpy as np

def get_waypoints():
    return np.array([
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1],
        [0, 0, 1, 1, 1],
        [1, 1, 1, 0, 1]
    ])

def get_start_goal():
    return (0, 0), (4, 4)
