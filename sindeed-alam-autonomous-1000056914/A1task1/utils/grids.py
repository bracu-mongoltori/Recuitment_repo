import numpy as np
import json

def load_g(f_paths):
    return np.loadtxt(f_paths,delimiter=',',dtype=int)

def load_config(path='config.json'):
    with open(path)as f:
        config=json.load(f)
        return tuple(config['start']),tuple(config['goal'])
    
    