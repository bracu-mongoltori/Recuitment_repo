import time
import numpy as np

def benchmark(grid, start, goal, algo_func, runs=5, **kwargs):
    times = []
    lengths = []
    successes = 0

    for _ in range(runs):
        start_time = time.time()
        path, _ = algo_func(grid, start, goal, **kwargs)
        elapsed = (time.time() - start_time)*1000  
        times.append(elapsed)
        if path:
            lengths.append(len(path))
            successes += 1
        else:
            lengths.append(0)

    return {
        "avg_time_ms": round(np.mean(times), 2),
        "avg_length": round(np.mean([l for l in lengths if l>0]),2) if successes > 0 else 0,
        "success_rate": round((successes / runs)*100, 2)
    }
