import pickle
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)


# how much time the user was interacting with the joystick
def interaction_time(data):
    ah_time = 0
    for ah in data["ah"]:
        ah_norm = np.linalg.norm(ah)
        if ah_norm > 0.01:
            ah_time += 0.1
    return ah_time

# Shannon entropy over belief as a function of time
# i.e., how uncertain the robot about the human's goal as a function of time
def entropy(data):
    H = []
    for b in data["belief"]:
        H.append(-sum(b * np.log(b + 1e-6)))
    return np.asarray(H)[-1]

# robot's learned belief in true goal
def belief(data):
    bfinal = data["belief"][-1]
    return bfinal[3]

# amount of time the robot was in critical states
# my understanding is that the robot stops in these states? So we should want to minimize this?
def critical_alignment(data):
    C = np.copy(data["critical"])
    C /= max(C)
    time_critical = 0
    for c in C:
        if c > 0.8:
            time_critical += 0.1
    return time_critical


TASK = 4
METHOD = ["A", "B", "C", "D"]

results = []

for method in METHOD:

    results_method = []
    filename = "data/" + method + str(TASK) + ".pkl"
    with open(filename, 'rb') as handle:
        datafull = pickle.load(handle)
        for data in datafull:
            ah_time = interaction_time(data)
            final_entropy = entropy(data)
            belief_in_g3 = belief(data)
            time_critical = critical_alignment(data)
            results_method.append([ah_time, final_entropy, belief_in_g3, time_critical])

    results.append(results_method)

results = np.asarray(results)
print(results)
