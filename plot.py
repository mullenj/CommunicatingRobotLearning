import pickle
import numpy as np
import sys
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

# score normalized by time on the distractor game
def score(data):
    return data["score"]

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
    if data["task"] == 1:
        return bfinal[5]
    if data["task"] == 2:
        return bfinal[2]
    if data["task"] > 2:
        return bfinal[3]

# how efficiently the human taught the robot their desired goal
def efficiency(data):
    return belief(data) / interaction_time(data)

# cumulative measure of criticality
def criticality(data):
    C = np.copy(data["critical"])
    return sum(C)

# total time spent on the task
def total_time(data):
    return data["time"][-1]


TASK = int(sys.argv[1])
METHOD = ["A", "B", "C", "D"]

results = []

for method in METHOD:

    results_method = []
    filename = "data/" + method + str(TASK) + ".pkl"
    with open(filename, 'rb') as handle:
        datafull = pickle.load(handle)
        for data in datafull:
            m1 = interaction_time(data)
            m2 = score(data)
            # m3 = entropy(data)
            # m4 = belief(data)
            m5 = efficiency(data)
            m6 = criticality(data)
            # m7 = total_time(data)
            results_method.append([m1, m2, m5, m6])
        results_method = np.asarray(results_method)
    # results.append(np.mean(results_method, axis=0))       # averages results
    results.append(results_method)                        # individual results

results = np.asarray(results)
print("Rows are A, B, C, D")
print("Columns are interaction time, score, efficiency, total criticality")
print(results)
