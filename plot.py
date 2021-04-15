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


TASK = [1, 2, 3, 4, 5]
METHOD = ["A", "B", "C", "D"]

results_all_mean = []
results_all_std = []

for task in TASK:

    results_mean = []
    results_std = []
    N_users = None

    for method in METHOD:

        results_method = []
        filename = "data/" + method + str(task) + ".pkl"
        with open(filename, 'rb') as handle:
            datafull = pickle.load(handle)
            N_users = len(datafull)
            for data in datafull:
                m1 = interaction_time(data)
                m2 = efficiency(data)
                m3 = score(data)
                # m3 = entropy(data)
                # m4 = belief(data)
                # m6 = criticality(data)
                # m7 = total_time(data)
                results_method.append([m1, m2, m3])
            results_method = np.asarray(results_method)
        results_mean.append(np.mean(results_method, axis=0))       # averages results
        results_std.append(np.std(results_method, axis=0))       # averages results

    results_mean = np.asarray(results_mean)
    results_std = np.asarray(results_std) / np.sqrt(N_users)
    results_all_mean.append(results_mean)
    results_all_std.append(results_std)
    print(results_mean)


print("Number of users: ", N_users)
x = [1, 2, 3, 4]
for task in range(5):
    plt.suptitle("Interaction Time", fontsize=14)
    ax = plt.subplot(151 + task)
    ax.bar(x, results_all_mean[task][:,0], yerr=results_all_std[task][:,0])
plt.show()

for task in range(5):
    plt.suptitle("Teaching Efficiency", fontsize=14)
    ax = plt.subplot(151 + task)
    ax.bar(x, results_all_mean[task][:,1], yerr=results_all_std[task][:,1])
plt.show()

for task in range(5):
    plt.suptitle("Distractor Score", fontsize=14)
    ax = plt.subplot(151 + task)
    ax.bar(x, results_all_mean[task][:,2], yerr=results_all_std[task][:,2])
plt.show()
