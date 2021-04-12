import pickle
import numpy as np


# takes the raw data and puts it into a dictionary
# each dictionary entry is a numpy array
def sort_data(data):
    D = {"time": [], "q": [], "s": [], "G": [], "ah": [], "astar": [],
         "ar": [], "belief": []}
    for datapoint in data:
        D["time"].append(datapoint[0])
        D["q"].append(list(datapoint[1]['q']))
        D["s"].append(list(datapoint[2]))
        D["G"].append(list(datapoint[3]))
        D["ah"].append(list(datapoint[4]))
        D["astar"].append(list(datapoint[5]))
        D["ar"].append(list(datapoint[6]))
        D["belief"].append(list(datapoint[7]))
    for key in D:
        D[key] = np.asarray(D[key])
    D["time"] -= D["time"][0]
    D["critical"] = get_criticality(D)
    return D

# helper function to recompute the critical states,
# should match what was in the code for the user study
def get_criticality(data):
    C = []
    for idx in range(len(data["time"])):
        belief = data["belief"][idx, :]
        a_star = data["astar"][idx]
        G = data["G"][idx]
        ar = data["ar"][idx, :]
        s = data["s"][idx, :]
        C.append(sum([b*(np.linalg.norm(s+ar - goal_x) - np.linalg.norm(s+a_star_x - goal_x)) for b, a_star_x, goal_x in zip(belief, a_star, G)]))
    return np.asarray(C)


USERS = [1, 2, 3, 4, 5]
TASK = [1, 2, 3, 4, 5]
METHOD = ["A", "B", "C", "D"]

for task in TASK:

    for method in METHOD:

        savename = "data/" + method + str(task) + ".pkl"

        datafull = []
        for user in USERS:
            if task < 3:
                folder = "users/user" + str(user) + "/task" + str(task)
                filename = "data_method_" + method + ".pkl"
            elif task == 3:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename = "data_method_" + method + "_prior_A.pkl"
            elif task == 4:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename = "data_method_" + method + "_prior_B.pkl"
            elif task == 5:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename = "data_method_" + method + "_prior_C.pkl"
            trial = pickle.load(open(folder + "/" + filename, "rb"))
            D = sort_data(trial)
            D["name"] = folder + "/" + filename
            datafull.append(D)

        with open(savename, 'wb') as handle:
            pickle.dump(datafull, handle, protocol=pickle.HIGHEST_PROTOCOL)
