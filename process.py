import pickle
import numpy as np


# takes the raw data and puts it into a dictionary
# each dictionary entry is a numpy array
def sort_data(data):
    D = {"time": [], "q": [], "s": [], "G": [], "ah": [], "astar": [],
         "ar": [], "belief": [], "critical": []}
    for datapoint in data:
        D["time"].append(datapoint[0])
        D["q"].append(list(datapoint[1]['q']))
        D["s"].append(list(datapoint[2]))
        D["G"].append(list(datapoint[3]))
        D["ah"].append(list(datapoint[4]))
        D["astar"].append(list(datapoint[5]))
        D["ar"].append(list(datapoint[6]))
        D["belief"].append(list(datapoint[7]))
        D["critical"].append(datapoint[8])
    for key in D:
        D[key] = np.asarray(D[key])
    D["time"] -= D["time"][0]
    return D


# specify which user studies you want to process the data for
USERS = [1, 2, 4, 5, 6, 7, 8, 9]
TASK = [1, 2, 3, 4, 5]
METHOD = ["A", "B", "C", "D"]

for task in TASK:
    for method in METHOD:
        savename = "data/" + method + str(task) + ".pkl"

        datafull = []
        for user in USERS:
            if task < 3:
                folder = "users/user" + str(user) + "/task" + str(task)
                filename1 = "data_method_" + method + ".pkl"
                filename2 = "game_method_" + method + ".pkl"
            elif task == 3:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename1 = "data_method_" + method + "_prior_A.pkl"
                filename2 = "game_method_" + method + "_prior_A.pkl"
            elif task == 4:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename1 = "data_method_" + method + "_prior_B.pkl"
                filename2 = "game_method_" + method + "_prior_B.pkl"
            elif task == 5:
                folder = "users/user" + str(user) + "/task" + str(3)
                filename1 = "data_method_" + method + "_prior_C.pkl"
                filename2 = "game_method_" + method + "_prior_C.pkl"
            D = sort_data(pickle.load(open(folder + "/" + filename1, "rb")))
            D["score"] = pickle.load(open(folder + "/" + filename2, "rb"))
            D["name"] = folder + "/" + filename1
            D["task"] = task
            datafull.append(D)

        with open(savename, 'wb') as handle:
            pickle.dump(datafull, handle, protocol=pickle.HIGHEST_PROTOCOL)
