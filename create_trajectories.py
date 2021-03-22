import numpy as np
import pickle

home = np.asarray([0.000297636, -0.785294, -0.000218009, -2.3567, 0.000397658, 1.57042, 0.785205])
state1 = np.asarray([0.049331, -0.200572, 0.159384, -2.31902, 0.0428248, 2.11483, 0.972073])
end = np.asarray([0.221363, 1.30969, 0.149847, -0.654274, -0.147172, 1.978, 1.10165])

path1_state1 = np.asarray([0.085859, 0.288162, 0.180373, -1.99921, -0.0574394, 2.28317, 1.08402])
path1_state2 = np.asarray([0.141547, 0.623617, 0.1571, -1.6992, -0.114225, 2.31175, 1.13382])
path2_state1 = np.asarray([0.136338, 0.26118, 0.157175, -1.80338, -0.00288155, 2.07467, 0.987893])
path2_state2 = np.asarray([0.171147, 0.780275, 0.156599, -1.29129, -0.0766674, 2.09306, 0.961107])
path3_state1 = np.asarray([-0.060401, 0.0571834, -0.0253441, -2.15802, -1.483e-05, 2.23061, 0.643051])
path3_state2 = np.asarray([0.0901471, 0.780731, -0.00383062, -1.3454, -0.000667255, 2.15011, 0.86205])
path4_state1 = np.asarray([0.511219, 0.168996, 0.0393572, -1.97411, -0.0012093, 2.14818, 1.33116])
path4_state2 = np.asarray([0.422457, 0.807449, 0.012779, -1.28046, -0.002805, 2.11632, 1.21366])


path1 = np.asarray([home, state1, path1_state1, path1_state2, end])
path2 = np.asarray([home, state1, path2_state1, path2_state2, end])
path3 = np.asarray([home, state1, path3_state1, path3_state2, end])
path4 = np.asarray([home, state1, path4_state1, path4_state2, end])

paths = [path1, path2, path3, path4]

for num, path in enumerate(paths):
    pickle.dump(path, open(f"task2_path{num}.pkl", "wb"))
