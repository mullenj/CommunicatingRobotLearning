import numpy as np
import matplotlib.pyplot as plt

Q1m = np.asarray([5.95, 4.2, 6.4, 6.7])
Q1s = np.asarray([.98460, 2.11082, .69921, .4216])

Q2m = np.asarray([4.25, 6.55, 2.35, 6.9])
Q2s = np.asarray([1.82954, .59861, 1.17969, .21082])

Q3m = np.asarray([5.45, 5.35, 5.7, 6.55])
Q3s = np.asarray([1.48043, 1.00139, .82327, .55025])

Q4m = np.asarray([5.65, 5.3, 5.5, 6.2])
Q4s = np.asarray([1.35503, .88819, 1.31233, 1.20646])

Q5m = np.asarray([4.9, 5.6, 4.7, 6.7])
Q5s = np.asarray([1.59513, 1.34990, 1.49443, .48305])

Q1s /= np.sqrt(10)
Q2s /= np.sqrt(10)
Q3s /= np.sqrt(10)
Q4s /= np.sqrt(10)
Q5s /= np.sqrt(10)

Qm = np.asarray([Q1m, Q2m, Q3m, Q4m, Q5m])
Qs = np.asarray([Q1s, Q2s, Q3s, Q4s, Q5s])


X = np.asarray([1, 2, 3, 4, 5])
plt.bar(X + 0.00, Qm[:,0], yerr=Qs[:,0], color = 'b', width = 0.2)
plt.bar(X + 0.2, Qm[:,1], yerr=Qs[:,1], color = 'g', width = 0.2)
plt.bar(X + 0.4, Qm[:,2], yerr=Qs[:,2], color = 'r', width = 0.2)
plt.bar(X + 0.6, Qm[:,3], yerr=Qs[:,3], color = 'k', width = 0.2)
plt.show()
