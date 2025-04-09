from utils.taylor import constrained_taylor_fit
import numpy as np
import matplotlib.pyplot as plt

"""
x: normalized thruster input
y: stable velocity can be normalized or anyother unit
"""
x = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0])
y = np.array([0.0, 0.0, 0.0, 2.7, 6.0, 7.5, 10.0, 21.8])
y = (y - np.min(y)) / (np.max(y) - np.min(y))  # Normalize target to [0,1]

coeffs, f_tuned = constrained_taylor_fit(x, y, degree=3, x0=0.0)

import matplotlib.pyplot as plt
x_plot = np.linspace(min(x), max(x), 200)
plt.plot(x, y, 'o', label='Data')
plt.plot(x_plot, f_tuned(x_plot), label='Constrained Fit')
plt.legend()
plt.grid(True)
plt.title("Constrained Taylor Fit (Output ∈ [0, 1])")
plt.show()