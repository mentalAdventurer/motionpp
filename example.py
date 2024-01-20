import matplotlib.pyplot as plt
import ctypes
from ctypes import POINTER, c_double

# Load the shared library
simulator = ctypes.CDLL('./lib/libsimulator.so')

# Define the return type and argument types for simulate
simulator.simulate.restype = POINTER(c_double)
simulator.simulate.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), ctypes.c_int]

# Number of time steps
n = 1000

# Initial conditions: [q, q']
# For example, starting at position 0 and velocity 0
x = (c_double * 2)(0.0, 0.0) 

# Input: Assuming constant input, e.g., u = 0 for all time steps
# Change this if your input varies over time
u = (c_double * n)(*[1.0]*n) 

# Time array: Assuming equally spaced time points
t_final = 10.0  # Example final time
t_step = t_final / n
t = (c_double * n)(*[(i * t_step) for i in range(n)])

# Call the function
result_ptr = simulator.simulate(u, x, t, n)


# Convert the result to a Python list
result = [result_ptr[i] for i in range(n)]

# Don't forget to free the memory allocated in C++
simulator.free_memory(result_ptr)

# create time array for plotting resutls
plt.plot(result[::2], label='Position')
plt.plot(result[1::2], label='Velocity')
plt.show()
