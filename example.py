import ctypes

# Load the shared library
simulator = ctypes.CDLL('./lib/libsimulator.so')

# Specify the return type of the calcSqrt function
simulator.calcSqrt.restype = ctypes.c_double

# Call the function
result = simulator.calcSqrt(ctypes.c_double(100.0)) 
print(result)
