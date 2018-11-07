import numpy as np
import ctypes

indata = np.ones((5,6), dtype=np.double)
outdata = np.zeros((5,6), dtype=np.double)
lib = ctypes.cdll.LoadLibrary('./ctest.so')
fun = lib.cfun
# Here comes the fool part.
fun(ctypes.c_void_p(indata.ctypes.data), ctypes.c_int(5), ctypes.c_int(6),
    ctypes.c_void_p(outdata.ctypes.data))

print('indata:', indata)
print('outdata:', outdata)