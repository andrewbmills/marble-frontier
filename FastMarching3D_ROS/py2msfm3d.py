import numpy as np
import ctypes
import scipy.io

if __name__ == '__main__':
	esdf_3d = scipy.io.loadmat('esdf_3d_example.mat')['esdf_3d']
	F = np.squeeze(esdf_3d[:,:,:,3])
	T = np.zeros(np.size(F), dtype=np.double)
	Y = T
	SourcePoints = np.array([59, 41, 30])
	usesecond = True
	usecross = True

	lib = ctypes.cdll.LoadLibrary('./msfm3d_test.so')
	fun = lib.pyFunction

	# Here comes the fool part.
	fun(ctypes.c_void_p(T.ctypes.data), ctypes.c_void_p(Y.ctypes.data),
		ctypes.c_void_p(F.ctypes.data), ctypes.c_void_p(SourcePoints.ctypes.data),
	    ctypes.c_bool(usesecond), ctypes.c_bool(usecross))

	print('F[0:9]:', F[0:9])
	print('T[0:9]:', T[0:9])