#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import numpy as np
import rospy

M_ORDER2 = np.array([[1, 0, 0], [-2, 2, 0], [1, -2, 1]])
M_ORDER3 = np.array([[1., 0., 0., 0.], [-3., 3., 0., 0.], [3., -6., 3., 0.], [-1., 3., -3., 1.]])
M_ORDER3_inv = np.array([[1., 0., 0., 0.], [1., 1./3., 0., 0.], [1., 2./3., 1./3., 0.], [1., 1., 1., 1.]])
M_ORDER4 = np.array([[1, 0, 0, 0, 0], [-4, 4, 0, 0, 0], [6, -12, 6, 0, 0], [-4, 12, -12, 4, 0], [1, -4, 6, -4, 1]])

def fitCurveCubic(points):
	# Fits a cubic bezier curve to a set of points (num_points x dimension) with (assumed) equal parameter (t) spacing between them
	shape = np.shape(points)
	dimension = shape[1]
	t = np.linspace(0.0, 1.0, points.shape[0])
	T = np.transpose(np.stack((np.ones(np.shape(t)), t, np.power(t,2), np.power(t,3))))
	TT_inv = np.linalg.inv(np.dot(T.transpose(), T))
	Mat_covariance = np.dot(np.dot(M_ORDER3_inv, TT_inv), T.transpose())
	controls = np.dot(Mat_covariance, points)
	return controls

def getBezier(points, t):
	# Uses de Casteljau's Algorithm to compute the bezier curve value at t given the control points
	if points.shape[0] == 1:
		return points
	else:
		newpoints = np.zeros((points.shape[0]-1, points.shape[1]))
		for i in range(points.shape[0]-1):
			for j in range(points.shape[1]):
				newpoints[i,j] = (1.0-t)*points[i,j] + t*points[i+1,j]
		return getBezier(newpoints, t)

def getDerivative(points):
	# Calculates the control points of the bezier curve derivative w.r.t. the parameter t. (dB(t)/dt)
	shape = np.shape(points)
	order = shape[0] - 1
	points_derivative = np.zeros((order,shape[1]))
	for i in range(0,order):
		points_derivative[i,:] = order*(points[i+1,:] - points[i,:])
	return points_derivative


def getCurve(points, N = 50):
	# Evaluates the curve location at N parametrically spaced (t-space) points along the curve.
	t = np.append(np.arange(0.0, 1.0, 1./(N-1)), 1.0)
	curve = np.zeros((t.size,points.shape[1]))
	for i in range(t.size):
		curve[i,:] = getBezier(points,t[i])
	return curve

def projectPointOntoCurve(points, p, tolerance= 0.001):
	# Returns the parameter, t, of the closest location on the bezier curve to the query point, p.
	diff = 1.0
	interval = 1.0
	t = 0.5
	d_old = 0.0
	while (diff > tolerance):
		b_low = getBezier(points, max(t - (interval/2.), 0.0))
		b = getBezier(points, t)
		b_high = getBezier(points, min(t + (interval/2.), 1.0))
		d1 = np.linalg.norm(p-b_low)
		d = np.linalg.norm(p-b)
		d2 = np.linalg.norm(p-b_high)
		if ((d2 < d) and (d2 < d1)):
			t = t + (interval/2.)
		if ((d1 < d) and (d1 < d2)):
			t = t - (interval/2.)
		else:
			interval = interval/2.
		diff = np.abs(d1+d2 - 2*d)

	return t

def getCurvature2D(points, t):
	# Returns the curvature at the point B(t) on the bezier curve
	# This function only works in 2D at the moment 
	d = getDerivative(points) # control points of derivative of bezier curve
	dd = getDerivative(d) # control points of second derivative of bezier curve
	d_xy = getBezier(d, t)
	dd_xy = getBezier(dd, t)
	numerator = d_xy[0,0]*dd_xy[0,1] - dd_xy[0,0]*d_xy[0,1]
	denominator = (d_xy[0,0]*d_xy[0,0] + d_xy[0,1]*d_xy[0,1])**1.5
	return numerator/denominator

def getCurvature3D(points, t):
	# Returns the curvature at the point B(t) on the bezier curve
	# Works in 3 dimensions as long as first/second derivative is defined
	d = getDerivative(points) # control points of derivative of bezier curve
	dd = getDerivative(d) # control points of second derivative of bezier curve
	numerator = np.linalg.norm(np.cross(getBezier(d, t), getBezier(dd, t)))
	denominator = np.linalg.norm(getBezier(d, t))**3.0
	return numerator/denominator

def getTangent(points, t):
	# Returns a normalized tangent vector at the point B(t)
	d = getDerivative(points)
	d_vec = np.squeeze(getBezier(d, t))
	return d_vec/(np.linalg.norm(d_vec))

def getNormalFrenet(points, t):
	# Returns the frenet normal vector at the point B(t)
	dim = np.size(points,1)
	a = getTangent(points, t)
	a = a/np.linalg.norm(a)
	d = getDerivative(points)
	dd = getDerivative(d)
	b = a + np.squeeze(getBezier(dd, t))
	b = b/np.linalg.norm(b)

	# Check if vectors are 2D or 3D
	if (dim < 3):
		a = [a[0], a[1], 0.0]
		b = [b[0], b[1], 0.0]

	r = np.cross(a, b) # The convention is actually the opposite but I prefer this
	r = r/np.linalg.norm(r)
	n = np.cross(r, a)

	if (dim < 3):
		n = n[0:2]

	return n/np.linalg.norm(n)

if __name__ == '__main__':
	p = np.array([[0.0, 0.0], [1.0, 2.0], [2.0, 3.0], [2.0, 4.5], [3.5, 5.0], [4.0, 10.0], [5.2, 8.8], [7.0, 10.0]])
	control_points = fitCurveCubic(p)
	print(control_points)
	print(getDerivative(control_points))
	curve_fit = getCurve(control_points, 30)
	# Project point 3 onto the curve
	query = np.array([6, 8])
	# query = p[4,:]
	projected1 = getBezier(control_points, projectPointOntoCurve(control_points, query, 0.001))
	print(projected1)
	k = getCurvature2D(control_points, projectPointOntoCurve(control_points, query, 0.001))
	t_vec = getTangent(control_points, projectPointOntoCurve(control_points, query, 0.001))
	n_vec = getNormalFrenet(control_points, projectPointOntoCurve(control_points, query, 0.001))
	c = np.squeeze(projected1) + n_vec*(1/np.abs(k))
	theta = (np.pi/180)*np.arange(0,360)
	print(t_vec)
	print(n_vec)

	plt.figure()
	plt.plot(p[:,0], p[:,1], 's-', label='data')
	plt.plot(control_points[:,0], control_points[:,1], '*', label='control points')
	plt.plot(curve_fit[:,0], curve_fit[:,1], label='fit')
	plt.plot([query[0],  projected1[0,0]], [query[1], projected1[0,1]], 's--', label='projection_fit_point3')
	plt.plot([projected1[0,0] + t_vec[0],  projected1[0,0]], [projected1[0,1] + t_vec[1], projected1[0,1]], '--', label='tangent')
	plt.plot([projected1[0,0] + n_vec[0],  projected1[0,0]], [projected1[0,1] + n_vec[1], projected1[0,1]], '--', label='normal')
	plt.plot(c[0] + (1/k)*np.cos(theta), c[1] + (1/k)*np.sin(theta), '--')
	plt.axis('equal')
	plt.show()