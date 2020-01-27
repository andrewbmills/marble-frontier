import os
import time
import string
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

def angle_Diff(a, b):
    # Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    # All angles are in degrees
    a = (360000 + a) % 360
    b = (360000 + b) % 360
    d = a - b
    d = (d + 180) % 360 - 180
    return d

def L2_Plus_Guidance_2D(p_L2, p_AC, v_AC, Tstar, phi_max):
    # print(p_L2)
    # print(p_AC)
    # print(v_AC)
    Vg = np.linalg.norm(v_AC)  # ground speed (m/s)
    L2_vec = p_L2 - p_AC  # vector from current position to lookahead point
    L2 = np.linalg.norm(L2_vec)  # distance to lookahead point
    
    # Compute the sine of the angle between the current heading and lookahead vector
    cross_prod = np.cross(L2_vec, v_AC)
    sin_eta = np.sign(cross_prod)*np.linalg.norm(cross_prod)/(Vg*L2)
    a_cmd = 2*Vg*sin_eta/Tstar  # commanded acceleration (m/s^2)
    return a_cmd

def L2_Plus_Guidance_3D(p_L2, p_AC, v_AC, Tstar, phi_max):
    Vg = np.linalg.norm(v_AC)  # ground speed (m/s)
    L2_vec = p_L2 - p_AC  # vector from current position to lookahead point
    L2 = np.linalg.norm(L2_vec)  # distance to lookahead point
    
    # Compute the sine of the angle between the current heading and lookahead vector
    cross_prod1 = np.cross(L2_vec, v_AC)
    cross_prod2 = np.cross(v_AC, cross_prod1)
    sin_eta = np.linalg.norm(cross_prod1)/(Vg*L2)
    if (abs(sin_eta) > 1.0e-9):
        a_hat = cross_prod2/np.linalg.norm(cross_prod2)
        a_cmd = (2*Vg*sin_eta/Tstar) * a_hat  # commanded acceleration vector (m/s^2)
    else:
        a_cmd = np.array([0., 0., 0.])
    return a_cmd

def trajectory_Shaping_Guidance(p_L2, p_AC, v_AC, v_path):
    Vg = np.linalg.norm(v_AC)  # Ground speed (m/s)
    L2_vec = p_L2 - p_AC  # Vector from current position to lookahead point
    L2 = np.linalg.norm(L2_vec)  # Distance to lookahead point
    t_go = L2/Vg  # Time until lookahead point is reached at current speed
    
    # Compute relevant angles for trajectory shaping guidance
    theta = math.atan2(L2_vec[1], L2_vec[0])*180/math.pi  # lookahead vector angle
    alpha_a = math.atan2(v_AC[1], v_AC[0])*180/math.pi  # heading angle
    alpha_t = math.atan2(v_path[1], v_path[0])*180/math.pi  # heading at lookahead point
    
    # Calculate commanded acceleration
    a_cmd = ((Vg/t_go)*(4*angle_Diff(alpha_a, theta) + 
            2*angle_Diff(alpha_t, theta))*math.pi/180)
    return a_cmd

def find_Lookahead_Discrete_2D(path, p_AC, R, gamma_max, Mstar):
    # Determine the last path index
    i_max = path.shape
    i_max = i_max[1] - 1
    # Loop initialization params
    intersection = True
    t_hat = -1
    i = 0
    i_closest = 0
    c_closest = 100.0*R

    while (t_hat < 0) or (t_hat > 1):
        # Break loop when you reach the last path index
        if (i == i_max):
            intersection = False
            break

        # Initialize Vectors in intersection geometry
        p1 = np.array([path[0,i], path[1,i]])
        p2 = np.array([path[0,i+1], path[1,i+1]])
        d = p2 - p1
        q = p1 - p_AC
        
        # Calculate coefficients for polynomial to solve for t_hat
        # Solve quadratic polynomial equation: a*t_hat^2 + b*t_hat + c = 0
        a = d[0]*d[0] + d[1]*d[1]
        b = 2*(d[0]*q[0] + d[1]*q[1])
        c = q[0]*q[0] + q[1]*q[1] - R*R
        discriminant = b*b - 4*a*c
        if (discriminant <= 0):
            # no intersection
            t_hat = -1
        else:
            # 1 or 2 solutions exist (numerically has to be 2)
            t_hat1 = (-b + math.sqrt(discriminant)) / (2*a)
            t_hat2 = (-b - math.sqrt(discriminant)) / (2*a)

            # Take larger of two intersections, furthest point down the path
            t_hat = max(t_hat1, t_hat2)
        # If p1 is the closest point to p_AC, store it
        if (c < c_closest):
            c_closest = c
            i_closest = i
        # Next iteration
        i = i + 1

    # Check for intersection of circle around p_AC with path
    if intersection:
        # Calculate lookahead point and its local tangent vector
        p_L2 = d*t_hat + p1
        v_L2 = d / np.linalg.norm(d)
        i_cut = i
    else:
        # Calculate lookahead point and its local tangent vector
        # Set lookahead point to the closest path node
        p_L2 = np.array([path[0,i_closest], path[1,i_closest]])
        v_L2 = np.array([path[0,i_closest+1], path[1,i_closest+1]]) - p_L2
        v_L2 = v_L2 / np.linalg.norm(v_L2)  # Normalize the vector
        i_cut = i_closest
    
    return p_L2, v_L2

def find_Lookahead_Discrete_3D(path, p_AC, R, gamma_max, Mstar):
    i = 0
    i_closest = 0
    c_closest = 100.0*R
    # Determine the last path index
    i_max = path.shape
    i_max = i_max[1] - 1
    
    intersection = True
    t_hat = -1
    while (t_hat < 0) or (t_hat > 1):
        # Break loop when you reach the last path index
        if (i == i_max):
            intersection = False
            break

        # Initialize Vectors in intersection geometry
        p1 = np.array([path[0,i], path[1,i], path[2,i]])
        p2 = np.array([path[0,i+1], path[1,i+1], path[2,i+1]])
        d = p2 - p1
        q = p1 - p_AC
        
        # Calculate coefficients for polynomial to solve for t_hat
        # Solve quadratic polynomial equation: a*t_hat^2 + b*t_hat + c = 0
        a = d[0]*d[0] + d[1]*d[1] + d[2]*d[2]
        b = 2*(d[0]*q[0] + d[1]*q[1] + d[2]*q[2])
        c = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] - R*R
        discriminant = b*b - 4*a*c
        if (discriminant <= 0):
            # no intersection
            t_hat = -1
        else:
            # 1 or 2 solutions exist (numerically has to be 2)
            t_hat1 = (-b + math.sqrt(discriminant)) / (2*a)
            t_hat2 = (-b - math.sqrt(discriminant)) / (2*a)

            # Take larger of two intersections, furthest point down the path
            t_hat = max(t_hat1, t_hat2)

        # If p1 is the closest point to p_AC, store it
        if (c <= c_closest):
            c_closest = c
            i_closest = i

        # Next iteration
        i = i + 1

    # Check for intersection of circle around p_AC with path
    if intersection:
        # Calculate lookahead point and its local tangent vector
        p_L2 = d*t_hat + p1
        v_L2 = d / np.linalg.norm(d)
        i_cut = i
    else:
        # Calculate lookahead point and its local tangent vector
        # Set lookahead point to the initial path node
        p_L2 = np.array([path[0,i_closest], path[1,i_closest], path[2,i_closest]])
        v_L2 = np.array([path[0,i_closest+1], path[1,i_closest+1], path[2,i_closest+1]]) - p_L2
        v_L2 = v_L2 / np.linalg.norm(v_L2)  # Normalize the vector
        i_cut = i_closest+1

    return p_L2, v_L2

def simple_Unicycle_Dynamics(X, U, V):
    # chi is the heading angle in radians.
    # chi_dot is the heading rate in rad/s.
    # V is the vehicle velocity in (m/s).
    chi = X[2]
    x_dot = np.array([V*math.sin(chi), V*math.cos(chi), U])
    return x_dot

def simple_Aircraft_Dynamics_Acceleration(X, U):
    # X is the inertial position and velocity of the aircraft
    # U is the commanded acceleration vector of the aircraft
    # V is the vehicle velocity in (m/s).
    x_dot = np.array([X[3], X[4], X[5], U[0], U[1], U[2]])
    return x_dot

def simple_Aircraft_Dynamics_Angle_Rates(X, U, V):
    # X is the inertial position and velocity angles
    # U is the commanded angle rates
    # V is the vehicle velocity in (m/s).
    chi = X[3]
    gamma = X[4]
    x_dot = np.array([V*np.sin(chi)*np.cos(gamma), V*np.cos(chi)*np.cos(gamma),
                      V*np.sin(gamma), U[0], U[1]])
    return x_dot

def forward_Euler(x0, x_dot, dt):
    # This function integrates current state with a forward euler integration scheme.
    x1 = x0 + x_dot*dt
    return x1

def convert_Acceleration_to_Angle_Rates(x, a, V):
    # Converts a 3D acceleration vector into angle rate commands on that vector
    chi = x[3]
    gamma = x[4]
    gamma_dot = a[2]/(V*np.cos(gamma))
    if (abs(chi) > 170*np.pi/180) or (abs(chi) < 10*np.pi/180):
        chi_dot = -(a[1] + a[2]*np.cos(chi)*np.tan(gamma)) / (V*np.sin(chi)*np.cos(gamma))
    else:
        chi_dot = (a[0] + a[2]*np.sin(chi)*np.tan(gamma)) / (V*np.cos(chi)*np.cos(gamma))
    rates = np.array([chi_dot, gamma_dot])
    return rates

def angleWrap(theta):
    theta = (theta + np.pi) % (2*np.pi) - np.pi
    return theta