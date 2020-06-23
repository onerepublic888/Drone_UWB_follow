import numpy as np
from scipy import optimize
import sys, collections, time
from scipy.optimize import lsq_linear, root, minimize

def lsq_method(distances_to_anchors, anchor_positions):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    if not np.all(distances_to_anchors):
        raise ValueError('Bad uwb connection. distances_to_anchors must never be zero. ' + str(distances_to_anchors))
    anchor_offset = anchor_positions[0]
    anchor_positions = anchor_positions[1:] - anchor_offset
    K = np.sum(np.square(anchor_positions), axis=1)   #ax=1 列加
    squared_distances_to_anchors = np.square(distances_to_anchors)
    squared_distances_to_anchors = (squared_distances_to_anchors - squared_distances_to_anchors[0])[1:]
    b = (K - squared_distances_to_anchors) / 2.
    res = lsq_linear(anchor_positions, b, lsmr_tol='auto', verbose=0)
    return res.x + anchor_offset

def costfun_method(distances_to_anchors, anchor_positions):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    tag_pos = lsq_method(distances_to_anchors, anchor_positions)
    anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(4, 1)
    new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (tag_pos[0] - new_anc_pos[:,0])**2 - (tag_pos[1] - new_anc_pos[:,1])**2))
    new_z = new_z.reshape(4,)

    a = (np.sum(new_disto_anc[:]**2) - 3*np.sum(new_z[:]**2))/len(anchor_positions)
    b = (np.sum((new_disto_anc[:]**2) * (new_z[:])) - np.sum(new_z[:]**3))/len(anchor_positions)
    function = lambda z: z**3 - a*z + b
    derivative = lambda z: 3*z**2 - a

    def newton(function, derivative, x0, tolerance, number_of_max_iterations=50):
        x1, k = 0, 1
        if (abs(x0-x1)<= tolerance and abs((x0-x1)/x0)<= tolerance):  return x0
        while(k <= number_of_max_iterations):
            x1 = x0 - (function(x0)/derivative(x0))
            if (abs(x0-x1)<= tolerance and abs((x0-x1)/x0)<= tolerance): return x1
            x0 = x1
            k = k + 1
            if (k > number_of_max_iterations): print("ERROR: Exceeded max number of iterations")
        return x1 

    newton_z = newton(function, derivative, 80, 0.01)
    new_tag_pos = np.concatenate((np.delete(np.array(tag_pos), 2), [newton_z] + anc_z_ls_mean))
    return np.around(new_tag_pos, 4)
         