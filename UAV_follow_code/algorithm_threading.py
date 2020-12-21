import numpy as np
from scipy import optimize
import sys, collections, time
from math import sin, cos, radians, sqrt , pi, atan
# from scipy.optimize import lsq_linear, root, minimize
'''
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
'''
def costfun_method(distances_to_anchors, anchor_positions, xy_pos):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    anc_z_ls_mean = np.mean(np.array([i[2] for i in anchor_positions]) )  
    new_z = (np.array([i[2] for i in anchor_positions]) - anc_z_ls_mean).reshape(4, 1)
    new_anc_pos = np.concatenate((np.delete(anchor_positions, 2, axis = 1), new_z ), axis=1)
    new_disto_anc = np.sqrt(abs(distances_to_anchors[:]**2 - (xy_pos[0] - new_anc_pos[:,0])**2 - (xy_pos[1] - new_anc_pos[:,1])**2))
    new_z = new_z.reshape(4,)

    cost = lambda z: np.sum(((z - new_z[:])**4 - 2*(((new_disto_anc[:])*(z - new_z[:]))**2 ) + new_disto_anc[:]**4))/len(anchor_positions) 

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

    newton_z = newton(function, derivative, 20, 0.01)

    # newton_z_from_postive = newton(function, derivative, 10, 0.01)
    # newton_z_from_negative = newton(function, derivative, -10, 0.01)

    # def find_newton_global(newton_z_from_postive, newton_z_from_negative):
    #     if cost(newton_z_from_postive) < cost(newton_z_from_negative):
    #         return newton_z_from_postive
    #     elif cost(newton_z_from_negative) < cost(newton_z_from_postive):
    #         return newton_z_from_negative

    # if(newton_z_from_postive < 5):
    #     newton_z = find_newton_global(newton_z_from_postive, newton_z_from_negative)
    # else:
    #     newton_z = newton_z_from_postive

    new_tag_pos = np.concatenate((xy_pos, [newton_z] + anc_z_ls_mean))
    # ranges = (slice(-10, 10, 0.1), )
    # resbrute = optimize.brute(cost, ranges, full_output = True, finish = optimize.fmin)
    # new_tag_pos = np.concatenate((xy_pos, resbrute[0] + anc_z_ls_mean))

    return np.around(new_tag_pos, 4)

def UWB_square_pos(Dis_arr, anc_dis):
    try:
        # cal = 0.6
        # anc_pos = np.array(anc_pos_list)
        # print('Dis_arr: ', Dis_arr)
        d1, d2, d3, d4 = Dis_arr[0], Dis_arr[1], Dis_arr[2], Dis_arr[3]
        dl, dw = anc_dis[0], anc_dis[1]
        # dis = Dis_arr[0] + 0.5
        '''
        ((H^T * H)^-1)*(H^T * b), H dimension define by how many distance, b will be 2*1 array
        '''   
        if(d1 != 0 and d2 == 0 and d3 != 0 and d4 != 0 ):
            d1, d3, d4 = d1-0.65, d3-0.65, d4-0.65
            H = np.array( [ [dl, -dw], [0, -dw] ] )
            b = np.array( [ [(d1**2 - d3**2)/2], [(d1**2 - d4**2)/2] ] )
            # print('less d2')

        elif(d1 != 0 and d2 != 0 and d3 == 0 and d4 != 0 ):
            d1, d2, d4 = d1-0.65, d2-0.65, d4-0.65
            H = np.array( [ [dl, 0], [0, -dw] ] )
            b = np.array( [ [(d1**2 - d2**2)/2], [(d1**2 - d4**2)/2] ] )
            # print('less d3')

        elif(d1 != 0 and d2 != 0 and d3 != 0 and d4 == 0 ):
            d1, d2, d3 = d1-0.65, d2-0.65, d3-0.65
            H = np.array( [ [dl, 0], [dl, -dw] ] )
            b = np.array( [ [(d1**2 - d2**2)/2], [(d1**2 - d3**2)/2] ] )
            # print('less d4')

        else:
            d1, d2, d3, d4 = d1-0.65, d2-0.65, d3-0.65, d4-0.65
            H = np.array( [ [dl,0],[0,-dw],[dl,-dw] ] )
            b = np.array( [ [(d1**2-d2**2)/2], [(d1**2-d4**2)/2], [(d1**2-d3**2)/2] ] )
            # print('No less ')

        x = np.dot(np.linalg.inv(np.dot(H.T, H)), np.dot(H.T, b))
        X, Y = x[0,0], x[1,0]
        ang = atan(Y/X)*180/pi

        if(Y>0 and X>0):ang = ang
        elif(Y>0 and X<0):ang = ang + 180
        elif(Y<0 and X>0):ang = ang
        elif(Y<0 and X<0):ang = ang + 180
        else:ang = ang
        xy_pos = np.array([X, Y])
        # new_tag_pos = costfun_method(Dis_arr, anc_pos, xy_pos)
        # print(new_tag_pos)
        # theta = atan(np.linalg.norm(xy_pos)/new_tag_pos[2])*180/pi

        return np.around(ang,2)#,np.around(theta,2), np.around(new_tag_pos,2)
    
    except TypeError:
        print('square_pos function:TypeError angle is none!!!')
    except ValueError:
        print('square_pos function: Math domain error! Probably only got 2 distance!!!')

         