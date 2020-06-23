import threading
import numpy as np
import sys, os, json, collections, time

def cal_error(D2, gt_dis):
    A = np.array([[1,1,0,0],[1,0,1,0],[1,0,0,1],[0,1,1,0],[0,1,0,1],[0,0,1,1]])
    lsq_A = np.dot(np.linalg.inv(np.dot(A.T, A)),(A.T))
    gt_dis = np.asarray(gt_dis)
    D2 = np.asarray(D2)
    if (D2.shape[0] != 6):
        raise ValueError('Distance array not right!')
    error = np.dot(lsq_A, (D2 - gt_dis))    
    return error

def cal_Anc_pos(dis, anchor_offset_high):#d01, d02, d03, d12, d13, d23 uncounterclockwise,with a0,a1,a3=>z=0, a2=>z != 0 , must be a2 higher
    dis = np.array(dis)
    a0, a1 = [0, 0, anchor_offset_high], [dis[0], 0, anchor_offset_high]   
    x3 = round((dis[2]**2 + dis[5]**2 - dis[0]**2)/(2*dis[5]), 2)     # x3 = (d03**2+d13**2-d01**2)/2d13
    y3 = round((abs(dis[5]**2 - x3**2))**0.5, 2)                    #y3 = (d13**2 - x3**2)*0.5 
    a3 = [x3, y3, anchor_offset_high]
    x2 = round((dis[2]**2 + dis[5]**2 - dis[0]**2)/(2*dis[5]), 2)     # x2 = (d03**2+d13**2-d01**2)/2d13
    y2 = round(((dis[2]**2 - dis[5]**2 + y3**2 + x3**2 - 2*x2*x3)/(2*y3)), 2)   #y2 = (d02**2 - d23**2 + y3**2 +x3**2 - 2*x2*x3)/2y3
    z2 = round((dis[1]**2 - x2**2 - y2**2), 2)
    if (z2 > 0):
        z2 = round((abs(z2))**0.5, 2)        #
    else:
        z2 = 0.5
    a2 = [x2, y2, z2 + anchor_offset_high]
    
    anc_pos = [a0, a1, a2, a3]
    return anc_pos
   
def remove_dis_err(original_dis):
    data = np.load('UWB_cali_data.npz')
    original_dis = np.array(original_dis)
    UWB_err = data['error']
    new_dis = original_dis - UWB_err
    
    return new_dis