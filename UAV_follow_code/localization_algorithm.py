# -*- coding: UTF-8 -*-
import socket
import threading
import time
import sys
import csv
import serial
import glob, json
import collections
import datetime
from math import sin, cos, radians, sqrt, pi, atan, acos, atan
import numpy as np
#from scipy.optimize import lsq_linear
from datetime import datetime
from kalmanfilter import KalmanFilter


def UWB_square_pos(Dis_arr, anc_dis):
    try:
        # cal = 0.6
        ti = datetime.now().strftime("%H:%M:%S")
        print('-----------Time-------------: ' + ti)
        d1, d2, d3, d4 = Dis_arr[0], Dis_arr[1], Dis_arr[2], Dis_arr[3]
        dl, dw = anc_dis[0], anc_dis[1]
        dis = Dis_arr[0] + 0.5
        '''
        ((H^T * H)^-1)*(H^T * b), H dimension define by how many distance, b will be 2*1 array
        '''   
        if(d1 != 0 and d2 == 0 and d3 != 0 and d4 != 0 ):
            d1, d3, d4 = d1-0.65, d3-0.65, d4-0.65
            H = np.array( [ [dl, -dw], [0, -dw] ] )
            b = np.array( [ [(d1**2 - d3**2)/2], [(d1**2 - d4**2)/2] ] )
            print('less d2')

        elif(d1 != 0 and d2 != 0 and d3 == 0 and d4 != 0 ):
            d1, d2, d4 = d1-0.65, d2-0.65, d4-0.65
            H = np.array( [ [dl, 0], [0, -dw] ] )
            b = np.array( [ [(d1**2 - d2**2)/2], [(d1**2 - d4**2)/2] ] )
            print('less d3')

        elif(d1 != 0 and d2 != 0 and d3 != 0 and d4 == 0 ):
            d1, d2, d3 = d1-0.65, d2-0.65, d3-0.65
            H = np.array( [ [dl, 0], [dl, -dw] ] )
            b = np.array( [ [(d1**2 - d2**2)/2], [(d1**2 - d3**2)/2] ] )
            print('less d4')

        else:
            d1, d2, d3, d4 = d1-0.65, d2-0.65, d3-0.65, d4-0.65
            H = np.array( [ [dl,0],[0,-dw],[dl,-dw] ] )
            b = np.array( [ [(d1**2-d2**2)/2], [(d1**2-d4**2)/2], [(d1**2-d3**2)/2] ] )
            print('No less ')

        x = np.dot(np.linalg.inv(np.dot(H.T, H)), np.dot(H.T, b))
        X, Y = x[0,0], x[1,0]
        ang = atan(Y/X)*180/pi
        if(ang<-3):ang = ang + 180
        else:ang = ang  
        
        return ang, dis
    
    except TypeError:
        print('square_pos function:TypeError angle is none!!!')
    
    except ValueError:
        print('square_pos function: Math domain error! Probably only got 2 distance!!!')