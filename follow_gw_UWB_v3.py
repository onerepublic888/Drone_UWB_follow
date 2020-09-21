# -*- coding: UTF-8 -*-
import socket
import threading
import time
import sys
import csv
import threading
import time
import serial
import glob, json
import collections
import datetime
from math import sin, cos, radians, sqrt, pi, atan, acos, atan
import numpy as np
#from scipy.optimize import lsq_linear
from smbus import SMBus
from datetime import datetime
from kalmanfilter import KalmanFilter
from threading import Thread, Event, Lock
from serial import Serial
from time import sleep, strftime

addr = 0x08
bus = SMBus(1)

i2c_value = 200         # 0 ~ 255
dis_queue = collections.deque(maxlen=1)
start_time = datetime.now().strftime("%H_%M_%S")


class Job(Thread):
	def __init__(self, *args, **kwargs):
		p = kwargs["port"]
		kwargs.pop("port", None)
		super(Job, self).__init__(group = None, *args, **kwargs)
		self.__flag, self.__running = Event(), Event()
		self.__flag.set()
		self.__running.set()
		self.port = p

	def pause(self):
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def stop(self):
		self.__flag.set()
		self.__running.clear()

	def run(self):
		global ports, d, dLock

		ser, retries = ports[self.port], 0
		while self.__running.isSet():
			self.__flag.wait()
			for _ in range(10):
				s = ser.readline()
				if not b'ma ' in s:
					continue
				break
			try:
				_, _, _, _, _, _, _ = int(s[-5:-4]), int(s[-3:-2]), int(s[6:14]), int(s[15:23]), int(s[24:32]), int(s[33:41]), int(s[42:50])
			except ValueError:
				sleep(0.1)
				retries += 1
				continue
			src, tgt, dist = int(s[-5:-4]), int(s[-3:-2]), [int(s[6:14]), int(s[15:23]), int(s[24:32]), int(s[33:41]), int(s[42:50])]
			dLock.acquire()
			d[src][tgt] = dist[tgt]
			dLock.release()


def UWB_square_pos(Dis_arr, anc_dis):
    try:
        # cal = 0.6
        # Dis_arr = Dis_arr - cal
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
        if(Y>0 and X>0):ang = ang
        elif(Y>0 and X<0):ang = ang + 180
        elif(Y<0 and X>0):ang = ang
        elif(Y<0 and X<0):ang = ang + 180
        else:ang = ang
        #print('---------------------->ang:', round(ang,2))   
        
        
        return ang, dis
    
    except TypeError:
        print('square_pos function:TypeError angle is none!!!')
    
    except ValueError:
        print('square_pos function: Math domain error! Probably only got 2 distance!!!')


def UWB_dis():
    ser_UWB = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.001)
    while True:
        rx = ser_UWB.readline()
        try:
            if(rx != ' ' and rx.find('mc') >= 0):
                dis = rx.split(' ')
                dis_array = np.array([(int(dis[2],16)),(int(dis[3],16)), (int(dis[4],16)), (int(dis[5],16))])/1000.0
                dis_queue.clear()
                dis_queue.append(dis_array)
                
        except ValueError:
            print('ValueError')
        except IndexError:
            print('IndexError')

# UWBdata[0] = yaw, UWBdata[1] = pitch, if receive UWBdata[0] = 100 / 200 than yaw value = -200 / 200,
# if receive UWBdata[1] = 100 / 200 than pitch value = -200 / 200

def i2c_send(yaw_value, pitch_value):
    def packdata(yaw_value, pitch_value):
        packls = ['','']
        if (yaw_value > 0):
            packls[0] = 150        
        elif(yaw_value < 0):
            packls[0] = 50       
        else:
            packls[0] = 0

        if 100 < pitch_value < 168: pitch_value = 168
        elif 0 < pitch_value < 34: pitch_value = 34  
        else: pitch_value = pitch_value
        packls[1] = pitch_value
            
        return packls
    
    try:
        bus.write_i2c_block_data(addr, 0, packdata(yaw_value, pitch_value))
        print('What I2C value send to arduino: ', packdata(yaw_value, pitch_value))
        time.sleep(0.1)
    
    except IOError:
       print('IOError ')   

uwb_thread = threading.Thread(target=UWB_dis)
uwb_thread.start()
print('UWB thread start!')

ang_range = 0.05
anc_dis  = [0.55, 0.9]

#------------kalman filter parameter--------------------
dt = 1.0/20
F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H = np.array([1, 0, 0]).reshape(1, 3)
Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R = np.array([0.5]).reshape(1, 1)
kf = KalmanFilter(F = F, H = H, Q = Q, R = R)

def _main():
    file_num = int(input('experiment number: '))
    try:
        print('Record initial distance !')
        time.sleep(2)
        if (len(dis_queue) > 0):
            dis_to_tag = dis_queue.popleft()
            if(dis_to_tag[0] != 0):
                ini_tag_dis = dis_to_tag[0] - 0.2
                # ini_tag_dis = 35
                print('--------------ini_tag_dis: ', ini_tag_dis)
                with open('ini_tag_dis_' + str(file_num) +'.txt', 'a') as fout:
                   json.dump({'time': [start_time], 'ini_tag_dis': [ini_tag_dis]}, fout)

            else:
                print('dis_to_tag[0] == 0!')
        else:
            print('len(dis_queue) == 0')

        print('Start following!')
        md = 0.4
        an = 3
        while True:
            if (len(dis_queue) > 0):
                dis_to_tag = dis_queue.popleft()
                if(dis_to_tag[0] != 0):
                    try:
                        ang, dis = UWB_square_pos(dis_to_tag, anc_dis)
                        new_ang = np.dot(H,  kf.predict())[0]
                        if(new_ang == 0): new_ang = ang
                        print('new_ang: ',round(new_ang , 2))
                        kf.update(ang)
                        print('Distance between UAV and Tag: ', dis_to_tag[0])
                        mov_dis = round(dis - ini_tag_dis, 1)
                        print('mov_dis: ',mov_dis)
                
                        if (new_ang > 90 + an):
                            yaw_value = -1*i2c_value
                            print('Turn left !')

                        elif(new_ang < 90 - an):
                            yaw_value = i2c_value  
                            print('Turn right !')

                        else:   
                            yaw_value = 0
                            print("Don't turn !")

                        if(mov_dis >= md):
                            if mov_dis>100: mov_dis = 100 
                            pitch_value = int(round(mov_dis)) + 100
                            print('Forward !')

                        elif(mov_dis <= -1 * md):
                            if mov_dis<-100: mov_dis = -100 
                            pitch_value = int(round(abs(mov_dis)) )
                            print('Backward !')

                        else:
                            pitch_value = 0
                            print('Do not move !')

                        i2c_send(yaw_value, pitch_value)
                        print('Send I2c value! ')


                        rightnow_ti = datetime.now().strftime("%H:%M:%S")
                        with open('follow_data_' + str(file_num) +'.txt', 'a') as fout:
                            json.dump({'time': rightnow_ti, 'dis_to_tag': [str(dis_to_tag)], 'mov_dis': [str(mov_dis)], 'new_ang': [str(new_ang)]}, fout)       

                            
                    except TypeError:
                        print(TypeError)
                        #print('Main Loop :TypeError angle is none!!!')
                else:
                    print('dis_to_tag[0] == 0 ! Do not move')
                    i2c_send(0, 0)

    except KeyboardInterrupt:
        i2c_send(0, 0)
        print('KeyboardInterrupt')
        uwb_thread.stop()
        print('All thread stop!')
        
    
if __name__=='__main__':
    _main()
