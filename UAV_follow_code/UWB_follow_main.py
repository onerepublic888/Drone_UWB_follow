# -*- coding: UTF-8 -*-
import threading, time, sys, serial
import glob, json
import collections
from datetime import datetime
from math import sin, cos, radians, sqrt, pi, atan, acos, atan
import numpy as np
from smbus import SMBus
import logging
from Adafruit_BNO055 import BNO055
from kalmanfilter import KalmanFilter
from localization_algorithm import UWB_square_pos
from device_threading import find_ports#, IMU_data

addr = 0x08
bus = SMBus(1)

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
time.sleep(1)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# dis_queue = collections.deque(maxlen=1)
start_time = datetime.now().strftime("%H_%M_%S")
tag_queue = collections.deque(maxlen=1)
last_data_queue = collections.deque(maxlen=1)

ang_range = 0.05
anc_dis  = [0.55, 0.9]

string_time = datetime.now().strftime("%H_%M_%S")
data_filename = 'follw_data_' + string_time +'.txt'

#------------kalman filter parameter--------------------
dt = 1.0/20
F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H = np.array([1, 0, 0]).reshape(1, 3)
Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R = np.array([0.0001]).reshape(1, 1)
kf = KalmanFilter(F = F, H = H, Q = Q, R = R)

def UWB_dis():
    #UWB_port = find_ports()
    #ser_UWB = serial.Serial(UWB_port[0], baudrate = 115200, timeout=0.05)
    ser_UWB = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.05)
    while True:
        rx = ser_UWB.readline()
        try:
            if(rx != ' ' and rx.find('mc') >= 0):
                dis = rx.split(' ')
                dis_to_tag = np.array([(int(dis[2],16)),(int(dis[3],16)), (int(dis[4],16)), (int(dis[5],16))])/1000.0
                
                ang, dis = UWB_square_pos(dis_to_tag, anc_dis)
                new_ang = np.around((np.dot(H,  kf.predict())[0]), 2)
                if(new_ang == 0): new_ang = ang
                kf.update(ang)
                #print(new_ang[0], dis, dis_to_tag.tolist())
                #tag_queue.clear()
                tag_queue.append([new_ang[0], dis, dis_to_tag.tolist()])

            else:
                pass

        except ValueError:
            print('ValueError')

        except IndexError:
            print('IndexError')

        except TypeError:
            print('TypeError')

# UWBdata[0] = yaw, UWBdata[1] = pitch, 0 ~ 200 mapping to 500 ~ 1500, 100(1000) will be middle value means don't move.
def i2c_send(yaw_value, pitch_value):
    def packdata(yaw_value, pitch_value):
        packls = ['','']
        if 100 < yaw_value < 105: yaw_value = 105    
        elif 95 < yaw_value < 100: yaw_value = 95      
        else: yaw_value = yaw_value

        if 100 < pitch_value < 120: pitch_value = 120
        elif 80 < pitch_value < 100: pitch_value = 80  
        else: pitch_value = pitch_value

        packls[0] = yaw_value
        packls[1] = pitch_value
            
        return packls
    
    try:
        bus.write_i2c_block_data(addr, 0, packdata(yaw_value, pitch_value))
    
    except IOError:
       print('IOError ')   


def _main():
    if (len(tag_queue) > 0):
        ini_tag_data = tag_queue.popleft() 
        print('--------------ini_tag_data: ', ini_tag_data[0], ini_tag_data[1])
        with open('ini_tag_dis_' + string_time  +'.txt', 'a') as fout:
            json.dump({'time': start_time, 'ini_tag_data': ini_tag_data}, fout)

    else:
        print('len(tag_queue) == 0')
    
    #-----------Tracking parameter threshold --------------------
    Mov_thr = 0.5    # Moving threshold
    Ang_thr = 5      # Turn Yaw angle turn threshold
    ini_yaw_value = 40  # 40*5 = 200, flycontroller will receive 200.
    yaw_ratio = 0.8

    print('Record initial distance. Start following!!')

    while True:
        heading, roll, pitch = bno.read_euler()
        AHRS = [heading, roll, pitch]
        #print('AHRS: ', AHRS)

        if (len(tag_queue) > 0):
            tag_data = tag_queue.popleft()

            if (len(last_data_queue) > 0):
                last_tag_data = last_data_queue.popleft()
            else:
                last_tag_data = ini_tag_data

            try:
                rightnow_ti = datetime.now().strftime("%H:%M:%S")
                print('---------------------------', rightnow_ti)
                
                mov_dis = round(tag_data[1] - ini_tag_data[1], 2)
                print('mov_dis: ',mov_dis)
                new_ang = tag_data[0]
                print('new_ang: ',round(new_ang , 2))
                print('AHRS: ', AHRS)
                #----------------- Delta_angle -----------------
                Delta_angle = round((abs(last_tag_data[0] - 90) - abs(new_ang - 90)), 2)
                print('Delta Angle: ', Delta_angle)

                if (new_ang > 90 + Ang_thr):
                    if Delta_angle > 0 : ini_yaw_value = ini_yaw_value * yaw_ratio
                    else: ini_yaw_value = 40
                    yaw_value = 100 - ini_yaw_value
                    print('Turn left !')

                elif(new_ang < 90 - Ang_thr):
                    if Delta_angle > 0 : ini_yaw_value = ini_yaw_value * yaw_ratio
                    else: ini_yaw_value = 40
                    yaw_value = ini_yaw_value + 100
                    print('Turn right !')

                else:   
                    yaw_value = 100     # middle value, don't move.
                    ini_yaw_value = 40
                    print("Don't turn !")

                if(mov_dis >= Mov_thr):
                    if mov_dis>100: mov_dis = 100 
                    pitch_value = int(round(mov_dis)) + 100
                    print('Forward !')

                elif(mov_dis <= -1 * Mov_thr):
                    if mov_dis<-100: mov_dis = -100 
                    pitch_value = 100 - int(round(abs(mov_dis)) )
                    print('Backward !')

                else:
                    pitch_value = 100     # middle value, don't move.
                    print('Do not move !')

                i2c_send(yaw_value, pitch_value)

                
                with open(data_filename, 'a') as fout:
                    json.dump({'time': rightnow_ti, 'dis_to_tag': tag_data[2], 'mov_dis': mov_dis, 'new_ang': new_ang, 'AHRS': AHRS}, fout)       

                last_data_queue.append(tag_data)

            except TypeError:
                #print(TypeError)
                i2c_send(100, 100)
                print(AHRS)
                
        else:
            #print('len(tag_queue) == 0')
            #print('AHRS: ', AHRS)
            i2c_send(100, 100)

    time.sleep(0.25)

        
    
if __name__=='__main__':
    try:
        uwb_thread = threading.Thread(target=UWB_dis)
        uwb_thread.start()
        time.sleep(2)
        print('UWB and Calulate angle thread start!')
        
        _main()

    except KeyboardInterrupt:
        i2c_send(100, 100)
        print('KeyboardInterrupt')
        uwb_thread.stop()
        print('All thread stop!')
