# -*- coding: utf-8 -*-
import sys, csv, threading, time, serial, glob, collections, datetime
from math import sin, cos, radians, sqrt , pi, atan, acos
import numpy as np
#from scipy.optimize import lsq_linear, root
from smbus import SMBus
from localization_algorithm_threading import costfun_method
from caliculate_error_threading import cal_error, cal_Anc_pos, remove_dis_err

dis_queue = collections.deque(maxlen = 1)
ang_range = 0.05
anc_dis = [0.55]
addr = 0x08
bus = SMBus(1)
i2c_value = 100         # 0 ~ 500
string_time = str(input('file number:')) 

data_name = 'UWB_follow_data_' + string_time +'.txt'
  
    
def i2c_send(i2c_value):
    def packdata(i2c_value):
        packls = ['','']
        if (i2c_value > 0):
            packls[0] = int(0)
            packls[1] = abs(i2c_value)
            return packls
            
        elif(i2c_value < 0):
            packls[0] = abs(i2c_value)
            packls[1] = int(0)
            return packls
        
        else:
            packls[0], packls[1] = 0, 0
            return packls
    #px4_value = packdata(value)
    #print('px4_value: ', px4_value)
    try:
        bus.write_i2c_block_data(addr, 0, packdata(i2c_value))
        #print('send write_i2c_block_data: ', packdata(i2c_value))
        #time.sleep(0.1)
        
    
    except IOError:
       print('IOError ')
        

def find_ports():
    ports = glob.glob('/dev/ttyACM[0-9]*')
    res = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            res.append(port)
        except:
            print('wrong port!')
            
    return res

def UWB_dis(ser):
    ser_UWB = serial.Serial(ser[0], baudrate = 115200, timeout=0.005)
    while True:
        rx = ser_UWB.readline()
        try:
            if(rx != ' ' and rx.find('mc') >= 0):
                dis = rx.split(' ')
                dis_array = [(int(dis[2],16)/1000.0),(int(dis[3],16)/1000.0), (int(dis[4],16)/1000.0), (int(dis[5],16)/1000.0)]
                dis_queue.clear()
                dis_queue.append(dis_array)
                #print(dis_array)
        except ValueError:
            print('ValueError')
                
def _main():
    UWB_port = find_ports()
    uwb_thread = threading.Thread(target = UWB_dis, args=(UWB_port,))
    uwb_thread.start()
    print('thread start!')
    try:
        print('Start following!')
        while True:
            if (len(dis_queue) > 0):
                dis_to_tag = dis_queue.popleft()
                dis_queue.clear()
                dis_to_tag = np.array(dis_to_tag)
                #if not np.all(dis_to_tag):
                    #raise ValueError('Got distance with zero. ' + str(dis_to_tag))
                if(np.all(dis_to_tag)):
                    print(time.asctime( time.localtime(time.time()) ))
                    try:
                        ang = costfun_method(dis_to_tag, anc_dis)
                        print('Tag angle: ', ang)
                        if (ang > 90):
                            print('Turn left ', i2c_value)
                            for _ in range(300):
                                i2c_send(i2c_value)
                                #print('send i2c value')
                        elif(ang < 90):
                            print('Turn right', i2c_value)
                            for _ in range(300):
                                i2c_send(i2c_value*(-1))
                                #print('send i2c value')
                        else:
                            print('Do not move')
                            
                    except TypeError:
                        print('TypeError: angle is none!!!')
                else:
                    print('Anchor number < 4 !')
                                           
    except KeyboardInterrupt:
        print('stop!')
        uwb_thread.stop()
            

if __name__=='__main__':
    _main()

