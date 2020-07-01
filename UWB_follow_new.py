# -*- coding: utf-8 -*-
import sys, csv, threading, time, serial, glob, collections, datetime
from math import sin, cos, radians, sqrt , pi, atan, acos
import numpy as np
from smbus import SMBus
from localization_algorithm_threading import costfun_method
from caliculate_error_threading import cal_error, cal_Anc_pos, remove_dis_err


'''Define queue or variable
anc_dis: distance between anchor on drone
addr: I2C address 
bus: rpi I2C library instance
i2c_value: value to send to arduino device
data_name: use for record experiment data
'''
dis_queue = collections.deque(maxlen = 1)
ang_range = 0.05
anc_dis = 0.55
anc_pos = [[0,0,1], [anc_dis,0,1], [anc_dis, anc_dis,1], [0, anc_dis,1]]
addr = 0x08
bus = SMBus(1)
i2c_value = 100         # 0 ~ 500
string_time = str(input('file number:')) 

data_name = 'UWB_follow_data_' + string_time +'.txt'
  
'''This function is to send value to arduino, if value is positive then the data format will be [0, value], if value is negative then the data format will be [value, 0]''' 
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
 
    try:
        bus.write_i2c_block_data(addr, 0, packdata(i2c_value))
        #print('send write_i2c_block_data: ', packdata(i2c_value))
    
    except IOError:
       print('IOError ')
        
'''If Rpi connect UWB via usb port need to use this function to find correct com port name'''
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

'''
This function is to read four UWB distance data from serial port and will put this function in threading, and UWB data format will translate from hex into decimal.
'''
def UWB_dis(ser):
    ser_UWB = serial.Serial(ser[0], baudrate = 115200, timeout=0.005)
    while True:
        rx = ser_UWB.readline()
        try:
            '''if receive data is not empty and find 'mc' at the head of string'''
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
    print('UWB thread start!')
    try:
        print('Start following!')
        while True:
            '''If dis_queue is not empty '''
            if (len(dis_queue) > 0):
                dis_to_tag = dis_queue.popleft()
                dis_queue.clear()
                dis_to_tag = np.array(dis_to_tag)
                #if not np.all(dis_to_tag):
                    #raise ValueError('Got distance with zero. ' + str(dis_to_tag))
                if(np.all(dis_to_tag)):
                    print(time.asctime( time.localtime(time.time()) ))
                    try:
                        tag_pos, ang = costfun_method(dis_to_tag, anc_pos)
                        print('tag_pos: ', tag_pos)
                        print('Tag angle: ', ang)
                        if (ang > 90):
                            print('Turn left ', i2c_value)
                            '''i2c value send 300 times'''
                            for _ in range(300):
                                i2c_send(i2c_value)
                                
                        elif(ang < 90):
                            print('Turn right', i2c_value)
                            for _ in range(300):
                                i2c_send(i2c_value*(-1))
                                
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

