import socket
import threading
import time
import sys
import csv
import threading
import time
import serial
import glob
import collections
import datetime
from math import sin, cos, radians, sqrt, pi, atan, acos, atan
import numpy as np
from scipy.optimize import lsq_linear
# from localization_algorithm_threading import costfun_method
# from caliculate_error_threading import cal_error, cal_Anc_pos, remove_dis_err


dis_queue = collections.deque(maxlen=1)
tello1_address = ('192.168.10.1', 8889)
local1_address = ('192.168.10.2', 9000)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.bind(local1_address)

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

def UWB_dis(ser):
    ser_UWB = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0.005)
    while True:
        rx = ser_UWB.readline()
        try:
            if(rx != ' ' and rx.find('mc') >= 0):
                dis = rx.split(' ')
                dis_array = np.array([int(dis[2], 16), int(dis[2], 16), int(dis[2], 16), int(dis[2], 16)])/1000
                # dis_array = [(int(dis[2], 16)/1000.0), (int(dis[3], 16)/1000.0),
                #              (int(dis[4], 16)/1000.0), (int(dis[5], 16)/1000.0)]
                dis_queue.clear()
                dis_queue.append(dis_array)
                # print(dis_array)
        except ValueError:
            print('ValueError')


def send(message, delay):
    try:
        sock1.sendto(message.encode(), tello1_address)
        print("Sending message: " + message)
    except Exception as e:
        print("Error sending: " + str(e))

    time.sleep(delay)


def receive():
    while True:
        try:
            response1, ip_address = sock1.recvfrom(128)
            print("Received message: " + response1.decode(encoding='utf-8'))

        except Exception as e:
            sock1.close()

            print("Error receiving: " + str(e))
            break

'''
send('battery?', 1)
'''
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

rec_thread = threading.Thread(target=receive)
rec_thread.daemon = True
rec_thread.start()

UWB_port = find_ports()

uwb_thread = threading.Thread(target=UWB_dis, args=(UWB_port,))
uwb_thread.start()
print('UWB thread start!')
x, y, z = 0.9, 0.6, 1 
anc_pos = [[0,0,z], [x,0,z], [x,y,z], [0,y,z]]

def _main():
    try:
        send("command", 3)
        send("streamoff", 3)  # unable camera
        print('Close video stream!')
        print('Take off!')
        send("takeoff", 6)
        if (len(dis_queue) > 0):
            dis_to_tag = dis_queue.popleft()
            if(np.all(dis_to_tag)):
                ini_tag_pos = costfun_method(dis_to_tag, anc_pos)
                print('ini_tag_pos: ', ini_tag_pos)
            else:
                print('Got less than 4 dis!')
        else:
            print('len(dis_queue) == 0')

        print('Start following!')
        while True:
            if (len(dis_queue) > 0):
                dis_to_tag = dis_queue.popleft()
                # dis_to_tag = np.array(dis_to_tag)
                if(np.all(dis_to_tag)):
                    print(time.asctime( time.localtime(time.time()) ))
                    tag_pos = costfun_method(dis_to_tag, anc_pos)
                    mov_dis = np.linalg.norm(ini_tag_pos - tag_pos, axis=0)*100
                    phi = atan((ini_tag_pos[0] - tag_pos[0])/(ini_tag_pos[1] - tag_pos[1]))*(180/pi)
                    if phi > 0 :
                        send("ccw " + str(abs(phi)), 3)
                    elif phi < 0:
                        send("cw " + str(abs(phi)), 3)
                    else:
                        print('Straight forward!')

                    send("forward " + str(mov_dis), 5)


                    time.sleep(1)


    except KeyboardInterrupt:
        print('KeyboardInterrupt')
        print('Land!')
        send("land", 5)
        send("land", 3)
        print('Land!')

        uwb_thread.stop()
        rec_thread.stop()
        sock1.close()
        print('All thread stop!')

if __name__=='__main__':
    _main()