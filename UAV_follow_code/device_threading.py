# -*- coding: UTF-8 -*-
import time
import sys
import serial
import glob, json
import collections
import datetime
import numpy as np
from datetime import datetime
import logging
from Adafruit_BNO055 import BNO055
# from __future__ import print_function
import threading

def find_ports():
    ports = glob.glob('/dev/ttyACM[0-4]*')
    res = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            res.append(port)
            print('port: ', res)
        except:
            print('wrong port!')
            
    return res

class IMU_data(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)               
        self.IMU_ls = []
        self.daemon = True
        self._stop = False
        
    def run(self):
        bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
        if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
            logging.basicConfig(level=logging.DEBUG)

        time.sleep(1)
        if not bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        status, self_test, error = bno.get_system_status()
        sw, bl, accel, mag, gyro = bno.get_revision()

        while True:
            heading, roll, pitch = bno.read_euler()
            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = bno.get_calibration_status()
            # print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
            #     heading, roll, pitch, sys, gyro, accel, mag))
            time.sleep(0.1)

            self.IMU_ls = [heading, roll, pitch]  

    def stop(self):
        self._stop = True


