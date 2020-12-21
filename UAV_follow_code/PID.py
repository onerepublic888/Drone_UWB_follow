# -*- coding: UTF-8 -*-
import time

class PID:
    def __init__(self, P, I, D):
        self.current_time = time.time()
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.previous_time = self.current_time
        self.previous_error = 0.0

    def update(self, error):
        self.current_time = time.time()
        dt = self.current_time - self.previous_time
        if dt <= 0.0: return 0

        delta_error = error - self.previous_error
        self.PTerm = self.Kp * error 
        self.ITerm += error * dt
        self.DTerm = delta_error / dt
        self.previous_time = self.current_time
        self.previous_error = error

        self.output = (self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm))