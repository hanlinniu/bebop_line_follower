#!/usr/bin/python

'''

__Author_ = Lowyi
__Email__ = MR.LowBattery@gmail.com
__Team__  = MRL_UAV

'''

import time


class PID:
    ''' PID controller algorithm '''

    def __init__(self, kp, ki, kd):
        self.last_error = 0
        self.integral = 0
        self.last_run = time.time()
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, error):
        self.integral += self.ki * float(self.last_error + error) / 2
        now = time.time()
        derivative = self.kd * float(error - self.last_error) / (now - self.last_run)
        self.last_error = error
        self.last_run = now
        output = self.integral + derivative + self.kp * error
        output = min(0.5, max(-0.5, output))
        return output

    def setConstants(self, kp, ki, kd):
        self.last_error = 0
        self.integral = 0
        self.last_run = time.time()
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.last_error = 0
        self.integral = 0
        self.last_run = time.time()
