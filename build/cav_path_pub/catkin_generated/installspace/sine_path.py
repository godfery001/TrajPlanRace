import math
import numpy as np
import rospy
from cav_msgs.msg import RefPoint

class SinePath(object):
    def __init__(self, param):
        self.A = param["sine_amp"]
        self.T = param["sine_T"]
        self.interval = param["interval"]
        self.l_marg = param["left_margin"]
        self.r_marg = param["right_margin"]
        self.length = param["length"]
        self.speed = param["speed"]
        self.center = param["center"]
    
    def get_path(self):
        path = []
        start_t = rospy.Time.now().to_sec()
        total_length = 0
        start_x = self.center[0] -10
        start_y = self.center[1] -10
        x_ = [0] 
        y_ = [0]
        c = 2*math.pi/self.T
        A = self.A
        while total_length < self.length:
            _p = RefPoint()
            x = x_[-1] + self.interval/2
            y = self.A * math.sin(c*(x+ self.T*0.25))
            cos = math.cos(c*(x+ self.T*0.25))
            sin = math.sin(c*(x+ self.T*0.25))
            total_length += np.sqrt((x-x_[-1])**2 + (y-y_[-1])**2)
            x_.append(x)
            y_.append(y)
            yd = A*c*cos
            ydd = -A*c**2*sin 
            
            _p.x = start_x + x*np.sqrt(2)/2 - y*np.sqrt(2)/2
            _p.y = start_y + x*np.sqrt(2)/2 + y*np.sqrt(2)/2
            _p.v = self.speed
            _p.cr = ydd/np.power((1+yd**2),1.5)
            _p.t = start_t + total_length/self.speed
            _p.heading = math.atan(self.A*c*cos)+math.pi/4
            _p.left_width = self.l_marg
            _p.right_width = self.r_marg
            path.append(_p)
        print(path)
        return path
            