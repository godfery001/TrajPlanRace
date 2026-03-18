import math
import numpy as np
import rospy
from cav_msgs.msg import RefPoint

class CirclePath(object):
    def __init__(self, param):
        self.rad = param["radius"]
        self.interval = param["interval"]
        self.l_marg = param["left_margin"]
        self.r_marg = param["right_margin"]
        self.length = param["length"]
        self.speed = param["speed"]
        self.center = param["center"]
    
    def get_path(self):
        path = []
        degree = self.length / self.rad
        point_num = int(self.length // self.interval+1)
        degree_interval = degree / point_num
        start_t = rospy.Time.now().to_sec()
        for i in range(0, point_num):
            _p = RefPoint()
            _p.x = self.center[0] + self.rad * math.cos(i*degree_interval)
            _p.y = self.center[1] + self.rad * math.sin(i*degree_interval)
            _p.v = self.speed
            _p.cr = 1/self.rad
            _p.heading = i*degree_interval + math.pi/2
            _p.left_width = self.l_marg
            _p.right_width = self.r_marg
            _p.t = start_t + i*self.interval/self.speed
            path.append(_p)
        return path