import math
import numpy as np
import rospy
from cav_msgs.msg import RefPoint


class LinePath(object):
    def __init__(self, param):
        self.interval = param["interval"]
        self.l_marg = param["left_margin"]
        self.r_marg = param["right_margin"]
        self.length = param["length"]
        self.speed = param["speed"]
        self.center = param["center"]
    
    def get_path(self):
        path = []
        start_t = rospy.Time.now().to_sec()
        start_x = self.center[0] - self.length*np.sqrt(2)/4
        start_y = self.center[1] - self.length*np.sqrt(2)/4
        point_num = int(self.length // self.interval+1)
        for i in range(0, point_num):
            _p = RefPoint()
            _p.x = start_x + i*self.interval*np.sqrt(2)/2
            _p.y = start_y + i*self.interval*np.sqrt(2)/2
            _p.v = self.speed
            _p.cr = 0
            _p.heading = math.pi/4
            _p.left_width = self.l_marg
            _p.right_width = self.r_marg
            _p.t = start_t + i*self.interval/self.speed
            path.append(_p)
        return path