#!/usr/bin/env python3
import rospy
import time
from cav_msgs.msg import Goal
import os
import sys
import matplotlib.pyplot as plt
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('cav_path_pub')
original_script_dir = os.path.abspath(os.path.join(package_path, 'scripts'))
sys.path.insert(0,original_script_dir)
from circle_path import CirclePath
from sine_path import SinePath
from line_path import LinePath

PathMode = {"Circle":CirclePath, "Sine":SinePath, "Line":LinePath}


class PathPublisher(object):
    def __init__(self):
        # ros node
        rospy.init_node('cav_path_pub')
        self.rate = rospy.Rate(2) #2Hz
        self.pub_goal = rospy.Publisher("/Goal", Goal, queue_size=2)
        
        path_mode = rospy.get_param("path_mode","Circle")
        param = self.load_params()
        print("Selected path_mode: ", path_mode)
        self.path_generator = PathMode[path_mode](param)
        self.goal_msg = Goal()
        
    
    def load_params(self):
        # common param
        speed = rospy.get_param("speed", 3)
        interval = 0.1
        length = rospy.get_param("length", 20)
        l_marg = rospy.get_param("left_margin", 0.1)
        r_marg = rospy.get_param("right_margin", 0.1)
        c_x = rospy.get_param("center_x", 443673.722475)
        c_y = rospy.get_param("center_y", 4429497.481223)
        
        # path specific param
        rad = rospy.get_param("radium", 10)
        sine_amp = rospy.get_param("sine_amp", 3)
        sine_T = rospy.get_param("sine_T", 6)
        
        param = {"radius":rad, "interval":interval, 
                 "left_margin":l_marg, "right_margin":r_marg, 
                 "length":length, "sine_amp":sine_amp, 
                 "sine_T":sine_T,"speed":speed,
                 "center":(c_x, c_y)}
        return param
        
    def gen_goal(self):
        path = self.path_generator.get_path()
        self.goal_msg = Goal()
        self.goal_msg.refPath = path
        self.goal_msg.timestamp = path[0].t
        self.goal_msg.Estop = 0
        self.goal_msg.Go = True
        self.pub_goal.publish(self.goal_msg)
        self.plot_goal()
        rospy.loginfo("Publish goal with timestamp: %f", self.goal_msg.timestamp)
    
    def plot_goal(self):
        x = []
        y = []
        for p in self.goal_msg.refPath:
            x.append(p.x)
            y.append(p.y)
        plt.plot(x, y)
        plt.axis('equal')
        image_path = os.path.join(package_path, 'resources', 'path.png')
        plt.savefig(image_path)
        

    def run(self):
        time.sleep(2) # Sleep for 2 seconds
        for x in range(10):
            self.gen_goal()
            time.sleep(0.01) 
     

if __name__ == '__main__':
    try:
        pb = PathPublisher()
        pb.run()
    except rospy.ROSInterruptException:
        pass