#!/usr/bin/env python3
#coding = utf-8

import numpy as np
from typing import List
from cav_msgs.msg import RefPoint,Goal
import math 
from scipy.interpolate import splprep, splev
import rospy

import os
import json



def splinePath(path, step:float):
    # spline the path with step meter
    # Convert path coordinates to separate x and y arrays
    x = [point[0]-1 for point in path]
    y = [point[1] for point in path]

    # Spline the path
    tck, u = splprep([x, y], s=0)
    u_new = np.arange(0, 1, step)
    x_spline, y_spline = splev(u_new, tck)
    
    # 求一阶和二阶导数
    dx, dy = splev(u_new, tck, der=1)
    ddx, ddy = splev(u_new, tck, der=2)

    # 曲率计算
    curvature = []
    for dx_, dy_, ddx_, ddy_ in zip(dx, dy, ddx, ddy):
        num = dx_ * ddy_ - dy_ * ddx_
        denom = (dx_**2 + dy_**2) ** 1.5
        kappa = num / denom if denom != 0 else 0.0
        curvature.append(kappa)

    # Convert the spline coordinates back to a list of Point objects
    spline_path = [RefPoint(x=x, y=y, cr=cr) for x, y, cr in zip(x_spline, y_spline, curvature)]
    return spline_path

def update_heading(path:List[RefPoint]):
    if len(path)<1: return
    for i in range(0, len(path)-1):
        path[i].heading = math.atan2(path[i+1].y-path[i].y, path[i+1].x-path[i].x)

    path[len(path)-1].heading = path[len(path)-2].heading



def update_speed_time(path:List[RefPoint], speed):
    if len(path)<2: return
    path[0].v = speed
    path[0].t = rospy.Time.now().to_sec()
    for i in range(1, len(path)):
        dist = np.sqrt((path[i].x-path[i-1].x)**2 + (path[i].y-path[i-1].y)**2)
        path[i].t = path[i-1].t + dist/speed
        path[i].v = speed
        path[i].left_width = 3
        path[i].right_width= 3


def generate_msg(sampled_points, speed):
    msg = Goal()
    msg.timestamp = rospy.Time.now().to_sec()
    msg.Go = 1
    msg.Estop = 0
    msg.refPath.clear()
    
    ref_path = splinePath(sampled_points,0.002)
    update_speed_time(ref_path, speed)
    update_heading(ref_path)
    
    msg.refPath = ref_path
    return msg
    


if __name__ == "__main__":

    rospy.init_node('path_generator')
    # sampled_points = [
    #     [442656.52, 4428702.81],
    #     [442649.23, 4428704.58],
    #     [442643.83, 4428710.13],
    #     [442641.90, 4428717.68],
    #     [442643.84, 4428725.58],
    #     [442649.06, 4428730.63],
    #     [442660.34, 4428729.78],
    #     [442656.57, 4428708.88],
    #     [442650.34, 4428711.51],
    #     [442647.71, 4428717.82],
    #     [442650.22, 4428724.17],
    #     [442656.66, 4428726.88],
    #     [442663.57, 4428718.88],
    #     ]

    center_x= 442656.12
    center_y= 4428720.75
    offset_x= 0
    offset_y= -12
    radius = rospy.get_param("~radius", 6.0)
    start_angle = -np.pi/2
    direction = -1 # 1 for counter-clockwise, -1 for clockwise
    
    crir_center_x= center_x + offset_x
    crir_center_y= center_y + offset_y
    # 以中心点为圆心，半径为radius，生成一圈的采样点
    sampled_points = []
    for angle in np.linspace(0, 2*np.pi-np.pi/18, num=12):
        angle = angle if direction == 1 else -angle
        angle += start_angle
        x = crir_center_x + radius * np.cos(angle)
        y = crir_center_y + radius * np.sin(angle)
        sampled_points.append([x, y])

    # cur_file_dir=os.path.dirname(os.path.abspath(__file__))
    # config_file_path=os.path.join(cur_file_dir,"../config/circle_path.json")
    
    # with open(config_file_path, "r") as f:
    #     data = json.load(f)
    #     sampled_points = data["sampled_points"]

    
    
    pub_goal = rospy.Publisher('/Goal', Goal, queue_size=2)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        print("Publishing path...")
        speed = rospy.get_param("~speed", 1.0)
        path_msg = generate_msg(sampled_points, speed)
        pub_goal.publish(path_msg)
        rate.sleep()


