#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry
from xproj_msgs.msg import VehicleState


class XprojVehicleStateToOdomBridge:
    def __init__(self):
        in_topic = rospy.get_param("~input_vehicle_state_topic", "/vehicle/vehicle_state")
        out_topic = rospy.get_param("~output_odom_topic", "/your_project/ego_odom")
        self._frame_id = rospy.get_param("~frame_id", "map")
        self._child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self._pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
        self._sub = rospy.Subscriber(in_topic, VehicleState, self._callback, queue_size=10)

        rospy.loginfo("[xproj_vehicle_state_to_odom_bridge] %s -> %s", in_topic, out_topic)

    def _callback(self, msg: VehicleState):
        odom = Odometry()
        stamp = msg.timestamp if msg.timestamp > 0.0 else rospy.Time.now().to_sec()
        odom.header.stamp = rospy.Time.from_sec(stamp)
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = msg.z

        half = 0.5 * msg.heading
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        odom.twist.twist.linear.x = msg.speed_x
        odom.twist.twist.linear.y = msg.speed_y
        odom.twist.twist.linear.z = msg.speed_z
        odom.twist.twist.angular.z = msg.yaw_rate

        self._pub.publish(odom)


def main():
    rospy.init_node("xproj_vehicle_state_to_odom_bridge")
    XprojVehicleStateToOdomBridge()
    rospy.spin()


if __name__ == "__main__":
    main()