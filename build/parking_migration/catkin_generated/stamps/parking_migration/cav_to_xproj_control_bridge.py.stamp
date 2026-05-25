#!/usr/bin/env python3

import rospy

from cav_msgs.msg import Control as CavControl
from xproj_msgs.msg import Control as XprojControl


class CavToXprojControlBridge:
    def __init__(self):
        in_topic = rospy.get_param("~input_control_topic", "/Control")
        out_topic = rospy.get_param("~output_control_topic", "/vehicle/control2bywire")

        self._pub = rospy.Publisher(out_topic, XprojControl, queue_size=10)
        self._sub = rospy.Subscriber(in_topic, CavControl, self._callback, queue_size=10)

        rospy.loginfo("[cav_to_xproj_control_bridge] %s -> %s", in_topic, out_topic)

    def _callback(self, msg: CavControl):
        out = XprojControl()
        out.timestamp = msg.timestamp
        out.count = msg.count
        out.brake_cmd = msg.brake_cmd
        out.throttle_cmd = msg.throttle_cmd
        out.steering_cmd_front_wheel = msg.steering_cmd_front_wheel
        out.gear_cmd = msg.gear_cmd
        out.turn_signal_cmd = msg.turn_signal_cmd
        out.speed_cmd = msg.speed_cmd
        out.acceleration_cmd = msg.acceleration_cmd
        out.bywire_control_enable = msg.bywire_control_enable
        out.emerg_brake = msg.emerg_brake
        out.front_light = msg.front_light
        out.engine_enable = msg.engine_enable
        out.park_enable = msg.park_enable
        out.vehicle_mode = msg.vehicle_mode
        out.yaw_speed_cmd = msg.yaw_speed_cmd
        out.ey_out = msg.ey_out
        out.ephi_out = msg.ephi_out
        self._pub.publish(out)


def main():
    rospy.init_node("cav_to_xproj_control_bridge")
    CavToXprojControlBridge()
    rospy.spin()


if __name__ == "__main__":
    main()