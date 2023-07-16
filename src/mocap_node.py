#!/usr/bin/env python
import rospy
# from clean_desired_state import setpoint_publisher_time_dependent
from mocap import get_mocap
# from get_setpoints import setpoint_publisher
rospy.init_node("RPY_publishing_node",anonymous=False)
RPY_publisher = get_mocap()

# waypoint_publisher = setpoint_publisher()
loop_rate = rospy.Rate(5)

while not rospy.is_shutdown():
    RPY_publisher.spin()
    loop_rate.sleep()
