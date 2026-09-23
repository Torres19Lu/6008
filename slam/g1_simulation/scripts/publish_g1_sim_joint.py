#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("g1_sim_joint_state")
    publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(20)

    message = JointState()
    message.name = ["waist_yaw_joint"]
    message.position = [0.0]
    message.velocity = []
    message.effort = []

    while not rospy.is_shutdown():
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
