#!/usr/bin/env python3
"""Move the simulated G1 toward the red chair while keeping it centered."""

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class ChairApproach:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "g1_sim")
        self.chair_x = rospy.get_param("~chair_x", 2.5)
        self.chair_y = rospy.get_param("~chair_y", 0.0)
        self.stop_distance = rospy.get_param("~stop_distance", 1.8)
        self.speed = rospy.get_param("~speed", 0.25)
        self.start_hold = rospy.get_param("~start_hold", 3.0)
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.step_frequency = rospy.get_param("~step_frequency", 1.4)
        self.lateral_sway = rospy.get_param("~lateral_sway", 0.012)
        self.vertical_bob = rospy.get_param("~vertical_bob", 0.008)
        self.roll_sway = rospy.get_param("~roll_sway", 0.010)
        self.pitch_sway = rospy.get_param("~pitch_sway", 0.006)

        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.get_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.set_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        self.odom_pub = rospy.Publisher(
            "/simulation/ground_truth_odom", Odometry, queue_size=10)

    def publish_odom(self, command, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "base_link"
        message.pose.pose = command.pose
        message.twist.twist = command.twist
        self.odom_pub.publish(message)

    def run(self):
        initial = self.get_state(self.model_name, "world")
        if not initial.success:
            rospy.logfatal("Cannot read G1 model state: %s", initial.status_message)
            return

        x = initial.pose.position.x
        y = initial.pose.position.y
        z = initial.pose.position.z
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        gait_phase = 0.0
        reached_logged = False

        rospy.loginfo(
            "Chair approach ready: target=(%.2f, %.2f), stop distance=%.2f m",
            self.chair_x, self.chair_y, self.stop_distance)

        while not rospy.is_shutdown():
            dx = self.chair_x - x
            dy = self.chair_y - y
            distance = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)
            elapsed = (rospy.Time.now() - start_time).to_sec()

            linear = 0.0
            if elapsed >= self.start_hold and distance > self.stop_distance:
                linear = self.speed
                step = min(self.speed * dt, distance - self.stop_distance)
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
            elif elapsed >= self.start_hold and not reached_logged:
                rospy.loginfo("Target red chair reached at %.2f m", distance)
                reached_logged = True

            # Recompute heading after the position update so the optical axis
            # remains centered on the chair during the whole approach.
            yaw = math.atan2(self.chair_y - y, self.chair_x - x)
            if linear > 0.0:
                gait_phase += 2.0 * math.pi * self.step_frequency * dt
                sway = self.lateral_sway * math.sin(gait_phase)
                bob = self.vertical_bob * 0.5 * (
                    1.0 - math.cos(2.0 * gait_phase))
                roll = self.roll_sway * math.sin(gait_phase)
                pitch = self.pitch_sway * math.sin(2.0 * gait_phase)
            else:
                sway = bob = roll = pitch = 0.0

            rendered_x = x - math.sin(yaw) * sway
            rendered_y = y + math.cos(yaw) * sway
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            command = ModelState()
            command.model_name = self.model_name
            command.reference_frame = "world"
            command.pose.position.x = rendered_x
            command.pose.position.y = rendered_y
            command.pose.position.z = z + bob
            command.pose.orientation.x = quaternion[0]
            command.pose.orientation.y = quaternion[1]
            command.pose.orientation.z = quaternion[2]
            command.pose.orientation.w = quaternion[3]
            command.twist.linear.x = linear

            response = self.set_state(command)
            if not response.success:
                rospy.logwarn_throttle(2.0, "Move failed: %s", response.status_message)
            self.publish_odom(command, rospy.Time.now())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("approach_red_chair")
    ChairApproach().run()
