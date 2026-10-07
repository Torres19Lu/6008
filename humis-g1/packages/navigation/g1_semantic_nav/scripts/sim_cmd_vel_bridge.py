#!/usr/bin/env python3
"""Apply the navigation stack's /cmd_vel output to the Gazebo G1 model."""

import math
import threading

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def clamp(value, limit):
    return max(-limit, min(limit, value))


class SimCmdVelBridge:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "g1_sim")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.6))
        self.max_lateral_speed = float(rospy.get_param("~max_lateral_speed", 0.4))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.0))

        self.lock = threading.Lock()
        self.command = Twist()
        self.last_command_time = rospy.Time(0)
        self.nominal_x = None
        self.nominal_y = None
        self.nominal_z = None
        self.nominal_yaw = None
        self.last_update = None

        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.get_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.set_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        self.odom_pub = rospy.Publisher(
            "/simulation/ground_truth_odom", Odometry, queue_size=10)
        rospy.Subscriber(
            self.cmd_topic, Twist, self.command_callback, queue_size=1)

        initial = self.get_state(self.model_name, "world")
        if not initial.success:
            raise RuntimeError(
                "cannot read Gazebo model {}: {}".format(
                    self.model_name, initial.status_message))
        orientation = initial.pose.orientation
        self.nominal_x = initial.pose.position.x
        self.nominal_y = initial.pose.position.y
        self.nominal_z = initial.pose.position.z
        self.nominal_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.last_update = rospy.Time.now()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self.update)
        rospy.loginfo(
            "Simulation cmd_vel bridge ready: %s -> Gazebo model %s",
            self.cmd_topic, self.model_name)

    def command_callback(self, message):
        with self.lock:
            self.command = message
            self.last_command_time = rospy.Time.now()

    def current_command(self, now):
        with self.lock:
            age = (now - self.last_command_time).to_sec()
            if age > self.command_timeout:
                return 0.0, 0.0, 0.0
            return (
                clamp(self.command.linear.x, self.max_linear_speed),
                clamp(self.command.linear.y, self.max_lateral_speed),
                clamp(self.command.angular.z, self.max_angular_speed),
            )

    def update(self, event):
        now = event.current_real
        dt = max(0.0, min((now - self.last_update).to_sec(), 0.2))
        self.last_update = now
        vx, vy, angular = self.current_command(now)

        cosine = math.cos(self.nominal_yaw)
        sine = math.sin(self.nominal_yaw)
        self.nominal_x += (cosine * vx - sine * vy) * dt
        self.nominal_y += (sine * vx + cosine * vy) * dt
        self.nominal_yaw = math.atan2(
            math.sin(self.nominal_yaw + angular * dt),
            math.cos(self.nominal_yaw + angular * dt),
        )

        quaternion = quaternion_from_euler(0.0, 0.0, self.nominal_yaw)
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.nominal_x
        state.pose.position.y = self.nominal_y
        state.pose.position.z = self.nominal_z
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.angular.z = angular

        try:
            response = self.set_state(state)
            if not response.success:
                rospy.logwarn_throttle(
                    2.0, "Cannot move Gazebo model: %s", response.status_message)
        except rospy.ServiceException as error:
            rospy.logerr_throttle(2.0, "Gazebo set_model_state failed: %s", error)
            return
        self.publish_odom(state, now)

    def publish_odom(self, state, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "base_link"
        message.pose.pose = state.pose
        message.twist.twist = state.twist
        self.odom_pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("sim_cmd_vel_bridge")
    try:
        SimCmdVelBridge()
        rospy.spin()
    except (RuntimeError, rospy.ROSInterruptException) as error:
        rospy.logfatal("Simulation cmd_vel bridge stopped: %s", error)
