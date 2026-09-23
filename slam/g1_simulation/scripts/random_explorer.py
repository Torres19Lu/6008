#!/usr/bin/env python3
"""Obstacle-reactive random exploration for the Gazebo g1_sim model."""

import math
import random

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RandomExplorer:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "g1_sim")
        self.min_x = rospy.get_param("~min_x", -4.1)
        self.max_x = rospy.get_param("~max_x", 4.1)
        self.min_y = rospy.get_param("~min_y", -4.1)
        self.max_y = rospy.get_param("~max_y", 4.1)
        self.linear_speed = rospy.get_param("~linear_speed", 0.28)
        self.angular_speed = rospy.get_param("~angular_speed", 0.45)
        self.turn_distance = rospy.get_param("~turn_distance", 1.0)
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.seed = rospy.get_param("~seed", -1)
        self.step_frequency = rospy.get_param("~step_frequency", 1.4)
        self.lateral_sway = rospy.get_param("~lateral_sway", 0.018)
        self.vertical_bob = rospy.get_param("~vertical_bob", 0.012)
        self.roll_sway = rospy.get_param("~roll_sway", 0.022)
        self.pitch_sway = rospy.get_param("~pitch_sway", 0.012)
        if self.seed >= 0:
            random.seed(self.seed)

        # (x, y, clearance radius): red chair and the diagonal box.
        self.obstacles = [(2.5, 0.0, 0.75), (2.0, 1.0, 1.0)]
        self.turn_target = None
        self.nominal_x = None
        self.nominal_y = None
        self.nominal_z = None
        self.nominal_yaw = None
        self.gait_phase = 0.0

        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.get_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.set_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        self.odom_pub = rospy.Publisher(
            "/simulation/ground_truth_odom", Odometry, queue_size=10)

    def is_free(self, x, y):
        if not (self.min_x <= x <= self.max_x and
                self.min_y <= y <= self.max_y):
            return False
        for ox, oy, radius in self.obstacles:
            if math.hypot(x - ox, y - oy) < radius:
                return False
        return True

    def clearance_ahead(self, x, y, yaw):
        """Distance along a heading to a wall or expanded circular obstacle."""
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        distances = []

        if dx > 1e-6:
            distances.append((self.max_x - x) / dx)
        elif dx < -1e-6:
            distances.append((self.min_x - x) / dx)
        if dy > 1e-6:
            distances.append((self.max_y - y) / dy)
        elif dy < -1e-6:
            distances.append((self.min_y - y) / dy)

        # Ray-circle intersections. Radii already include robot clearance.
        for ox, oy, radius in self.obstacles:
            rel_x = ox - x
            rel_y = oy - y
            along = rel_x * dx + rel_y * dy
            if along <= 0.0:
                continue
            perpendicular_sq = rel_x * rel_x + rel_y * rel_y - along * along
            if perpendicular_sq >= radius * radius:
                continue
            half_chord = math.sqrt(max(0.0, radius * radius - perpendicular_sq))
            hit = along - half_chord
            if hit >= 0.0:
                distances.append(hit)

        positive = [distance for distance in distances if distance >= 0.0]
        return min(positive) if positive else float("inf")

    def choose_turn_heading(self, x, y, yaw):
        """Randomly choose among the safest headings, vacuum-cleaner style."""
        candidates = []
        min_turn = math.radians(55.0)
        max_turn = math.radians(125.0)
        for _ in range(40):
            direction = random.choice((-1.0, 1.0))
            turn = direction * random.uniform(min_turn, max_turn)
            candidate = wrap_angle(yaw + turn)
            clearance = self.clearance_ahead(x, y, candidate)
            # Prefer open space, while random jitter varies left/right choices.
            score = min(clearance, 6.0) + random.uniform(0.0, 0.8)
            candidates.append((score, candidate))
        candidates.sort(reverse=True)
        safe = candidates[:8]
        _, heading = random.choice(safe)
        rospy.loginfo("Obstacle %.2f m ahead; turning from %.1f to %.1f deg",
                      self.clearance_ahead(x, y, yaw), math.degrees(yaw),
                      math.degrees(heading))
        return heading

    def publish_odom(self, state, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.pose.pose = state.pose
        msg.twist.twist = state.twist
        self.odom_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.control_rate)
        dt = 1.0 / self.control_rate

        while not rospy.is_shutdown():
            current = self.get_state(self.model_name, "world")
            if not current.success:
                rospy.logerr_throttle(2.0, "Cannot read Gazebo model state: %s",
                                      current.status_message)
                rate.sleep()
                continue

            x = current.pose.position.x
            y = current.pose.position.y
            q = current.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            # Keep a separate unswayed trajectory. Gazebo reports the rendered
            # pose (including gait sway), which must not be integrated again.
            if self.nominal_x is None:
                self.nominal_x = x
                self.nominal_y = y
                self.nominal_z = current.pose.position.z
                self.nominal_yaw = yaw
            x = self.nominal_x
            y = self.nominal_y
            yaw = self.nominal_yaw
            linear = 0.0
            angular = 0.0

            if (self.turn_target is None and
                    self.clearance_ahead(x, y, yaw) <= self.turn_distance):
                self.turn_target = self.choose_turn_heading(x, y, yaw)

            if self.turn_target is not None:
                error = wrap_angle(self.turn_target - yaw)
                if abs(error) < 0.04:
                    yaw = self.turn_target
                    self.turn_target = None
                else:
                    angular = math.copysign(self.angular_speed, error)
            else:
                linear = self.linear_speed

            new_yaw = wrap_angle(yaw + angular * dt)
            new_x = x + linear * math.cos(new_yaw) * dt
            new_y = y + linear * math.sin(new_yaw) * dt

            if not self.is_free(new_x, new_y):
                self.turn_target = self.choose_turn_heading(x, y, yaw)
                linear = 0.0
                new_x, new_y = x, y

            self.nominal_x = new_x
            self.nominal_y = new_y
            self.nominal_yaw = new_yaw

            moving = abs(linear) > 1e-4
            if moving:
                self.gait_phase = wrap_angle(
                    self.gait_phase + 2.0 * math.pi * self.step_frequency * dt)
                sway = self.lateral_sway * math.sin(self.gait_phase)
                bob = self.vertical_bob * 0.5 * (
                    1.0 - math.cos(2.0 * self.gait_phase))
                roll = self.roll_sway * math.sin(self.gait_phase)
                pitch = self.pitch_sway * math.sin(2.0 * self.gait_phase)
            else:
                sway = bob = roll = pitch = 0.0

            # Lateral displacement is perpendicular to the walking direction.
            rendered_x = new_x - math.sin(new_yaw) * sway
            rendered_y = new_y + math.cos(new_yaw) * sway

            command = ModelState()
            command.model_name = self.model_name
            command.reference_frame = "world"
            command.pose.position.x = rendered_x
            command.pose.position.y = rendered_y
            command.pose.position.z = self.nominal_z + bob
            quat = quaternion_from_euler(roll, pitch, new_yaw)
            command.pose.orientation.x = quat[0]
            command.pose.orientation.y = quat[1]
            command.pose.orientation.z = quat[2]
            command.pose.orientation.w = quat[3]
            command.twist.linear.x = linear
            command.twist.angular.z = angular

            result = self.set_state(command)
            if not result.success:
                rospy.logwarn_throttle(2.0, "Cannot move Gazebo model: %s",
                                       result.status_message)
            self.publish_odom(command, rospy.Time.now())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("g1_random_explorer")
    RandomExplorer().run()
