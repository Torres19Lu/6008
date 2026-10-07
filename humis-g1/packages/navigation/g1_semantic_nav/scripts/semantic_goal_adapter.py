#!/usr/bin/env python3
"""Convert a stable VLM object coordinate into a reachable G1 navigation goal."""

import json
import math
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from g1_msgs.msg import NavigateToAction, NavigateToGoal
from nav_msgs.msg import OccupancyGrid
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped TF conversion
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from navigation_tools import GridView, choose_approach_candidates


class SemanticGoalAdapter:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.status_topic = rospy.get_param(
            "~status_topic", "/vlm/semantic_target")
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/vlm/target_pose")
        self.costmap_topic = rospy.get_param("~costmap_topic", "/nav/costmap")
        self.action_name = rospy.get_param("~action_name", "/navigate_to")
        self.expected_target = rospy.get_param("~expected_target", "red chair").strip().lower()
        # Simulation starts automatically. The real launch overrides this to
        # false so seeing a target can never move hardware before an operator
        # explicitly opens the semantic-navigation gate.
        self.enabled = bool(rospy.get_param("~enabled_on_start", True))
        self.max_goal_cost = int(rospy.get_param("~max_goal_cost", 50))
        self.target_update_distance = float(
            rospy.get_param("~target_update_distance", 0.30))
        self.retry_cooldown = float(rospy.get_param("~retry_cooldown", 10.0))
        self.radii = [float(value) for value in rospy.get_param(
            "~approach_radii", [1.2, 1.0, 1.4])]
        self.angle_offsets = [math.radians(float(value)) for value in rospy.get_param(
            "~candidate_angle_degrees", [0, 45, -45, 90, -90, 135, -135, 180])]

        self.lock = threading.Lock()
        # ROS callbacks can run concurrently. This second lock serializes
        # send_goal/cancel_goal so closing the real-hardware safety gate cannot
        # race with a goal submission and leave that goal uncancelled.
        self.action_lock = threading.Lock()
        self.stable = False
        self.status_target = ""
        self.pending_pose = None
        self.costmap = None
        self.active = False
        self.generation = 0
        self.candidates = []
        self.current_target_xy = None
        self.last_attempt_target_xy = None
        self.last_attempt_time = rospy.Time(0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = actionlib.SimpleActionClient(
            self.action_name, NavigateToAction)

        rospy.Subscriber(
            self.status_topic, String, self.status_callback, queue_size=1)
        rospy.Subscriber(
            self.target_pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber(
            self.costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1)

        rospy.loginfo("Waiting for navigation action %s", self.action_name)
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            raise RuntimeError(
                "navigation action {} was not available within 30 seconds".format(
                    self.action_name))
        self.enable_service = rospy.Service(
            "~enable", SetBool, self.enable_callback)
        rospy.loginfo(
            "Semantic goal adapter ready: %s + %s -> %s (enabled=%s)",
            self.status_topic, self.target_pose_topic, self.action_name,
            self.enabled)

    def enable_callback(self, request):
        """Open or close the automatic VLM-to-navigation safety gate."""

        with self.lock:
            was_active = self.active
            self.enabled = bool(request.data)
            if not self.enabled:
                # Invalidate pending action callbacks before cancelling the goal.
                self.generation += 1
                self.active = False
                self.candidates = []
                pose = None
            else:
                pose = self.pending_pose if self.stable else None
            enabled = self.enabled

        if not enabled and was_active:
            with self.action_lock:
                self.client.cancel_goal()
        if enabled and pose is not None:
            self.consider_target(pose)

        state = "enabled" if enabled else "disabled"
        rospy.logwarn("Semantic navigation gate %s", state)
        return SetBoolResponse(success=True, message=state)

    def status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(5.0, "Invalid VLM status JSON: %s", error)
            return

        target = str(payload.get("target_text", "")).strip().lower()
        stable = bool(payload.get("stable", False))
        if self.expected_target and target != self.expected_target:
            return

        with self.lock:
            self.status_target = target
            self.stable = stable
            pose = self.pending_pose if stable and self.enabled else None
        if pose is not None:
            self.consider_target(pose)

    def pose_callback(self, message):
        with self.lock:
            self.pending_pose = message
            ready = self.stable and self.enabled
        if ready:
            self.consider_target(message)

    def costmap_callback(self, message):
        orientation = message.info.origin.orientation
        origin_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])[2]
        view = GridView(
            resolution=message.info.resolution,
            width=message.info.width,
            height=message.info.height,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            origin_yaw=origin_yaw,
            data=tuple(message.data),
        )
        with self.lock:
            self.costmap = view
            pose = self.pending_pose if self.stable and self.enabled else None
        if pose is not None:
            self.consider_target(pose)

    def transform_target(self, message):
        if not message.header.frame_id:
            message.header.frame_id = self.map_frame
        if message.header.frame_id == self.map_frame:
            return message
        return self.tf_buffer.transform(
            message, self.map_frame, timeout=rospy.Duration(0.5))

    def robot_xy(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def consider_target(self, raw_pose):
        with self.lock:
            if not self.enabled:
                return
        try:
            target_pose = self.transform_target(raw_pose)
            target_xy = (
                target_pose.pose.position.x,
                target_pose.pose.position.y,
            )
            robot_xy = self.robot_xy()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(3.0, "Cannot prepare semantic goal: %s", error)
            return

        with self.lock:
            grid = self.costmap
            active = self.active
            previous = self.last_attempt_target_xy
            elapsed = (rospy.Time.now() - self.last_attempt_time).to_sec()
        if grid is None:
            rospy.logwarn_throttle(3.0, "Waiting for %s", self.costmap_topic)
            return

        if previous is not None:
            moved = math.hypot(
                target_xy[0] - previous[0], target_xy[1] - previous[1])
            if moved < self.target_update_distance and (
                    active or elapsed < self.retry_cooldown):
                return

        candidates = choose_approach_candidates(
            target_xy=target_xy,
            robot_xy=robot_xy,
            grid=grid,
            radii=self.radii,
            angle_offsets=self.angle_offsets,
            max_goal_cost=self.max_goal_cost,
        )
        if not candidates:
            rospy.logwarn(
                "No free approach point around target (%.2f, %.2f)", *target_xy)
            with self.lock:
                self.last_attempt_target_xy = target_xy
                self.last_attempt_time = rospy.Time.now()
            return

        with self.lock:
            # Candidate generation performs TF and costmap work outside the
            # lock. Re-check the hardware gate here so a concurrent disable
            # request cannot be followed by a newly submitted navigation goal.
            if not self.enabled:
                return
            self.generation += 1
            generation = self.generation
            was_active = self.active
            self.active = True
            self.current_target_xy = target_xy
            self.last_attempt_target_xy = target_xy
            self.last_attempt_time = rospy.Time.now()
            self.candidates = list(candidates)
        if was_active:
            with self.action_lock:
                self.client.cancel_goal()
        self.send_next_candidate(generation)

    def send_next_candidate(self, generation):
        with self.lock:
            # Disabling the gate increments generation and clears enabled. The
            # explicit enabled check also protects the small interval before a
            # cancellation callback reaches this method.
            if generation != self.generation or not self.enabled:
                return
            if not self.candidates:
                self.active = False
                target_xy = self.current_target_xy
                rospy.logerr(
                    "All approach points failed for target (%.2f, %.2f)",
                    target_xy[0], target_xy[1])
                return
            candidate = self.candidates.pop(0)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = candidate.x
        pose.pose.position.y = candidate.y
        quaternion = quaternion_from_euler(0.0, 0.0, candidate.yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        goal = NavigateToGoal()
        goal.goal_type = NavigateToGoal.GOAL_POSE
        goal.target_pose = pose
        rospy.loginfo(
            "Navigating to approach pose (%.2f, %.2f, yaw=%.1f deg, cost=%d)",
            candidate.x, candidate.y, math.degrees(candidate.yaw), candidate.cost)
        # Closing the gate updates enabled before waiting for action_lock. The
        # re-check below therefore guarantees one of two safe outcomes:
        # no goal is sent, or the disable callback cancels the just-sent goal.
        with self.action_lock:
            with self.lock:
                if generation != self.generation or not self.enabled:
                    return
            self.client.send_goal(
                goal,
                done_cb=lambda state, result: self.navigation_done(
                    generation, state, result),
            )

    def navigation_done(self, generation, state, result):
        with self.lock:
            if generation != self.generation:
                return
        success = state == GoalStatus.SUCCEEDED and bool(
            getattr(result, "success", False))
        if success:
            with self.lock:
                self.active = False
            rospy.loginfo("Semantic navigation succeeded: %s", result.message)
            return

        message = getattr(result, "message", "no result")
        rospy.logwarn(
            "Approach pose failed (action state=%d): %s; trying next candidate",
            state, message)
        self.send_next_candidate(generation)


if __name__ == "__main__":
    rospy.init_node("semantic_goal_adapter")
    try:
        SemanticGoalAdapter()
        rospy.spin()
    except (RuntimeError, rospy.ROSInterruptException) as error:
        rospy.logfatal("Semantic goal adapter stopped: %s", error)
