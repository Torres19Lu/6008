#!/usr/bin/env python3
"""Bridge Gazebo PointCloud and simulated motion to the G1 Livox SLAM inputs."""

import math

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud


class SimLivoxBridge:
    def __init__(self):
        self.scan_period = rospy.get_param("~scan_period", 0.2)
        self.scan_lines = rospy.get_param("~scan_lines", 16)
        self.last_angular_z = 0.0

        self.lidar_pub = rospy.Publisher(
            "/livox/lidar", CustomMsg, queue_size=5)
        self.imu_pub = rospy.Publisher(
            "/livox/imu", Imu, queue_size=200)

        rospy.Subscriber(
            "/simulation/lidar_points", PointCloud,
            self.pointcloud_callback, queue_size=2)
        rospy.Subscriber(
            "/simulation/ground_truth_odom", Odometry,
            self.odom_callback, queue_size=10)

        self.imu_timer = rospy.Timer(rospy.Duration(0.01), self.publish_imu)
        rospy.loginfo(
            "Simulation Livox bridge ready: /simulation/lidar_points -> "
            "/livox/lidar, synthetic IMU -> /livox/imu")

    def odom_callback(self, message):
        self.last_angular_z = message.twist.twist.angular.z

    def pointcloud_callback(self, cloud):
        output = CustomMsg()
        output.header.stamp = cloud.header.stamp
        output.header.frame_id = "mid360_link"
        output.timebase = cloud.header.stamp.to_nsec()
        output.lidar_id = 1
        output.rsvd = [0, 0, 0]

        count = len(cloud.points)
        output.point_num = count
        output.points = []

        denominator = max(1, count - 1)
        scan_ns = int(self.scan_period * 1.0e9)
        for index, source in enumerate(cloud.points):
            if not (math.isfinite(source.x) and
                    math.isfinite(source.y) and
                    math.isfinite(source.z)):
                continue

            point = CustomPoint()
            point.offset_time = int(index * scan_ns / denominator)
            point.x = source.x
            point.y = source.y
            point.z = source.z
            point.reflectivity = 100
            point.tag = 0x10
            point.line = index % self.scan_lines
            output.points.append(point)

        output.point_num = len(output.points)
        self.lidar_pub.publish(output)

    def publish_imu(self, _event):
        message = Imu()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "imu_in_torso"
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.z = self.last_angular_z
        message.angular_velocity_covariance = [
            1.0e-4, 0.0, 0.0,
            0.0, 1.0e-4, 0.0,
            0.0, 0.0, 1.0e-4,
        ]
        # A stationary, Z-up accelerometer measures +g.
        message.linear_acceleration.z = 9.81
        message.linear_acceleration_covariance = [
            1.0e-3, 0.0, 0.0,
            0.0, 1.0e-3, 0.0,
            0.0, 0.0, 1.0e-3,
        ]
        self.imu_pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("sim_livox_bridge")
    SimLivoxBridge()
    rospy.spin()
