#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WL100 里程计积分与消息构建模块

坐标系: 严格遵循 REP-105 标准
  - 父坐标系: odom (世界固定)
  - 子坐标系: base_link (机器人体固定)

运动学模型: 全向底盘死区推算 (Dead Reckoning)
  - delta_x     = (Vx·cosθ - Vy·sinθ) · dt
  - delta_y     = (Vx·sinθ + Vy·cosθ) · dt
  - delta_theta = Vz · dt

遵循规范: /home/nvidia/robot_ws/JETSON_RULES.md
"""

import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdometryIntegrator:
    """
    全向底盘死区推算积分器

    使用方法:
        integrator = OdometryIntegrator()
        integrator.update(vx, vy, vz, dt)
        odom_msg = integrator.build_odom_msg(stamp, 'odom', 'base_link')
        tf_msg = integrator.build_tf_msg(stamp, 'odom', 'base_link')
    """

    def __init__(self):
        # 全局位姿状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 当前速度快照（用于填充 Odometry.twist）
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def update(self, vx, vy, vz, dt):
        """
        根据当前速度和时间间隔进行位姿积分

        Args:
            vx: X 方向线速度 (m/s, 机体坐标系)
            vy: Y 方向线速度 (m/s, 机体坐标系)
            vz: Z 方向角速度 (rad/s)
            dt: 时间间隔 (秒)
        """
        # 异常 dt 保护：负数、零、或超大值（如首次调用）
        if dt <= 0.0 or dt > 1.0:
            return

        self.vx = vx
        self.vy = vy
        self.vz = vz

        # 全向底盘运动学：机体坐标系 → 全局坐标系
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        delta_x = (vx * cos_theta - vy * sin_theta) * dt
        delta_y = (vx * sin_theta + vy * cos_theta) * dt
        delta_theta = vz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def _yaw_to_quaternion(self):
        """
        从 yaw 角生成四元数（仅绕 Z 轴旋转，2D 平面机器人）

        Returns:
            tuple: (qx, qy, qz, qw)
        """
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        return (0.0, 0.0, qz, qw)

    def build_odom_msg(self, stamp, odom_frame_id, base_frame_id):
        """
        构建 nav_msgs/Odometry 消息

        Args:
            stamp: ROS 时间戳 (builtin_interfaces/Time)
            odom_frame_id: 里程计父坐标系名 (如 'odom')
            base_frame_id: 机器人基座坐标系名 (如 'base_link')

        Returns:
            Odometry 消息
        """
        qx, qy, qz, qw = self._yaw_to_quaternion()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame_id
        msg.child_frame_id = base_frame_id

        # 位姿 (全局坐标系下)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 速度 (机体坐标系下，即 child_frame_id)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.vz

        return msg

    def build_tf_msg(self, stamp, odom_frame_id, base_frame_id):
        """
        构建 odom -> base_link 的 TF 变换消息

        Args:
            stamp: ROS 时间戳 (builtin_interfaces/Time)
            odom_frame_id: 父坐标系名
            base_frame_id: 子坐标系名

        Returns:
            TransformStamped 消息
        """
        qx, qy, qz, qw = self._yaw_to_quaternion()

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame_id
        msg.child_frame_id = base_frame_id

        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        return msg
