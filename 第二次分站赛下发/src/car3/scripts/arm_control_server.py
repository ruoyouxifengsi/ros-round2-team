#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
arm_control_server.py — 机械臂控制节点
自 ustb_20th_smartcar/src/arm_control_pkg 移植并适配 Round1（无 sim_bot 命名空间）。

封装 JointTrajectory 发布和夹爪控制，供命令行和状态机调用。

Topic API:
  订阅 (命令入口):
    /arm_control/move_to_pose   std_msgs/String  — 位姿名 或 JSON 角度数组
    /arm_control/gripper_cmd    std_msgs/String  — "open" / "close" / 数值
  发布 (状态反馈):
    /arm_control/state          std_msgs/String  — ready / moving / gripper_*

使用示例:
  rostopic pub -1 /arm_control/move_to_pose std_msgs/String "data: 'init'"
  rostopic pub -1 /arm_control/gripper_cmd  std_msgs/String "data: 'close'"
"""

import json
import rospy
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rospy import ROSInterruptException


JOINT_LIMITS = [
    (-3.14, 3.14),  # arm_joint1
    (-3.14, 3.14),  # arm_joint2
    (-3.14, 3.14),  # arm_joint3
    (-3.14, 3.14),  # arm_joint4
    (-1.51, 1.51),  # arm_joint5
]
GRIPPER_LIMIT = (-1.51, 1.51)
JOINT_NAMES = ["arm_joint1", "arm_joint2", "arm_joint3",
               "arm_joint4", "arm_joint5"]


def clamp_angles(angles):
    """校验并截断关节角度到限位范围"""
    clamped = list(angles)
    for i, (v, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS)):
        if not (lo <= v <= hi):
            clamped[i] = max(lo, min(hi, v))
            rospy.logwarn("arm_joint%d: %.3f clamped to %.3f [%.2f, %.2f]",
                          i + 1, v, clamped[i], lo, hi)
    return clamped


class ArmControlServer:
    def __init__(self):
        rospy.init_node("arm_control_server")

        # ---------- 从 param server 加载配置 ----------
        self.poses = rospy.get_param("~poses", {})
        self.gripper_open_val = rospy.get_param("~gripper/open", 0.0)
        self.gripper_close_val = rospy.get_param("~gripper/close", 0.8)
        self.move_duration = rospy.get_param("~move_duration", 2.0)

        if not self.poses:
            rospy.logwarn("No poses loaded! Check poses.yaml is loaded via rosparam.")
        else:
            rospy.loginfo("Loaded %d poses: %s",
                          len(self.poses), list(self.poses.keys()))

        # ---------- 关节状态 ----------
        self.current_positions = {}
        rospy.Subscriber("/joint_states", JointState, self._joint_cb)

        # ---------- 输出 Publisher ----------
        self.arm_pub = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=5)
        self.gripper_pub = rospy.Publisher(
            "/gripper_controller/command", Float64, queue_size=5)
        self.state_pub = rospy.Publisher(
            "/arm_control/state", String, queue_size=5)

        # ---------- 命令入口 (两个 String topic) ----------
        rospy.Subscriber("/arm_control/move_to_pose", String,
                         self._on_pose_cmd)
        rospy.Subscriber("/arm_control/gripper_cmd", String,
                         self._on_gripper_cmd)

        rospy.loginfo("ArmControlServer ready — gripper(open=%.1f, close=%.1f), "
                      "duration=%.1fs", self.gripper_open_val, self.gripper_close_val,
                      self.move_duration)
        self._publish_state("ready")

        # ---------- 启动自动归位 ----------
        init_pose = rospy.get_param("~init_pose", "")
        if init_pose:
            # 等 arm_controller 就绪再发轨迹（慢机器上 controller_spawner 可能要很久，
            # 提前发会被丢弃导致不归位）
            try:
                rospy.wait_for_message("/arm_controller/state",
                                       __import__("control_msgs.msg",
                                                  fromlist=["JointTrajectoryControllerState"]).JointTrajectoryControllerState,
                                       timeout=120)
            except rospy.ROSException:
                rospy.logerr("arm_controller/state 等待超时, 仍尝试发布 init 位姿")
            rospy.sleep(1.0)
            self._execute_pose(init_pose)

    # ---- 回调 ----

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

    def _on_pose_cmd(self, msg):
        self._execute_pose(msg.data.strip())

    def _execute_pose(self, data):
        angles = self._resolve_pose(data)
        if angles is None:
            return

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = angles
        pt.time_from_start = rospy.Duration(self.move_duration)
        traj.points.append(pt)

        self.arm_pub.publish(traj)
        rospy.loginfo("Arm → [%s]", ", ".join(f"{a:.2f}" for a in angles))
        self._publish_state("moving")

    def _on_gripper_cmd(self, msg):
        data = msg.data.strip().lower()
        if data == "open":
            pos = self.gripper_open_val
        elif data == "close":
            pos = self.gripper_close_val
        else:
            try:
                pos = float(data)
            except ValueError:
                rospy.logerr("Gripper: expected 'open'/'close' or number, got '%s'",
                             msg.data)
                return

        pos = max(GRIPPER_LIMIT[0], min(GRIPPER_LIMIT[1], pos))
        self.gripper_pub.publish(Float64(data=pos))
        label = "CLOSE" if pos > 0.5 else "OPEN" if pos < 0.1 else "pos=%.2f" % pos
        rospy.loginfo("Gripper → %s", label)
        self._publish_state("gripper_" + ("close" if pos > 0.5 else "open"))

    # ---- 内部 ----

    def _resolve_pose(self, data):
        if data.startswith("["):
            try:
                angles = json.loads(data)
            except json.JSONDecodeError:
                rospy.logerr("Invalid JSON: %s", data)
                return None
            source = "JSON"
        elif data in self.poses:
            angles = self.poses[data]
            source = data
        else:
            rospy.logerr("Unknown pose '%s'. Available: %s",
                         data, list(self.poses.keys()))
            return None

        if len(angles) != 5:
            rospy.logerr("Expected 5 joint angles, got %d", len(angles))
            return None

        return clamp_angles(angles)

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))


if __name__ == "__main__":
    try:
        ArmControlServer()
        rospy.spin()
    except ROSInterruptException:
        pass
