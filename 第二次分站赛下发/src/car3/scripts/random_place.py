#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 Gazebo 中随机摆放 3 个物块 + 1 个 QR 码。

QR 候选: qr_board_s / qr_board_goods / qr_board_electronic
  随机选 1 个放到 (0.45, 0.25, yaw=-1.57)，
  其余 2 个移到远处隐藏。

Cube 候选: cube_0 / cube_1 / cube_2
  随机打乱后依次放到 3 个目标点。

依赖: ROS1 (noetic) + gazebo_ros
运行: rosrun <your_pkg> random_place.py
"""

import math
import random

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion


# ---------- 参数 ----------
QR_CANDIDATES = ["qr_board_s", "qr_board_goods", "qr_board_electronic"]
CUBE_CANDIDATES = ["cube_0", "cube_1", "cube_2"]

# QR 目标位姿 (x, y, z, yaw)
QR_TARGET = (0.45, 0.25, 0.0, -1.57)

# 物块目标位姿 (x, y, z, yaw)
#   z = 0.02，是 cube 半高，保证底面贴地；若要严格 0，改成 0.0
Z_CUBE = 0.02
CUBE_TARGETS = [
    (-2.30, 2.50, Z_CUBE, 0.0),
    (-1.75, 2.80, Z_CUBE, 0.0),
    (-1.20, 2.50, Z_CUBE, 0.0),
]

# 未选中 QR 的藏匿位置（远在地下，防止穿帮）
HIDDEN_POS = (100.0, 100.0, -50.0, 0.0)
# --------------------------


def yaw_to_quaternion(yaw):
    """绕 z 轴的 yaw → 四元数"""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


def make_state(model_name, x, y, z, yaw):
    st = ModelState()
    st.model_name = model_name
    st.pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=yaw_to_quaternion(yaw),
    )
    st.reference_frame = "world"
    return st


def call_set_state(proxy, model_name, pose4):
    try:
        ok = proxy(make_state(model_name, *pose4))
        if not ok.success:
            rospy.logwarn("[random_place] 设置 %s 失败: %s", model_name, ok.status_message)
        return ok.success
    except rospy.ServiceException as e:
        rospy.logwarn("[random_place] 调用 %s 异常: %s", model_name, e)
        return False


def main():
    rospy.init_node("random_place", anonymous=True)

    # ---- 随机抽选 ----
    chosen_qr = random.choice(QR_CANDIDATES)
    shuffled_cubes = CUBE_CANDIDATES[:]
    random.shuffle(shuffled_cubes)

    rospy.loginfo("[random_place] 选中 QR : %s", chosen_qr)
    rospy.loginfo("[random_place] cube 分配: %s",
                  list(zip(shuffled_cubes, CUBE_TARGETS)))

    # ---- 等待服务 ----
    rospy.loginfo("[random_place] 等待 /gazebo/set_model_state ...")
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # ---- 1) 隐藏未被选中的 QR ----
    for qr in QR_CANDIDATES:
        if qr == chosen_qr:
            continue
        call_set_state(set_state, qr, HIDDEN_POS)
        rospy.loginfo("[random_place] 隐藏 %s", qr)

    # ---- 2) 放置选中的 QR ----
    if call_set_state(set_state, chosen_qr, QR_TARGET):
        rospy.loginfo("[random_place] QR %s -> (%.2f, %.2f, yaw=%.2f)",
                      chosen_qr, QR_TARGET[0], QR_TARGET[1], QR_TARGET[3])

    # ---- 3) 按随机顺序放 cube ----
    for cube_name, target in zip(shuffled_cubes, CUBE_TARGETS):
        if call_set_state(set_state, cube_name, target):
            rospy.loginfo("[random_place] %s -> (%.2f, %.2f, yaw=%.2f)",
                          cube_name, target[0], target[1], target[3])

    rospy.loginfo("[random_place] 完成 ✅")


if __name__ == "__main__":
    main()