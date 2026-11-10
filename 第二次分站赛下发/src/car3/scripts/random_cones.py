#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 Gazebo 中随机摆放 5 个 Construction Cone 锥桶（yaw 统一为 0）。

区域（行×列）：
    0 0 0             ← g1: (0,0)(0,1)(0,2)  这三格不能同时为锥桶
    0 0 0
    0 0 0 g g         ← g2: (2,3)(2,4)       这两格不能同时为锥桶
    0 0 0 0 x         ← x: (3,4) 不可放锥桶，但参与连通性

左上角(0,0): (-2.25, 1.75)
右下角(3,4): (-0.25, 0.25)
步长: 0.5 m

约束：
  1) 放 5 个锥桶后，剩余空格（含 x 和门的空格）必须全部 4-连通；
  2) g1 组三格不能全为锥桶；
  3) g2 组两格不能全为锥桶。

运行：rosrun <pkg> random_place_cones.py
可选环境变量：CONE_SEED=42 固定随机种子
"""

import math
import os
import random

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

# ------------------ 参数 ------------------
GRID_ROWS, GRID_COLS = 4, 5
X0, Y0 = -2.25, 1.75          # 左上角世界坐标
DX, DY = 0.5, 0.5             # 列/行步长
X_CELL = (3, 4)               # 不可放锥桶的 x 格 (row, col)
G1_CELLS = [(0, 0), (0, 1), (0, 2)]   # 门组 g1：不能全为锥桶
G2_CELLS = [(2, 3), (2, 4)]           # 门组 g2：不能全为锥桶
N_CONES = 5                   # 生成数量
OLD_CONE_NAME = "Construction Cone"   # world 中原有锥桶名（先删掉）

CONE_SCALE = 5                # 与 world 中一致；若觉得太大/太小可改

CONE_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='{name}'>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <mesh>
            <scale>{s} {s} {s}</scale>
            <uri>model://construction_cone/meshes/construction_cone.dae</uri>
          </mesh>
        </geometry>
        <max_contacts>10</max_contacts>
        <surface>
          <contact><ode/></contact>
          <bounce/>
          <friction><torsional><ode/></torsional><ode/></friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <mesh>
            <scale>{s} {s} {s}</scale>
            <uri>model://construction_cone/meshes/construction_cone.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz>
          <izz>1</izz>
        </inertia>
        <mass>1</mass>
      </inertial>
    </link>
  </model>
</sdf>
"""
# ------------------------------------------


# ---------- 网格工具 ----------
def cell_exists(r, c):
    if r < 0 or r >= GRID_ROWS or c < 0 or c >= GRID_COLS:
        return False
    if r in (0, 1):
        return c < 3          # 前两行只有 3 列
    return True


def all_cells():
    return [(r, c) for r in range(GRID_ROWS) for c in range(GRID_COLS) if cell_exists(r, c)]


def cell_xy(r, c):
    return X0 + DX * c, Y0 - DY * r


def neighbors(r, c):
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if cell_exists(nr, nc):
            yield nr, nc


def is_connected(free):
    """free 是否全部 4-连通。"""
    free = set(free)
    if not free:
        return True
    start = next(iter(free))
    seen, stack = {start}, [start]
    while stack:
        r, c = stack.pop()
        for nr, nc in neighbors(r, c):
            if (nr, nc) in free and (nr, nc) not in seen:
                seen.add((nr, nc))
                stack.append((nr, nc))
    return seen == free


def pick_cone_cells(n_cones, max_tries=50000):
    cells = all_cells()
    candidates = [c for c in cells if c != X_CELL]
    for _ in range(max_tries):
        chosen = set(random.sample(candidates, n_cones))

        # 约束 2：g1 三格不能全为锥桶
        if all(c in chosen for c in G1_CELLS):
            continue
        # 约束 3：g2 两格不能全为锥桶
        if all(c in chosen for c in G2_CELLS):
            continue

        # 约束 1：剩余空格（含 x、含门留下的空格）必须 4-连通
        free = [c for c in cells if c not in chosen]
        if is_connected(free):
            return list(chosen)
    raise RuntimeError("无法找到满足约束的摆放方案")


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


# ---------- 主流程 ----------
def main():
    rospy.init_node("random_place_cones", anonymous=True)

    seed = int(os.environ.get("CONE_SEED", "0"))
    if seed:
        random.seed(seed)
        rospy.loginfo("使用固定随机种子 seed=%d", seed)

    rospy.loginfo("等待 Gazebo 服务 ...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30)
    rospy.wait_for_service("/gazebo/delete_model", timeout=30)
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    # 先删掉 world 里原有的锥桶，避免“多出一个”
    try:
        resp = delete_srv(model_name=OLD_CONE_NAME)
        if resp.success:
            rospy.loginfo("已删除原有 %s", OLD_CONE_NAME)
        else:
            rospy.logwarn("删除 %s 失败: %s", OLD_CONE_NAME, resp.status_message)
    except rospy.ServiceException as e:
        rospy.logwarn("delete_model 调用失败: %s", e)

    # 1) 抽 5 个格点，满足 g1/g2 门控 + 4-连通
    cones = pick_cone_cells(N_CONES)
    rospy.loginfo("选中格子: %s", sorted(cones))
    g1_occ = [c for c in G1_CELLS if c in cones]
    g2_occ = [c for c in G2_CELLS if c in cones]
    rospy.loginfo("g1 占用 %d/3 %s, g2 占用 %d/2 %s",
                  len(g1_occ), g1_occ, len(g2_occ), g2_occ)

    # 2) 生成锥桶（yaw 统一为 0）
    for i, (r, c) in enumerate(sorted(cones)):
        name = "construction_cone_%d" % i
        x, y = cell_xy(r, c)
        yaw = 0.0                                   # ← 统一 yaw
        sdf = CONE_SDF_TEMPLATE.format(name=name, s=CONE_SCALE)
        pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=yaw_to_quat(yaw),
        )
        try:
            resp = spawn_srv(
                model_name=name,
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world",
            )
            if resp.success:
                rospy.loginfo("锥桶 #%d  grid(%d,%d)  world(%.2f, %.2f)  yaw=0",
                              i, r, c, x, y)
            else:
                rospy.logwarn("生成 %s 失败: %s", name, resp.status_message)
        except rospy.ServiceException as e:
            rospy.logwarn("spawn_sdf_model 调用失败: %s", e)

    rospy.loginfo("锥桶摆放完成 ✅")


if __name__ == "__main__":
    main()