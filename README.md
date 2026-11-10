# 第二轮 ROS 智能车：控制 baseline 与图像协作入口

这是供队内协作的 ROS Noetic 项目。控制部分已在 WSL Ubuntu 20.04 跑通连续弯道、QR 停靠、五桶区域、物块停靠和终点停车。**图像识别、语音播报、相机观察姿态尚未完成**；请不要把 `CONTROL_ROUTE_DONE` 当成完整比赛完赛。

## 先给 agent 的任务

把仓库克隆到 Ubuntu 20.04 / WSL 后，可直接向 agent 提交以下任务：

> 先阅读 README.md、第二次分站赛下发中的规则与仿真教程、round2_baseline/scripts/control.py 和 launch/control.launch。先复现原版控制 baseline，再在新分支实现现场 QR 识别、任务保存、对应物块识别和规定格式播报。识别必须使用本地模型及当前相机图像。将控制程序的两个 IMAGE_SKIPPED 占位替换为有超时、可取消、等待完成的图像任务调用；不得仅订阅状态后让小车继续行驶。默认使用原版车模，保留姿态保护及原有验收。不要修改官方计时器、布景程序、碰撞模型或读取 Gazebo 真值。低质心版只用于实验，参赛许可尚待确认。先报告现有接口和实施方案，再实现并测试；没有实测的结果明确标注。

## 获取仓库与安装

私有仓库需要先由仓库所有者邀请你的 GitHub 账号并接受邀请。agent 不会自动获得你的账号权限；不要把 token 写进代码或聊天。

```bash
gh auth login
gh repo clone ruoyouxifengsi/ros-round2-team
cd ros-round2-team
git switch -c feature/vision-integration
```

已验证环境：WSL2 Ubuntu 20.04、ROS Noetic、Gazebo 11。其他 ROS 版本未经验证。Windows 需 WSLg 或可用的 X 显示服务；官方计时器使用 Tk，即使隐藏 Gazebo 也需要显示环境。以下命令在 WSL 终端执行，假设 `/opt/ros/noetic/setup.bash` 已存在：

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-ros-controllers ros-noetic-move-base ros-noetic-teb-local-planner \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher-gui \
  python3-tk python3-yaml python3-numpy python3-matplotlib python3-rospkg
bash round2_baseline/scripts/setup_wsl.sh
source ~/round2_team_ws/devel/setup.bash
```

安装脚本创建独立的 `~/round2_team_ws`，复制官方包、链接控制包，补齐锥桶与白线模型并编译；不会覆盖已有官方包。需要换工作空间时先 `export ROUND2_WORKSPACE="$HOME/另一个工作空间"`，后续 source 相应路径。每开一个终端都要 source。没有 ROS Noetic 的电脑需先安装该环境，本仓库不安装操作系统或 ROS 发行版。

## 两种版本如何选择

控制代码共用，两份车模分别在 `round2_baseline/variants/original.urdf` 和 `low_com.urdf`。唯一动力学参数差异是 `base_link/inertial/origin` 的 z：`0.045754 → 0.02`；重量、惯性矩阵、碰撞和外观均未改变。

默认启动与默认测试均为官方原版。练习脚本加 `--low-com` 会在本次输出目录生成低质心模型并单独加载，不覆盖官方文件。低质心版目前**未获第二轮参赛许可确认**。不要通过修改导航 footprint 冒充车模尺寸变小。

```bash
# 在仓库根目录，先关闭自己此前启动的同项目仿真
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/scripts/run_trial.py round2_baseline/evidence/local_original42 --cones --seed 42 --timer
python3 round2_baseline/scripts/analyze_trial.py round2_baseline/evidence/local_original42

# 单独运行低质心版，不能与上一条同时运行
python3 round2_baseline/scripts/run_trial.py round2_baseline/evidence/local_lowcom42 --cones --seed 42 --timer --low-com
python3 round2_baseline/scripts/analyze_trial.py round2_baseline/evidence/local_lowcom42
```

每次输出目录必须不存在；重复测试请换名字。测试器使用独立 ROS 11321 / Gazebo 11355 端口并清理自己创建的进程。`--seed 42` 仅为练习，不用于比赛。`--debug-*` 是单段诊断，不能算整轮通过。无显示环境时先解决 DISPLAY/WSLg，不能去掉官方计时器后声称验证了比赛发车。

手动原版入口：`roslaunch round2_baseline baseline.launch`。等待 `/baseline/ready` 为 true，按规则取得发车许可后运行 `rosrun car3 random_cones.py` 和 `rosrun car3 random_place.py`，再在官方计时器点击 Draw。比赛发车后不得操作电脑。低质心版当前提供测试器入口，未改动正式手动入口。

## 图像队友在哪里接入

| 文件或接口 | 当前作用 |
|---|---|
| `round2_baseline/scripts/control.py` 的 `Control.run()` | 顺序运行路线；到达 `qr` / `blocks` 并停稳后只报告 `IMAGE_SKIPPED`，立即继续 |
| `launch/control.launch` | 18 个目标及控制参数；坐标在 odom 中，单位米/弧度 |
| `/camera/rgb/image_raw` | 机械臂彩色相机，`sensor_msgs/Image` |
| `/depth_camera/depth/image_raw` | 深度图，可按需使用；不是已完成的识别接口 |
| `/arm_controller/command` | `trajectory_msgs/JointTrajectory`，机械臂 5 个关节；见官方教程 |
| `/nav_start` | `std_srvs/SetBool`：true 发车，false 取消；由官方计时器调用 |
| `/baseline/status` | `std_msgs/String` 中的 JSON，字段 state/detail/stamp/control_only |
| `/baseline/ready` | `std_msgs/Bool` 发车就绪 |
| `/nav_cmd_vel → 姿态保护 → /cmd_vel` | 唯一底盘速度链路，图像模块不应另外发布速度 |

目前没有图像服务实现，也没有二维码识别包、OCR 权重或播报音频。推荐新建独立 catkin 包 `round2_vision`：为 QR 与物块任务提供有界的请求/结果接口，控制在停车后请求并等待结果，完成或明确失败后才进入下一阶段。若用 action，可实现取消和进度；接口名称及消息类型由联调实现确定，不要把建议误当作现有接口。

接入必须处理：

1. QR 阶段移动机械臂到观察姿态，等待稳定后取本次现场图像，识别任务并播报。例如“前往车间获取日用品”。
2. 保存任务类别与识别证据（时间、原图、识别结果），供后续物块阶段和裁判核验；不能用随机布景日志获取答案。
3. 物块阶段使用保存的任务匹配本次相机图像中的物块，再播报。例如“取得日用品，现在前往对应生产车间。”格式以官方规则为准。
4. 在两个占位处阻塞后续导航，但持续响应取消、保护和超时；不能仅订阅 IMAGE_SKIPPED，因为当前程序报告后会立即离开。
5. 失败路径必须明确：按赛事规则可继续后续任务，但不能伪造识别成功。增加任务状态后同步修改 `report()` 中写死的 `control_only=True` 和验收脚本，保留控制与完整比赛两种判定。
6. 当前机械臂收起；观察姿态可能改变车身轮廓、重心和雷达遮挡。驶离前恢复并确认收臂，展开姿态不能直接沿用收臂 footprint 假设。
7. 当前控制总超时 540 秒、阶段 90 秒，无进展保护 28 秒；加入识别后重新预算。规则总计时 10 分钟含准备，不能简单延长超时就当合规。ROS 仿真时间与墙钟时间也要分开。

识别只能使用现场传感器与本地模型；禁止预设答案、云端代答，禁止导航/识别读取官方提供代码之外的 Gazebo 真值。控制用 `/odom`、激光及 TF；不要因为计时器或布景程序能读取真值而给识别模块同样权限。本轮无需抓取物块。

## 结果与测试

| 已保存结果（同种子42，各一次） | 原版 | 低质心 |
|---|---:|---:|
| 控制目标 / 入框检查 | 18/18，通过 | 18/18，通过 |
| 姿态保护次数 | 11 | 0 |
| 最大倾角 | 3.18° | 0.27° |
| 实际耗时 | 497.39秒 | 506.40秒 |
| 仿真行驶时间 | 166.805秒 | 151.400秒 |

低质心版本布局更稳定，但没有证明实际用时更短或任意布局都成功。证据摘要在 `round2_baseline/evidence/`；未上传大量调试日志、逐帧地图和历史失败记录。官方模型与计时器文件保持原样。首次安装后先运行：

```bash
python3 round2_baseline/scripts/attitude_guard.py --self-test
python3 round2_baseline/scripts/test_docking.py
python3 round2_baseline/scripts/verify_assets.py
```

图像完成后的验收应包含：真实相机识别与播报、任务传递、取消与超时、两处停车等待、收臂后安全行驶、不同桶布局、官方 Draw 全程以及完整比赛总时间。不要用现有 `control_baseline_pass` 证明图像也完成。

## 协作约定与材料

`main` 保留可复现控制基线，图像工作在 `feature/vision-integration` 分支，通过 PR 合并。模型获准前不切换默认版本；不要提交 token、模型权重、大型 rosbag 或个人环境目录。第三方官方模型与规则保留原来源和许可，不因控制包声明 MIT 就视为全部素材都可公开再分发；本仓库按队内私有协作用途提供。

官方材料在 `第二次分站赛下发/`；车模及计时器在其 `src/`。`round2_baseline/操作记录.md` 是原开发机历史操作记录，其中 D 盘路径、工作空间名及完整证据目录不一定存在于本仓库，队友以本 README 的相对路径和 `round2_team_ws` 为准。
