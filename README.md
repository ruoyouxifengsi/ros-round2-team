# 第二轮 ROS 智能车：控制 baseline 与图像协作入口

## 2026-11-12 当前交付：入口与停靠提速，三个布局通过

继续使用 `official/fast-control`，新增 **`--efficient-approach`**，与 `sprint + continuous-bends + low-com` 组合使用：

- 锥桶入口先用带碰撞预测的短距离横移离开边界，再检查车身转动范围并沿短方向转身，避免此前部分布局绕约 270° 才对准入口。穿桶仍使用 TEB。
- QR 接近前段上限从 0.10 提至 0.20 m/s，最后 12 厘米恢复低速；QR 退出上限 0.22 m/s，终点接近上限 0.28 m/s，边走边纠偏并按剩余距离减速。
- 最终停靠容差、车身尺寸、姿态保护及官方引擎物理参数保持原值。上一版完整代码保留在 `continuous-before-docking-v2`；省略新开关即可关闭本次优化。

相对于上一版连续过弯 sprint，三个相同官方布局的完整控制路线对照如下（均为仿真秒）：

| 官方种子 | 上一版 | 新版 | 耗时减少 |
|---|---:|---:|---:|
| 42 | 82.135 | 69.822 | 15.0% |
| 1 | 81.141 | 78.386 | 3.4% |
| 2 | 81.041 | 77.298 | 4.6% |

平均 **81.44 → 75.17 秒，减少 7.7%**。最终同一控制代码下三个布局各一次，**3/3 通过**；车桶接触和姿态保护均为 0，最大倾角 0.527°，QR / 终点入框及必要停稳检查通过。没有超过 5 毫米的桶位移或导航卡住；规划重试分别为 7、10、8 次。种子 1 原本入口就能短转，新入口反而多用约 1.8 秒，整轮仍更快；不承诺每个路段都提速。

[对照证据](round2_baseline/evidence/approach_20261112/summary.json) 核对了每组桶位置、物理参数和低质心模型一致，以及最终三轮导航代码/配置一致；同时保留两个入口开发失败/中止、一次停稳采样边界误判及修正后的原数据复核。控制代码在最终三轮间没有修改，未为修正离线验收再跑仿真。这是三个布局的控制验证，仍未包括图像、播报和正式计时成绩。

在 WSL 仓库根目录运行，每次更换输出目录：

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/scripts/run_trial.py \
  round2_baseline/evidence/local_efficient_practice_01 \
  --low-com --cones --seed 42 --speed-profile sprint \
  --continuous-bends --efficient-approach --timeout-clock sim --gui --seconds 350
```

此命令自动布景和发车，跳过图像与播报。队友电脑正式计时联调时，去掉 `--timeout-clock sim`，加 `--timer` 并将 `--seconds` 改为 `600`。比赛不得带开发观察参数 `--audit-cones`、`--contact-observer`。具体改法、局部反例、验证与回退见 [操作记录](round2_baseline/操作记录.md)。下方为此前版本记录。

## 2026-11-12 此前交付：连续过弯与提速，三个布局通过

工作分支仍为 `official/fast-control`。采用 **TEB 行驶避障 + 专门的精确停车控制**，不修改官方引擎或碰撞模型。低质心沿用已获准的版本。

- `--continuous-bends`：普通弯道点附近衔接下一目标，报告 `PASSED`；QR、物块和终点等任务位置仍停稳后报告 `ARRIVED`。仅普通弯道取消目标朝向收敛，不放宽任务停靠精度。
- `--speed-profile sprint`：前进 0.50 m/s、侧向 0.25 m/s、规划角速度 0.60 rad/s、平移加速度 0.65 m/s²；精确停车仍减速，姿态保护阈值不变。
- 本次改动前完整代码保留在 `fast-before-continuous-v1` 标签；`main` 未覆盖。默认仍是原来的 `stable` 和逐点停车，需要显式开启新选项。

固定种子 42 的完整控制路线对照已通过：**123.05 → 82.14 仿真秒，减少 33.3%**。连续弯道局部单独对照（保持原 fast 速度）为 **65.12 → 36.83 仿真秒，减少 43.4%**。

| 官方布局种子 | 新版完整控制耗时（仿真秒） | 车桶接触记录 | 姿态保护 | 最大倾角 |
|---|---:|---:|---:|---:|
| 42 | 82.135 | 0 | 0 | 0.363° |
| 1 | 81.141 | 0 | 0 | 0.425° |
| 2 | 81.041 | 0 | 0 | 0.405° |

最终 **3/3** 通过，必要停靠和终点入框检查均通过，未检测到超过 5 毫米的锥桶位移。三轮分别有 8、7、13 次规划重试，未出现卡住或侧翻。仅种子 42 做了本次完整前后对照；这是三个布局各一次的控制验证，不包括图像、播报或正式计时成绩。[完整证据](round2_baseline/evidence/continuous_20261112/summary.json) 同时核对了三轮控制代码一致、物理参数与低质心模型和对照版一致。

本机无官方计时器练习，在 WSL 的仓库根目录执行，每次换新输出目录：

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/scripts/run_trial.py \
  round2_baseline/evidence/local_continuous_practice_01 \
  --low-com --cones --seed 42 --speed-profile sprint \
  --continuous-bends --timeout-clock sim --gui --seconds 350
```

这是自动布景、自动发车的控制测试，仍跳过图像和播报；正式计时验证添加 `--timer`、去掉 `--timeout-clock sim`，并将 `--seconds` 改为 `600`。开发用的 `--audit-cones` 和 `--contact-observer` 不得带入比赛，它们只用于离线验收，不向导航提供数据。详细复现、三布局检查与回退见 [操作记录](round2_baseline/操作记录.md)。下方章节属于历史版本。

## 2026-11-12 此前交付：官方引擎加速控制

**请使用 `official/fast-control` 分支。组委会不允许修改仿真架构，并行碰撞实验已停用并归档，不能参赛。** 此分支从官方引擎代码建立，不包含并行插件或其加载入口；`main` 保留此前版本。

本机不打开官方计时器的低质心练习，在 WSL、仓库根目录运行：

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/scripts/run_trial.py \
  round2_baseline/evidence/local_official_practice_01 \
  --low-com --cones --seed 42 --speed-profile fast \
  --timeout-clock sim --gui --seconds 600
```

每次更换输出目录。程序准备场景后通过 `/nav_start` 自动发车；不需要点击 Draw。`--timeout-clock sim` 只把练习用的路线、阶段和无进展预算按仿真秒计算，不改变 Gazebo 物理步长、重力或时钟速度。传感器过期、时钟冻结和姿态保护仍保留。

加速档沿用：前进上限 **0.40 m/s**、倒车 **0.18 m/s**、侧向 **0.22 m/s**、平移加速度上限 **0.50 m/s²**。弯道按曲率限速，停车阶段单独减速。控制修复增大行驶中的规划净空，并在目标附近恢复原来的接近设置，降低停停走走的重规划。

原引擎无计时局部复核：首弯 **5.454 仿真秒**、实测峰值速度 **0.399 m/s**、重试 0；二维码入口 **6.977 仿真秒**、重试 2。两段保护均为 0，最大倾角低于 0.1°。这是两段复核，**本次新控制版本尚未在官方引擎下重新完成整轮验收**；不能把归档并行引擎的四布局结果当作其证明。证据见 [运动测量](round2_baseline/evidence/official_fast_20261112/motion.json)。

队友在自己的电脑整轮验收时，使用官方计时器和真实时间预算：

```bash
python3 round2_baseline/scripts/run_trial.py \
  round2_baseline/evidence/local_teammate_full_01 \
  --low-com --cones --seed 42 --speed-profile fast --timer
python3 round2_baseline/scripts/analyze_trial.py \
  round2_baseline/evidence/local_teammate_full_01
```

以上固定种子只用于测试，正式比赛按规则随机布景。测试器拒绝 `--timer --timeout-clock sim` 的组合。完整获取步骤、手动入口和回退说明见 [操作记录](round2_baseline/操作记录.md)。以下 2026-11-11 的整轮结果属于历史版本。

这是供队内协作的 ROS Noetic 项目。控制部分已在 WSL Ubuntu 20.04 跑通连续弯道、QR 停靠、五桶区域、物块停靠和终点停车。**图像识别、语音播报、相机观察姿态尚未完成**；请不要把 `CONTROL_ROUTE_DONE` 当成完整比赛完赛。

## 2026-11-11 提速更新

在稳定版四布局通过后，新增可选 `fast`：保留TEB避障，增加转弯指令调速，提高直行上限并按剩余距离减速停车。低质心版官方种子1、2、3、42各跑一次，**4/4通过**，实际平均耗时从501.65秒降至427.35秒（减少14.8%），仿真行驶总时间减少15.3%。四轮保护0次、无卡住、未发现可测锥桶位移，最大倾角0.456°。种子3仅节省约3%，不能保证每种布局同样收益。

默认速度仍为 `stable`。低质心练习命令添加 `--speed-profile fast` 启用提速；仅此组合完成本次验收。原车模与低质心车模都保留，提速前稳定代码另存标签 `stable-multilayout-v1`。完整对照、方法取舍和初学者步骤见 [操作记录](round2_baseline/操作记录.md)；数据见 [提速证据](round2_baseline/evidence/speed_20261111/summary.json) 和 [逐布局耗时](round2_baseline/evidence/speed_20261111/comparison.json)。准备、图像和播报尚未计入，不能视为完整比赛验收。

## 2026-11-11 稳定性更新

低质心已获用户确认合规。修复了现场新布局暴露的入口转身、锥桶底座漏检、出口目标净空和终点收敛问题。最终相同代码下，官方种子 **1、2、3、42 四次整轮控制验收全部通过**，均未发现可测锥桶位移，姿态保护均为0次；实际耗时约475–541秒。四种布局各一次，不代表任意布局保证；图像和准备时间尚未计入。

详见 [最新操作记录](round2_baseline/操作记录.md) 与 [结果汇总](round2_baseline/evidence/multilayout_20261111/summary.json)。改动前完整版本保留在标签 `baseline-before-multilayout`，原版车模不变。新增 `cone_obstacles.py` 基于激光与官方允许格点补足底座；**不要将开发专用 `audit_cones.py` 或 `--audit-cones` 带入比赛**，其Gazebo位移记录只用于开发复核，不向导航提供数据。

## 先给 agent 的任务

把仓库克隆到 Ubuntu 20.04 / WSL 后，可直接向 agent 提交以下任务：

> 先阅读 README.md、第二次分站赛下发中的规则与仿真教程、round2_baseline/scripts/control.py 和 launch/control.launch。先复现原版控制 baseline，再在新分支实现现场 QR 识别、任务保存、对应物块识别和规定格式播报。识别必须使用本地模型及当前相机图像。将控制程序的两个 IMAGE_SKIPPED 占位替换为有超时、可取消、等待完成的图像任务调用；不得仅订阅状态后让小车继续行驶。默认使用原版车模，保留姿态保护及原有验收。不要修改官方计时器、布景程序、碰撞模型或读取 Gazebo 真值。用户已于2026-11-11确认低质心符合规则；新增底座补偿适用于第二轮官方格点布桶。先报告现有接口和实施方案，再实现并测试；没有实测的结果明确标注。

## 获取仓库与安装

本仓库已公开，队友和 agent 无需登录即可读取与克隆。直接推送仍需仓库写入权限；没有写入权限时可 fork 后提交 PR。不要把 token 写进代码或聊天。

```bash
git clone --branch official/fast-control https://github.com/ruoyouxifengsi/ros-round2-team.git
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

默认启动与默认测试均为官方原版。练习脚本加 `--low-com` 会在本次输出目录生成低质心模型并单独加载，不覆盖官方文件。用户已于2026-11-11确认低质心版符合第二轮规则。不要通过修改导航 footprint 冒充车模尺寸变小。

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
7. 当前控制总超时 540 秒、穿桶阶段 150 秒，其余导航阶段通常 90 秒，无进展保护 28 秒；加入识别后重新预算。规则总计时 10 分钟含准备，不能简单延长超时就当合规。ROS 仿真时间与墙钟时间也要分开。

识别只能使用现场传感器与本地模型；禁止预设答案、云端代答，禁止导航/识别读取官方提供代码之外的 Gazebo 真值。控制用 `/odom`、激光及 TF；不要因为计时器或布景程序能读取真值而给识别模块同样权限。本轮无需抓取物块。

## 历史单布局结果与基础测试

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
python3 round2_baseline/scripts/test_cone_obstacles.py
python3 round2_baseline/scripts/verify_assets.py
```

本次协作包已在新的 `round2_team_ws` 中编译通过，姿态保护、直线停车几何自检及官方文件校验通过。尚未在一台全新电脑安装 ROS 后从零验证；整轮通过数据来自此前本机实验。保存的 source_hashes 是当时实验的源码摘要，安装脚本等已为移植调整，不代表当前整个仓库与历史快照逐字节一致。

图像完成后的验收应包含：真实相机识别与播报、任务传递、取消与超时、两处停车等待、收臂后安全行驶、不同桶布局、官方 Draw 全程以及完整比赛总时间。不要用现有 `control_baseline_pass` 证明图像也完成。

## 协作约定与材料

`main` 保留可复现控制基线，图像工作在 `feature/vision-integration` 分支，通过 PR 合并。默认仍保留原版入口，低质心测试显式加 `--low-com`；不要提交 token、模型权重、大型 rosbag 或个人环境目录。第三方官方模型与规则保留原来源和许可，不因控制包声明 MIT 就视为全部素材都可公开再分发；本仓库按所有者要求公开供协作使用；公开可读不等于授予第三方素材额外许可。

官方材料在 `第二次分站赛下发/`；车模及计时器在其 `src/`。`round2_baseline/操作记录.md` 是原开发机历史操作记录，其中 D 盘路径、工作空间名及完整证据目录不一定存在于本仓库，队友以本 README 的相对路径和 `round2_team_ws` 为准。
