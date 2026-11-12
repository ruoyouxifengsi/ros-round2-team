# 碰撞并行实验（独立分支，不作为正式比赛默认入口）

> **已停用：用户于 2026-11-12 确认组委会不允许修改架构。以下构建命令仅作历史记录，不用于比赛或后续验收。** 多布局实验完成种子 42、1、2、3 四轮；第五轮按用户要求取消，第六轮未启动。记录在 `evidence/parallel_multilayout_archived_20261112`，不得把这些结果当作官方引擎的验收。

本实验针对 Gazebo 11.15.1 的 ODE 碰撞计算：把互不依赖的碰撞对交给多个 CPU 线程计算，然后按原来顺序创建接触约束。它不是调整车速，也不修改碰撞网格、重力、步长、摩擦、求解迭代数或接触点上限。低质心版本仍只使用此前获准的质心改动。

用户已授权尝试修改仿真架构；**组委会是否允许修改 Gazebo 碰撞执行方式尚未确认**。本分支仅供实验。系统安装的 Gazebo 和仓库原版车模均保留；不传实验参数就使用原引擎，不需要卸载或覆盖系统库。

## 2026-11-12 本机实测

WSL Ubuntu 20.04 分配 20 个逻辑处理器，无 Gazebo 界面；同一低质心车模、fast 控制参数、官方种子 42。以下短段从 `MOVING` 到完成计真实时间，全部是起点第一弯道：

| 方式 | 真实耗时 | 实时率 | Gazebo CPU 中位数 |
|---|---:|---:|---:|
| 原引擎，第一次 | 33.87 秒 | 0.164 | 124% |
| 原引擎，独立补测 | 35.21 秒 | 0.158 | 见证据 |
| 实验插件，1 线程 | 35.86 秒 | 0.160 | 125% |
| 实验插件，4 线程 | 10.07 秒 | 0.577 | 309% |
| 实验插件，4 线程，独立复测 | 9.56 秒 | 0.619 | 317% |
| 实验插件，8 线程 | 12.30 秒 | 0.474 | 315% |

Linux 的 100% CPU 约代表一个逻辑处理器的累计负载。WSL 分配 20 个处理器和插件设置 4 个碰撞线程是两个不同选项。此次两次 4 线程首弯均值比两次有效原版对照均值少约 72% 时间；8 线程没有进一步改善。这是短段少量样本，不是整轮 3 倍加速的结论。

4 线程随后使用官方计时器发车完成种子 42 的 **18 个控制目标**。测试器耗时 **361.41 秒**；其中移动开始至完成为 357.81 秒、126.776 仿真秒，实时率 0.354，Gazebo CPU 中位数 321%。二维码区和终点车身包络入框、终点停稳检查通过；最大倾角 **0.435°**、姿态保护 **0 次**，开发观察未发现超过 5 毫米的锥桶位移。图像及播报仍跳过。

原算法对照：早期短段逐步核对至少 **48,757 组碰撞对**、差异 0；整轮每 100 个物理更新抽查一次，日志最后记录 **5,962 组核对、差异 0**，并行结果已实际采用至少 595,829 次。原版和实验版有记录的运行时物理属性完全一致，官方车模、世界、布景和计时器文件校验通过。数据在 [短段证据](../../evidence/performance_20261112/parallel_short_trials.json) 和 [整轮证据](../../evidence/performance_20261112/parallel_full_trial.json)。

保留的限制和失败：

- 整轮只有一个官方随机布局通过，不能称为全部布局稳定。整轮有 **81 条轨迹不可行后的重规划警告**，最终完成且没有无进展失败；碰撞并行不会自动解决规划器的全部停顿。
- 一次补测在发车前 Gazebo 以 255 退出，未进入插件更新；具体原因未确认。该次没有速度数据，失败保存在 [启动失败记录](../../evidence/performance_20261112/parallel_startup_failure.json)。随后独立启动的整轮和最终首弯复测都正常完成。
- `local_parallel_benchmark/4_stock` 的 37.52 秒与编译任务重叠，排除出收益计算；用独立补测 35.21 秒替代。早期只加载库、没有采用并行结果的试验也不计入收益。
- 运行时物理配置一致和抽样接触结果一致，是有限验证，不是所有场景、所有时序的完全等价证明；不替代组委会对架构修改的许可。

## 原理与边界

- 独立系统插件注册 `round2_parallel_ode`，继承原 ODE 引擎，仅替换碰撞更新。普通队列和网格队列都会处理，因为当前实际网格碰撞也可能进入普通队列。
- 每组任务使用独立几何对象副本；网格数据只读共享，碰撞临时缓存按线程隔离。OpenMP 分发原 `dCollide` 调用；实际接触约束仍用固定版本的原 `ODEPhysics::Collide` 代码，按原顺序串行创建。
- 支持无自定义回调的三角网格、平面、盒子、球、胶囊和圆柱；其余类型保留串行。不会拆开单个特别复杂的碰撞对，也不并行化整个物理求解器。
- 多线程需要 ODE 的 `ODE_EXT_mt_collisions` 支持；缺失时拒绝启动。当前只验证一个世界、一个物理引擎实例。新 Gazebo 版本、多世界、自定义几何/回调都未经验证。
- `--verify-collisions` 会对实际处理的碰撞对额外运行原算法，比较接触点数量、顺序、面编号、位置、法向和深度，误差容限 `1e-10`；发现差异即中止。默认每步检查。`--collision-verify-stride 100` 表示每 100 个物理更新抽查一次，不能解读为全量等价证明。
- 日志 `pairs` 与 `cache_hits` 必须大于零，才证明计算结果被采用。只有 “adapter active” 不够。周期统计是运行中的累计下界，不等于整场最后精确总数。
- 使用 `OMP_WAIT_POLICY=PASSIVE`，让等待任务的线程休眠，避免空转把 CPU 占用率抬高。CPU 占用率不是验收指标，主要看相同路线的真实耗时与实时率。

源码固定为 [Gazebo 11.15.1](https://github.com/gazebosim/gazebo-classic/tree/b22c6e15e52299865b31093b8feebc9ca19e26e8)。`prepare_source.py` 从该版本取出原接触约束代码，避免手工重写摩擦、接触点选择和约束行为。它仍是底层实验实现，短段一致性与单布局整轮通过不能证明所有场景完全等价。

## 在 WSL 中编译

先按仓库 README 配置 Ubuntu 20.04 / ROS Noetic 工作空间，再在 **WSL 终端**、仓库根目录运行。下面是第一次下载源码的命令；已有该目录时直接执行编译，脚本会检查版本，不要覆盖自己的目录。

```bash
sudo apt install g++ pkg-config libgazebo11-dev libtbb-dev
mkdir -p ~/round2_engine_experiment
git clone --depth 1 --branch gazebo11_11.15.1 \
  https://github.com/gazebosim/gazebo-classic.git \
  ~/round2_engine_experiment/source
bash round2_baseline/experimental/parallel_collision/build.sh
```

生成 `~/round2_engine_experiment/build/libround2_parallel_collision.so`。这一步只编译插件，不重编或替换系统 Gazebo。运行实验期间不要重新编译正在使用的库，也不要同时运行其他仿真。

## 复现短段对比

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/experimental/parallel_collision/benchmark.py \
  round2_baseline/evidence/local_parallel_mytest
```

脚本依次测试原引擎、1/4/8 个碰撞线程、原引擎复测。每次独立启动并关闭场景；不同时运行。结果在输出目录的 `comparison.json`。`wall` 是开始移动到完成的真实秒数；`sim` 是仿真秒数；`rtf=sim/wall`，越接近 1 越接近实时。`elapsed_wall` 还包含发车等待和轮询，不能混用两种口径。以上只测试起点第一个弯道，不能作为整轮完成证明。

## 实验版整轮与回退

以下命令用于开发验证，启用官方计时器发车、低质心和加速控制。每次更换输出目录名；`--gui` 可选，用来显示 Gazebo，但界面开销与无界面对比测试不同。

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/scripts/run_trial.py \
  round2_baseline/evidence/local_parallel_full_mytest \
  --low-com --cones --seed 42 --speed-profile fast --timer \
  --profile-performance \
  --experimental-collision-library "$HOME/round2_engine_experiment/build/libround2_parallel_collision.so" \
  --collision-threads 4 --verify-collisions --collision-verify-stride 100
python3 round2_baseline/scripts/analyze_trial.py \
  round2_baseline/evidence/local_parallel_full_mytest
```

恢复原引擎：从上述命令删除 `--experimental-collision-library ...`、`--collision-threads ...`、`--verify-collisions` 和 `--collision-verify-stride ...`。不需要重装、不需要修改官方模型；默认启动入口始终使用原引擎。比赛是否能用实验插件须以组委会答复为准。

`--audit-cones` 另有开发专用锥桶位移观察功能，读取 Gazebo 状态并只写日志，不向导航反馈；不得带入正式比赛。它能发现可测位移，不能把零位移当作绝无接触。

控制整轮仍跳过图像识别和播报，`CONTROL_ROUTE_DONE` 不代表完整比赛完赛。正式接入图像后必须重新验收全程和比赛时间预算。

## 连续启停和多布局复测

以下命令每次使用新的输出目录，顺序执行 6 次独立启动/关闭检查，再执行 6 轮控制路线，包含 4 种布局和 2 次重复。各轮使用官方随机布桶脚本；导航只读取现场传感器。不要同时启动其他同端口的练习程序。

```bash
source ~/round2_team_ws/devel/setup.bash
python3 round2_baseline/experimental/parallel_collision/validate_layouts.py \
  round2_baseline/evidence/local_parallel_layouts_mytest \
  --startup-repeats 6 --seeds 42 1 2 3 42 3
```

`summary.json` 随每轮结束更新。`scope=startup` 只说明启动、传感器就绪和清理通过；`scope=full` 才参与整轮统计。脚本遇到失败立即停止，保留失败分母和日志，不自动覆盖或重跑。修复后换新目录开始下一批。脚本使用每轮不超过 10 条规划不可行警告作为开发质量门槛，这不是官方计分规则；还必须同时通过完整路线、停车、保护、接触、位移和清理检查。

开发用 `--audit-cones` 在并行插件中记录 `cone_contacts.jsonl`：每个车体碰撞部件与锥桶的接触开始/结束、接触点、仿真时刻，以及覆盖统计。`contact_episodes` 是按碰撞部件对记录的连续接触段，不能直接等同裁判扣分次数。`unsupported>0` 或记录缺失时，不得把结果写成“0 次接触”。它仅写文件，不向控制节点提供 Gazebo 真值；正式比赛不启用该观察功能。

记录器有独立的相交/分离自检，不修改官方世界文件：

```bash
python3 round2_baseline/experimental/parallel_collision/test_contact_audit.py \
  round2_baseline/evidence/local_contact_check_mytest
python3 round2_baseline/scripts/test_trial_processes.py
```

启动前会检查测试端口，发现已有程序占用时拒绝继续，不结束不属于本轮的程序。退出时清理本轮创建的独立子进程，按 PID 和启动时间核对归属；清理结果写进 `result.json`。若仍失败，保留该目录的 `simulation.log`，其中已启用 Gazebo 详细输出。
