# so101-training

SO-101 小项目工作区，用于 SO-101 机械臂的数据采集与 ACT 入门训练。所有项目文件、数据、校准记录、
模型缓存和训练输出都放在本工作区，依赖安装在独立 Conda 环境中。

GitHub 仓库包含脚本、测试代码、配置、文档与 `lerobot/` 源码快照（含本地相机示例），
保留上游许可证及依赖锁定文件，不上传数据集、缓存、模型权重或上游测试素材。
`data/`、`outputs/`、`logs/`、`.cache/`、`.tools/` 和 `.local-proxy/` 仅保留在本机，
由 `.gitignore` 排除；下文指向这些目录的链接需在本机查看。
配置和脚本中的绝对路径对应当前工作区，迁移到其他机器时需要调整，并重新准备数据、校准文件和模型。

## 2026-10-15 当前采集入口

终端独立操作请使用 [采集手册](docs/collect-manual.md)。
新入口采用官方同步位置读取与标准 LeRobotDataset 写入，保留本项目的腕部映射、
N 保存后自动复位、R 作废重录和 V 首条确认；舵机保护参数不变。
完整状态检查保留在启动和自动保持/复位阶段，手动采集不再逐周期轮询电压和 Status。
自动保持/复位中的设备故障现进入 `FAULT_PAUSED`：停止目标并尝试释放双臂力矩，
保留窗口和已保存数据；已提交的保存完成后只计数一次。暂停时只读复查，连续两轮新鲜完整反馈正常、
双臂力矩全关且保存完成后，仍需重新按 R，才会从当前实测位置重新规划复位。
故障期间的 R 不排队，反馈恢复不会自动运动；操作步骤见 [故障暂停处理](docs/collect-manual.md#4-故障暂停与异常退出)。

已按用户确认应用 **16 个编码器计数的校准边界容差**（臂关节约 1.41°）。
双臂原始反馈及臂关节实测、映射目标、保持、复位共用对应边界；从臂肘关节由 `[837, 3075]`
放宽为 `[821, 3091]`。编码器仍限制在 0–4095，夹爪归一化目标仍为 0–100；
真实读数不改写，`max_relative_target=null`、PID、保护寄存器和复位速度/加速度不变。
补丁已应用到采集代码，下次启动时生效；此次未启动实机验证新的容差。
134 项离线回归测试及 `--check` 通过，覆盖边界内 16 计数接受、边界外 17 计数拦截、
真实读数保留、保存后复位及既有故障暂停流程。
记录：[回归日志](outputs/position-tolerance-proposal-20261015/applied-regression-tests.log)、
[生效参数](outputs/position-tolerance-proposal-20261015/applied-config-check.log)。

本批数据 `so101_stack_white_on_red_20261015_01` 已完成只读审核，两条均保留为成功演示：

| 演示 | 帧数 | 数据时长 | 实际采集频率 | 画面结论 |
| --- | ---: | ---: | ---: | --- |
| 第 1 条（episode 0） | 1247 | 41.57 秒 | 29.88 Hz | 白块叠在红块上，夹爪松开并撤离 |
| 第 2 条（episode 1） | 1061 | 35.37 秒 | 29.88 Hz | 白块叠在红块上，夹爪松开并撤离 |

Parquet 共 2308 帧；top / wrist 视频完整解码后各 2308 帧，640×480、30 fps，
时间戳与片段边界连续，状态/动作与录制日志一致。逐条抽查关键画面及末帧，未见叠放后掉落。
会话中的 5 次低压暂停均在录制片段之外，未混入训练帧；手动采集大多数帧不读取电压/Status，
因此不能据此认定供电问题已解决。保存、故障暂停后重试及继续采第二条的流程已在本批日志中得到验证。
这些数据采于容差补丁应用之前；原数据未裁剪、覆盖或重写。

可改进处：开头和任务结束后有等待，后续采集可缩短；腕部画面偏软、夹爪占比大，
但顶部视角可确认两次任务成功，本批不因此作废。
证据：[完整性报告](outputs/recording-review/so101_stack_white_on_red_20261015_01/integrity-report.json)、
[画面审核](outputs/recording-review/so101_stack_white_on_red_20261015_01/visual-review.json)、
[第 1 条关键帧](outputs/recording-review/so101_stack_white_on_red_20261015_01/episode-1-overview.jpg)、
[第 2 条关键帧](outputs/recording-review/so101_stack_white_on_red_20261015_01/episode-2-overview.jpg)。

```bash
bash scripts/collect_so101.sh record --name so101_stack_white_on_red_20261015_02 --episodes 2 --check
# 核对有效配置、支撑双臂后，由操作者启动：
bash scripts/collect_so101.sh record --name so101_stack_white_on_red_20261015_02 --episodes 2
# 检查已经完成并退出的 01 批次：
bash scripts/collect_so101.sh status so101_stack_white_on_red_20261015_01
```

新名字对应新数据集，已有目录不会覆盖；清理与异常处理见手册中的 `archive` 操作。
这不是原样运行官方 `lerobot-record`，窗口按键仍使用 S/N/R/V；自动复位流程按用户要求保留。
当前阶段已获得两条经审核的成功演示，尚未训练或评估策略；下一批使用新名字，保留已有数据。
本次操作仅包含代码修改、离线回归及现有数据审核。以下 2026-10-14 内容保留为历史记录。

## 2026-10-14 设备恢复进度

已准备“保留腕部映射、重设初始姿态”入口：`--reuse-wrist-from` 只加载并锁定偏移，
H 保存当前姿态为新起点，R 才复位，新姿态另存 v2，旧文件保留。96 项离线测试通过。
当前文件的偏移已更新为 −76.75°，此前会话为 −95.03°；来源选择仍待用户确认，尚未启动新入口。
候选配置 `configs/so101_home_v2.proposed.json`，记录 `outputs/home-recapture/20261014-211341/`。

最新电压测试：按用户要求完成 120 秒主臂只读巡检，3600 轮、实际约 30 Hz，
夹爪出现 4 次 3.4–3.9 V 低读数，均在下一轮恢复；通信错误为 0，六关节位置读数未变。
异常伴随电压错误位，其中两次独立读取也确认低值；不能认定只是软件毛刺或已排除供电问题。
已将主臂力矩关闭时的单次低压退出改为限时复核：50 ms 内连续两次电压正常且错误位清零后继续，
否则仍停止；复核期间不发新目标、不写训练帧，恢复后使用最新完整反馈。
从臂、使能力矩的主臂、过压、其他故障和通信失败仍停止。PID、保护寄存器及运动参数未改。
93 项离线测试通过，修改后尚未重新启动实机。见 [短时低压处理](docs/direct-home.md#短时低压复核)；
测试记录位于 `outputs/voltage-surveys/20261014-192842/`。

最新设备变更：用户更换了主臂本体和 USB 控制板，原电源及线材继续使用。
旧主臂校准和旧控制板路径不适用于本次替换；从臂未更换，校准保留。
新主臂控制板已识别为 `5AAF220248`（`/dev/ttyACM0`），从臂仍为 `5AB9065574`。
19:03 只读核验新主臂六电机 ID 1–6、型号 777、力矩全关、电压均 4.9 V，无故障或通信错误。
新主臂被动校准已由用户完成并审核通过：1138 个采样点覆盖六关节中位两侧，
腕转 151..3960（约 334.86°）；60 次写入均在白名单内，文件与最终读回一致。
新校准为 `data/calibration/replacement-leader-20261014-185158/leader/my_awesome_leader_arm.json`。
新主臂 PID 保持 16/0/32；启动提案显式传入 `--expected-leader-pid 16 0 32`，只读核对，不改写。
当前 setup 提案为 `configs/so101_replacement_leader_setup.proposed.json`，已通过离线检查，
81 项测试通过。用户批准后于 19:16:58 启动，进入参照等待约 0.90 秒后，新主臂夹爪读到
3.4 V（低于原 4.0 V 阈值，状态码和包错误均为 0），程序保护退出，未开始跟随或复位。
19:17:39 只读确认双臂 12 个关节力矩全关，主臂电压回到 4.9 V；间歇异常来源仍未确定。
见 [本次结果](outputs/adaptive-sessions/20261014-191658/result.json) 和 [当前启动单](docs/direct-home.md#setup-命令审核单)。
旧 `so101_recovery.proposed.json` 保留为历史配置，不能用于新主臂；供电稳定性尚未验收。
记录见 `outputs/device-replacements/20261014-185158-leader/replacement.json`。

腕部映射与一键复位软件已改好：保存腕转偏移和两臂实际初始姿态，按 r 从各自当前位置平滑回位，
无需固定入口或示教回程。新流程见 [腕部映射与一键回位](docs/direct-home.md)。77 项离线测试、
双视频模拟保存及隐藏窗口按钮检查通过；新腕偏移/初始姿态尚待实机采样，复位尚未实机验收。
`--mode setup` 已于 18:45 按用户审核的参数启动；约 16 秒后，在等待腕部参照阶段，
主臂夹爪再次报出 3.7 V 低压故障，程序保护退出，未开始跟随或复位。
18:47 只读确认双臂 12 个关节力矩均关闭；供电异常仍阻碍后续设置。
证据见 [本次启动结果](outputs/adaptive-sessions/20261014-184530/result.json)。

控制界面已修复中文字体、空格对齐和长提示挤占布局问题：双相机等比例缩放、
关节数据分列、提示区可滚动、停止按钮固定显示。见 [界面说明](docs/direct-home.md#界面显示)。

校准流程最新调整：默认人工操作只有设置中位和采集范围；寄存器、文件与故障检查自动完成，
不再要求手动回中位或单独方向确认。两臂均已由用户完成并通过文件/寄存器读回审核；
从臂腕转重新采样后范围为 172..3997，约 336.26°，已覆盖中位两侧。
结果见 [最终校准审核](outputs/calibration-reviews/20261014-173117-final-pair/audit.json)。助手不代替用户运行校准。
旧固定入口回放已从活动状态机移除，旧路径文件保留。

本轮已保存基线并只读核验两臂：12 个电机力矩均关闭，PID 和旧校准寄存器匹配，
这次读取无故障码。用户确认主臂 5 V、从臂 12 V，并要求暂缓低压故障排查；
供电稳定性仍未通过验收。

已完成原始位置越界检查、可恢复 `LIMIT_PAUSED`、异步作废当前演示、首条数据复查入口，
以及单臂被动校准和性能报告工具。
恢复配置已选择两份新校准。逐关节实机跟随/暂停验收、从当前位置回初始姿态的实机验收，
以及 7 条正式采集仍待分阶段推进。

当前操作以 [腕部映射与一键回位](docs/direct-home.md) 为准，校准历史见 [设备恢复说明](docs/device-recovery.md)；新主臂配置是
`configs/so101_replacement_leader_setup.proposed.json`，仍保持 `max_relative_target=null`。
基线/交接位于 `outputs/handoffs/20261014-141553-baseline`，正式数据目录目前为空。
旧 v1 回程保留，不能用于新校准坐标，也不能标记为已通过实机自动回程。

## 重要命令的执行审核

**启动遥操作等重要命令，必须先把完整命令和实际生效的关键参数交给用户审核，
得到明确同意后再执行。** 此规则同样适用于代理通过脚本、配置文件或界面
按钮执行的相同操作，不能通过更换执行入口跳过审核。

需要审核的操作包括启动/重启遥操作、使能扭矩或发送运动目标的诊断、校准、
电机设置、实机数据录制、动作回放、策略实机运行，以及修改限幅、PID、速度、
加速度、扭矩等参数。长时间训练、依赖升级、覆盖或删除数据集、模型、校准
文件等重要操作也须先审核。

提交审核时应说明：

1. 完整命令、所用环境、脚本及配置文件路径。
2. 主从臂端口和校准 ID、相机映射、控制频率、位置单位、目标限幅等生效值。
   脚本内部设置和默认值也要说明，不能只提交一条脚本启动命令。
3. 是否使机械臂运动、使能/关闭扭矩、写舵机寄存器或覆盖文件，以及停止方式。
4. 与上一次已审核配置的差异及修改原因。

代理不得自行增加、修改或取消控制限幅。用户一般性地要求“测试一下”，不代表
已经审核代理自行选择的参数；既有审批只覆盖其明确批准的命令、参数和执行范围。
关键行为发生变化时重新审核。普通代码/日志读取、设备枚举、只读状态诊断和
已授权文档修改可继续执行；只读诊断不得写入运动目标或修改控制参数。
用户明确要求停止当前运动时，立即停止。

供代理遵循的同一约束已写入工作区 `AGENTS.md`。

## 位置与版本

- 工作区：`/home/pangk/workspaces/so101-training`
- Conda 环境：`lerobot-so101`
- 环境位置：`/home/pangk/miniforge3/envs/lerobot-so101`
- Python：3.12.15
- ffmpeg：7.1.1
- LeRobot：Seeed-Projects/lerobot，0.6.2
- PyTorch：2.11.0，CUDA 12.8 构建
- 源码提交：`4aaff99be4a1d81568c08c8f0296b41b40c99ec4`
- 依赖：按 `lerobot/uv.lock` 安装 `feetech`、`training`、`core_scripts`
- 基础策略：ACT；初始阶段为软件准备，目前已进入相机摆位与实机遥操作诊断

## 进入工作区

```bash
source /home/pangk/workspaces/so101-training/activate.sh
```

也可以在终端使用 `conda activate lerobot-so101`。新环境内配置的缓存路径和
Python 隔离变量会随激活生效，并在退出环境时由 Conda 恢复。

用 VS Code 打开：

```bash
code /home/pangk/workspaces/so101-training/so101-training.code-workspace
```

工作区已指定新环境的 Python 解释器。终端中仍建议先运行上述 `source` 命令。

## 环境检查

激活后运行：

```bash
uv run --no-project --python "$CONDA_PREFIX/bin/python" python scripts/check_env.py --act-smoke
# 在 GPU 上验证一次 ACT 参数更新：
python scripts/check_env.py --act-smoke --act-device cuda
lerobot-train --help
lerobot-record --help
lerobot-calibrate --help
```

检查脚本只验证软件导入、ffmpeg、路径隔离、CUDA 和一次使用随机数据的 ACT
前向传播、反向传播与参数更新，不连接机械臂，不下载模型或数据集。

安装与验证记录在 `logs/`。如果需要按本次源码的锁定依赖重新同步：

```bash
bash scripts/sync_dependencies.sh
```

本次为改善下载速度，从清华 PyPI 镜像和上海交大 PyTorch 镜像获取安装包，
每个文件均核对原始 `uv.lock` 中的 SHA-256。校验清单见 `logs/mirror-wheels.json`；
全局 Pip、Conda、Git 和代理配置均保持原状。

本次验证已通过：软件导入路径指向新仓库；视频解码正常；ACT 使用随机数据
在 CPU 上完成一次前向传播、反向传播和参数更新；训练、录制、校准命令的
`--help` 正常；106 个已安装 Python 包的依赖兼容性检查通过。
详细结果见 `logs/environment-check.json`、`logs/pip-check.txt` 和各命令的帮助日志。

与创建前的快照对比，8 个现有仓库的已跟踪文件状态、7 个原有配置/环境历史
文件均未变化；新 LeRobot 仓库也没有源码改动。审计结果见
`logs/isolation-check.json`。

## 目录用途

| 目录 | 用途 |
| --- | --- |
| `lerobot/` | 独立克隆的 LeRobot 源码 |
| `data/` | LeRobot 数据集缓存与校准记录 |
| `outputs/` | 后续模型与训练输出 |
| `scripts/` | 本工作区的安装与检查脚本 |
| `logs/` | 安装日志、验证结果与版本清单 |
| `.cache/` | 本工作区独立的软件包、Hub、Torch 等缓存 |
| `.tools/` | 用于创建环境的独立 Micromamba 工具 |

初次检出跳过了 Git LFS 素材；源码和依赖锁定文件已检出。需要上游测试素材时，
可在本仓库中单独执行 `git lfs pull`，不需要运行全局 `git lfs install`。

## 后续准备

先确认主从机械臂、相机、组装/校准状态，再进行设备识别、遥操作、录制数据集
和 ACT 训练。工作区创建时尚未检测到 `/dev/ttyACM*`、`/dev/ttyUSB*` 或
`/dev/video*`；后续连接情况见下方记录。

GPU 为 RTX 3060 Ti，显存 8 GiB。创建时，系统自动更新了 NVIDIA 驱动，
已加载的旧模块 580.173.02 与磁盘上的新库 580.178.04 不匹配，导致
`nvidia-smi` 和 CUDA 初始化失败。重启前的记录保留在
`logs/environment-check.json` 与 `logs/environment-check.stderr`。

2026-10-10 重启后，内核模块和用户态库已统一为 580.178.04，`nvidia-smi`
恢复正常；驱动诊断记录见 `logs/nvidia-after-reboot.json`。本工作区未修改
系统驱动。新环境的重启后验证结果见 `logs/environment-check-after-reboot.json`。
该次检查已通过：`cuda_available=true`，CUDA 张量运算正常，ACT 在 `cuda:0`
上使用随机数据完成一次前向传播、反向传播和参数更新，损失为有限值。

## 搭积木实验：相机预览（2026-10-12）

主从臂已连接，本次任务是选择相机位置。使用已有 `ffplay` 打开两个独立的
RGB 实时窗口，默认请求 640×480、30 fps；脚本只访问相机，不连接机械臂。

```bash
source /home/pangk/workspaces/so101-training/activate.sh
bash scripts/preview_cameras.sh --grid
```

`--grid` 显示九宫格参考线，省略该参数则显示原始画面。在窗口内按 `F`
切换全屏，按 `Q` 或 `Esc` 关闭该窗口。后续录制前关闭两个预览窗口，释放相机。

| 相机 | 本次 RGB 节点 | 脚本使用的设备路径 |
| --- | --- | --- |
| UGREEN Camera 2K | `/dev/video0` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0` |
| Orbbec Gemini 335 | `/dev/video8` | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0` |

`/dev/videoN` 编号可能随插拔变化。Orbbec 使用 RGB 接口的 `by-path` 链接，
避免与它的深度/红外节点混淆；换 USB 插口后需要检查并更新脚本中的 `ORBBEC` 路径。
此预览仅使用彩色流。日志位于 `logs/preview-ugreen.log` 和
`logs/preview-orbbec.log`；初始参考帧与读取检查在 `outputs/camera-preview/`。

搭积木的初始摆位可用一个斜上方全景视角覆盖取料区、叠放区与夹爪，另一个
较低的侧面视角观察积木高度和叠放接触点。以实际画面确认两个视角的遮挡
互补，并为最高叠放位置留出画面空间。初始参考帧中绿联朝墙，需要转向
操作台；Orbbec 的亮窗背景较多，应调整角度让台面和操作区占据主要画面。

## 双相机实时遥操作（2026-10-12）

用户已固定相机并确认 **Gemini 为 `top` 全局视角**；绿联在当前配置中命名为
`wrist`。使用已有校准，不重新校准：

| 角色 | USB 序列号 | 本次串口 | 校准 ID |
| --- | --- | --- | --- |
| 主臂 leader | `5B79050214` | `/dev/ttyACM0` | `my_awesome_leader_arm` |
| 从臂 follower | `5AB9065574` | `/dev/ttyACM1` | `my_awesome_follower_arm` |

待审核的启动命令（会使能从臂并跟随主臂运动；执行前须展开下述配置交用户审核）：

```bash
bash /home/pangk/workspaces/so101-training/scripts/teleoperate_cameras.sh
```

配置保存在 `configs/teleoperate_cameras.json`，使用稳定的设备路径：
Gemini RGB 当前为 `/dev/video8`，绿联 RGB 当前为 `/dev/video0`；均采用
640×480、30 fps、MJPG。`/dev/video2` 并非本次使用的 Gemini 彩色流。
脚本启动前检查端口占用、两臂校准一致性和舵机故障状态，并将从臂的初始目标
设为其当前实测位置，再启动已安装的 `lerobot-teleoperate`。

本次测试配置为 30 Hz、`max_relative_target=2.0`：每周期相对当前实测位置，
臂关节目标差限制为 2 度，夹爪目标差限制为其归一化范围的 2 个百分点。
该限幅已在后续测试中影响大范围跟随，不能视为已确认合适的常规配置；后续
保留、修改或取消它，都需在执行命令前向用户明确说明并审核。这不是严格的
速度限制。Rerun 显示两路图像与关节数据；此命令不录制数据集。
停止控制请在启动它的终端按 **Ctrl+C**；仅关闭 Rerun 窗口不会停止遥操作。
正常退出时 LeRobot 会关闭从臂扭矩。

本次启动前，两臂各 6 个舵机均正常响应、无故障，校准与文件一致；主臂约
4.9 V，从臂约 11.9–12.0 V。两台相机已通过同时读取验证，启动后最初观察的
650 多个控制周期中，最近的循环频率为 30 Hz，当时未见错误；后续跟随迟缓
和可视化停顿的排查见下一节。运行日志：
`logs/teleoperate-cameras-live.log`。

连接排查记录：从臂曾在 USB 枚举后约 1 秒断开；用户更换 USB 数据线后，
设备保持连接并成功启动遥操作。不要将示例命令中的主从端口顺序或校准 ID
直接套用到本工作区。

## 跟随延迟与关节诊断记录（2026-10-12）

用户反馈主臂大范围移动时从臂跟随很慢，大臂/肘部疑似不动。本次启动脚本所用
`max_relative_target=2.0` 是代理加入的参数：它以**从臂当前实测位置**为基准
截断目标，并不保证每周期实际移动 2 度。日志中出现 5,238 次限幅提示，主臂
目标与限幅后的目标有明显差距。用户据此指出 2 度限幅是大范围跟随受阻的
重要原因，并要求以后重要命令执行前必须审核。

单独测量两臂各 100 次六关节位置读取，主臂和从臂中位耗时均约 0.90 ms，
该次只读测量未发现明显读取瓶颈，记录在 `logs/joint-diagnostic-baseline.json`。
此前一次控制循环耗时约 28.4 秒，附近有 Rerun 连接错误；这与持续的限幅问题
分开记录，尚不能仅据日志断言停顿全部来自某一个环节。

`scripts/joint_monitor.sh` 打开独立六关节诊断窗口，默认只读；显示主臂目标、
实际下发目标、舵机目标回读、从臂实测位置，以及通信耗时、扭矩和故障状态。
界面中的“启动诊断跟随”会使从臂运动，也属于需审核的操作。停止跟随可按
停止按钮或 `Esc`。只读时目标寄存器保留历史值，不代表正在发送动作。

排查期间两臂均能读取、未报告故障码，不能因此直接排除机械或带载执行问题。
本次文档更新不修改现有控制参数，也不授权重新启动运动或取消限幅。

## 已审核的独立视频窗口遥操作（2026-10-12 18:51）

本次经用户明确审核同意后，使用下列命令启动。配置文件名保留 `.proposed`
以对应审核时的原始路径；本条记录表示本次启动已经获批，后续操作仍按前述
审核范围执行。

```bash
bash /home/pangk/workspaces/so101-training/scripts/teleoperate_cameras.sh \
  --config /home/pangk/workspaces/so101-training/configs/teleoperate_direct_preview.proposed.json
```

实际参数为 30 Hz、`max_relative_target=null`、`robot.cameras={}`、
`display_data=false`。取消此前 2 度软件目标差限幅，主从臂沿用原校准 ID、
序列号设备路径和角度单位，PID 为 16/0/32。启动检查通过后，先将从臂目标
设置为当前实测位置，再由 LeRobot 使能跟随。

相机由 `scripts/preview_cameras.sh` 独立显示：左侧 `SO101 top` 为 Gemini
全局视角，右侧 `SO101 wrist` 为绿联，均请求 640×480、30 fps。按 `F`
切换视频窗口全屏；在遥操作终端按 `Ctrl+C` 停止运动，正常退出关闭从臂
扭矩。视频窗口的关闭键只关闭画面。

启动时两臂各 6 个舵机均正常响应且无故障，校准一致。已观察约 1,700 个
控制周期，最近约 33.44 ms/周期（30 Hz）。日志保存在
`logs/teleoperate-direct-preview-live.log`。此记录仅证明通信与控制循环正在
运行，实际跟随效果由现场测试确认。省略 `--config` 仍会使用历史 2 度限幅
配置，因此应完整保留已审核命令中的配置路径。

## 首轮试采集（2026-10-12，已审核并完成）

任务：**抓起白色积木，叠在红色积木上，再松开夹爪。** 用户指定先试录
3 段，每段最多 30 秒，检查数据后再决定批量采集。用户审核后回复“启动”，
已按下列完整命令执行，配置文件名保留 `.proposed` 以对应审核记录：

```bash
bash /home/pangk/workspaces/so101-training/scripts/record_trial.sh \
  --config /home/pangk/workspaces/so101-training/configs/record_stack_white_on_red_trial.proposed.json
```

本次启动已获得用户审核同意，后续启动仍按重要命令审核规则执行。
脚本激活本工作区的 `lerobot-so101` 环境，使用
`uv run --no-project --python "$CONDA_PREFIX/bin/python"` 调用
`scripts/record_with_preview.py`；录制主体仍为本地 LeRobot 的 `record()`。
不会终止其他进程；串口或相机被占用时会拒绝开始。

生效参数：

| 项目 | 本轮值 |
| --- | --- |
| 从臂 | SO101；`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9065574-if00`；ID `my_awesome_follower_arm` |
| 主臂 | SO101；`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050214-if00`；ID `my_awesome_leader_arm` |
| 校准目录 | 从臂 `data/calibration/robots/so_follower`；主臂 `data/calibration/teleoperators/so_leader`，使用各自 ID 的 JSON |
| 控制与数据频率 | 请求 30 Hz；实际频率在录制日志中检查 |
| 位置与目标限幅 | 主从臂 `use_degrees=true`；臂关节为度，夹爪为 0–100；`max_relative_target=null`，沿用上次无 2 度目标差限幅设置 |
| 舵机设置 | 沿用 PID 16/0/32，位置模式 0；Return_Delay_Time=0，Maximum_Acceleration/Acceleration=254；必要时清除 Phase 的 0x10 位 |
| 夹爪设置 | Max_Torque_Limit=500，Protection_Current=250，Overload_Torque=25，沿用 LeRobot 配置 |
| top | Gemini RGB，`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0` |
| wrist | UGREEN，`/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0` |
| 相机格式 | 两路均为 MJPG 640×480、30 fps，输出 RGB；仅彩色，无深度 |
| 录制 | 3 段，每段最多 30 秒；段间 30 秒复位，期间仍遥操，不保存复位帧 |
| 视频编码 | H.264，yuv420p，CRF 23，GOP 2，ultrafast；实时编码，每路 2 线程、30 帧队列 |
| 本地数据集 | `data/datasets/so101_stack_white_on_red_trial_001`；repo_id `local/so101_stack_white_on_red_trial_001` |
| 保存规则 | `push_to_hub=false`，`resume=false`，`no_stamp=true`；目录存在即退出，不覆盖、不追加、不上传 |
| 界面与退出 | 独立双画面窗口，共用录制图像；`display_data=false` 禁用 Rerun；`play_sounds=false`；正常退出关闭从臂扭矩 |

启动后先等待终端 **Enter**。摆好两臂对应的起始姿态及积木后再按键。
预检会检查设备是否空闲、校准匹配和电机错误码；然后**把从臂 Goal_Position
写为当前实测位置**，再调用 LeRobot 连接、配置舵机并使能从臂扭矩，开始遥操录制。
该预检包含目标写入，不是只读诊断。不自动重新校准。

预览约 10 Hz 更新，显示采集程序取得的 top/wrist 图像，以及六关节主臂请求
`action` 与从臂实测 `observation.state`；不是舵机目标寄存器回读。
显示通过容量 1 的非阻塞队列旁路传递，队列满时只跳过预览更新；数据仍由
LeRobot 按请求 30 Hz 采集。界面标注录制/复位阶段及超过 1 秒未更新的画面。
关闭窗口按钮仅最小化，不停止机械臂。

- `n` / 右箭头：录制中提前结束当前段；复位中提前结束等待。
- `r` / 左箭头：丢弃当前待保存段，经过复位后重录。失败演示应使用此键。
- `q` / `Esc`：结束采集并保存当前缓冲段；可能留下未完成演示，检查时需识别。
- 终端 `Ctrl+C`：中断控制循环并执行退出清理；未保存的当前段不保证保留。
  LeRobot 会先完成数据清理再断开机械臂，正常退出时关闭从臂扭矩；这些按键
  不等同于硬件急停。

离线检查命令（不打开相机或串口）：

```bash
bash scripts/record_trial.sh --config configs/record_stack_white_on_red_trial.proposed.json --check
source ./activate.sh
uv run --no-project --python "$CONDA_PREFIX/bin/python" python scripts/check_recording_setup.py
```

已使用模拟图像/关节数据写入 3 段、合计 270 帧，校验 action/state、时间戳、
帧序号并完整解码两路视频各 270 帧；预览处理不修改动作，队列阻塞时不等待。
结果见 `logs/recording-setup-check.json`。模拟测试产物仅位于
`outputs/recording-smoke/`，不能作为真实演示训练。此检查不能代替实机帧率、
画面遮挡、对焦与动作成功率验收。实录后先逐段回看、检查实际周期与数据完整性，
再进行下一轮采集。

### 本轮实录结果

2026-10-12 19:28:02 开始第一段，19:32:28 正常退出；日志确认主从臂及
两路相机均已断开。用户期间重录 4 次，最终保存 3 段，共 2,687 帧。
两臂启动预检均为六电机响应、校准匹配、无错误标志。

| 段号（数据集索引） | 保存帧数 | 日志实测频率 | 抽帧观察 |
| --- | --- | --- | --- |
| 第 1 段（0） | 893 | 29.76 Hz | 结束画面中白色积木仍在原位，未完成堆叠；标记待重录 |
| 第 2 段（1） | 897 | 29.89 Hz | 结束画面显示白块叠在红块上、夹爪已离开；成功候选 |
| 第 3 段（2） | 897 | 29.90 Hz | 结束画面显示白块叠在红块上、夹爪已离开；成功候选 |

三段录制的控制计算均未超过 33.3 ms 的单周期预算；每段墙钟约 30 秒，
元数据中的视频时长按帧数/30 计算，分别为 29.77、29.90、29.90 秒。
结构检查通过：两路 H.264 视频均完整解码 2,687 帧，640×480；action/state
均为有限的六维数据，帧序号连续、时间戳与名义 30 fps 对应。
这证明已保存的数据相互对应，不等于证明相机物理曝光同步或没有重复采集帧。

完整报告与各段双相机抽帧图位于
`outputs/recording-review/so101_stack_white_on_red_trial_001/`；运行日志为
`logs/record-stack-white-on-red-trial.log`。动作质量判断基于每段每相机六张
时间采样图和末帧，不是全视频逐帧审核。原始三段数据均保留，没有删除、
覆盖或上传。第 1 段不应直接作为成功演示纳入训练；尚未启动下一轮采集。

## 第二轮旧采集方案（2026-10-13，历史配置）

下面保留旧入口的参数记录。日志显示此入口后来进入第 4 段，当前数据集元信息
记录 3 段、10,762 帧，但没有正常结束记录。旧目录待按用户要求清理。
后续采用下节新的腕部映射与双臂复位入口，请勿再使用此旧配置启动新采集。

用户指定继续同一搭积木任务，新增 7 段，每段最多 120 秒。
本轮使用独立新数据集，保留首轮试录数据。完整待审核命令：

```bash
bash /home/pangk/workspaces/so101-training/scripts/record_dataset.sh \
  --config /home/pangk/workspaces/so101-training/configs/record_stack_white_on_red_batch_001.proposed.json
```

与首轮配置相比，仅修改下列四个数据集字段：

| 字段 | 首轮 | 本轮 |
| --- | --- | --- |
| `dataset.num_episodes` | 3 | 7 |
| `dataset.episode_time_s` | 30 | 120 |
| `dataset.repo_id` | `local/so101_stack_white_on_red_trial_001` | `local/so101_stack_white_on_red_batch_001` |
| `dataset.root` | 工作区 `data/datasets/so101_stack_white_on_red_trial_001` | 工作区 `data/datasets/so101_stack_white_on_red_batch_001` |

`record_dataset.sh` 使用与首轮相同的环境、缓存隔离和
`scripts/record_with_preview.py` 录制程序；日志单独追加到
`logs/record_stack_white_on_red_batch_001.log`。设备、预检、目标写入、
使能扭矩、录制和退出行为均沿用上节完整审核说明。

- 主臂：序列号路径 `usb-1a86_USB_Single_Serial_5B79050214-if00`，
  ID `my_awesome_leader_arm`；从臂：`usb-1a86_USB_Single_Serial_5AB9065574-if00`，
  ID `my_awesome_follower_arm`，两路径都位于 `/dev/serial/by-id/`。
- 继续使用 `data/calibration/teleoperators/so_leader` 和
  `data/calibration/robots/so_follower` 下对应 ID 的校准文件。
- 30 Hz；臂关节单位为度、夹爪为 0–100；`max_relative_target=null`；
  PID 16/0/32、位置模式 0、Return_Delay_Time=0、加速度设置 254；
  Phase 位及夹爪 500/250/25 寄存器设置均沿用首轮行为。
- Gemini 为 top，绿联为 wrist，设备路径沿用首轮配置；双路均为
  MJPG 640×480、30 fps，保存为 H.264，双画面与关节预览继续启用。
- 段间复位仍为 30 秒，期间继续遥操、不记录复位帧；等待结束会自动开始下一段。
- 启动后等待用户在终端按 Enter，再检查设备和校准，将从臂启动目标写为
  当前实测位置，由 LeRobot 配置并使能从臂扭矩。
- 只保存到本地新目录；目录存在即退出，不覆盖、不追加、不上传；正常退出
  关闭从臂扭矩。`q` / `Esc` 结束整轮，终端 `Ctrl+C` 中断控制并清理。

每段无需录满 120 秒。成功堆叠、松爪并留出短暂稳定画面后，按
`n` / 右箭头结束当前段，再复位积木；复位中按同一键会提前结束等待。
失败时按 `r` / 左箭头丢弃当前待保存段并重录，重录不增加已保存段数。

LeRobot 按 episode 边界读取数据，允许各段长度不同。本地 ACT 的加载器
在片尾越界处生成补齐掩码，动作损失排除这些补齐位置，因此不需要把每段
演示录成等长。动作应完整、采样频率应一致，并尽量避免录入大量等待和复位动作。
此段描述旧入口行为；当前状态及新流程见下节。

## 腕部映射与双臂自动复位（当前新方案）

用户确认：主臂握把竖直对应从臂夹爪横向，之后同向等幅跟随；示教一次固定起点和
空旷回程入口到起点的路径。成功后按 `n` 保存并复位两臂，手动摆放积木后按 `s`
开始下一段。主臂使用独立 5 V 电源。重新采集 7 段，每段最多 120 秒。

软件入口 `scripts/so101_session.sh`、待审核配置
`configs/so101_adaptive.proposed.json` 已实现。`--mode teach` 保存参照及路径，
`--mode teleop` 验证映射和回放，`--mode record` 正式录制。三个阶段均须先审核。
`--check` 仅读取配置及校准，无设备访问。

正常跟随 `max_relative_target=null`。新入口只写扭矩开关与位置目标，PID/模式及校准
只读取核验；回程使用示教速度 50%、30 Hz。超过录制时限只暂停，等待用户确认。
`action` 存从臂映射后的实际发送目标；自动回程、等待和整理环境不进入训练集。

新数据目录为 `data/datasets/so101_stack_white_on_red_batch_002`，示教文件为
`data/motion_profiles/stack_white_on_red_v1.json`。旧的 trial_001 和 batch_001 数据集
已按用户要求列入清理范围，具体删除命令仍须审核；校准与日志保留。

完整命令、所有生效参数、按键步骤、异常处理及清理范围见
[腕部映射与双臂复位操作说明](docs/adaptive-session.md)。
离线端到端检查已验证标准 LeRobot v3 保存、不同长度片段、双相机视频及映射动作标签。
39 项离线回归测试通过；已支持对齐阶段按 `c` 重采腕部参照，修复退出信号锁重入风险，
并在力矩使能确认包异常时只读核验一次，无法确认时仍退出清理。

2026-10-14：实机示教已完成并保存，腕部偏移 **-87.38°**，回程路径 **21.22 秒、
635 点**。校准指纹、关节范围、映射一致性与时序校验通过；详见
`outputs/adaptive-sessions/20261014-101955/profile-validation.json`。
此文件用于腕部映射和固定入口到起点的双臂复位。下一步须审核并启动 `--mode teleop`
进行实机自动回程测试；按当前半速设置，含倒计时、衔接和停稳正常约 46 秒。
七段任务数据采集将在回程测试后进行。此前间歇低电压和通信问题的记录保留在操作说明中，
不能仅凭本次示教成功认定故障已根治。

## ACT 推理 attention 热力视频（2026-10-17）

已对最新十轮实验中的 **第 1 轮** 制作离线注意力重建视频：

- [观看热力视频](outputs/act-attention/20261017-095317-752014-round-001-enhanced/attention.mp4)：双相机、30 fps，约 36.37 秒。
- [画面预览](outputs/act-attention/20261017-095317-752014-round-001-enhanced/contact-sheet.jpg)。
- [重建与验证报告](outputs/act-attention/20261017-095317-752014-round-001-enhanced/report.json)。
- [原始 attention 数组](outputs/act-attention/20261017-095317-752014-round-001-enhanced/attention.npz)。
- [生成原理与验证报告（Markdown）](docs/ACT注意力热力视频生成原理报告.md)。

视频上排是当前推理录像，下排是正在执行的动作所对应的**模型输入帧与热力图**。
本轮执行 1,074 个动作，只调用 ACT 11 次，每次预测 100 个动作。
下排图像在动作块内保持不变，热力图按日志里的 `chunk_step` 选择对应的动作 query；
画面显示输入年龄，避免把缓存动作解释成每帧重新推理。上排依据录像的实际采集时间
重采样到 30 fps，动作切换依据日志中目标写入完成时间对齐。

入口为 `scripts/visualize_act_attention.sh`，仅加载工作区内的录像、状态和模型，
离线计算并写入**新输出目录**，不连接机械臂或摄像头。复用示例：

```bash
bash scripts/visualize_act_attention.sh \
  --trace-dir outputs/act-traces/20261017-095317-752014/round-001 \
  --output-dir outputs/act-attention/round-001-rerun \
  --device cuda --fps 30 --color-gamma 0.5
```

输出目录必须不存在；再次运行需指定另一个新名字。模型与归一化器来自该轮记录的
`080000/pretrained_model`。只读 hook 提取最后一个 decoder 的 cross-attention，
使用该层默认返回的 8 头均值；[PyTorch 的权重返回格式说明](https://docs.pytorch.org/docs/2.9/generated/torch.nn.MultiheadAttention.html)
解释了这一维度。去掉 latent 和关节状态两个 token，按 top、wrist 的实际顺序分别
还原为 15×20 网格，再双线性放大到 640×480。数组保留全部 11×100×602 个原始权重，
也保存重算动作与原日志动作，便于进一步检查。

全片、双相机共享一个固定色阶，上限取所有已执行动作的图像权重 99.5 分位数，
显示采用 `clip(weight / vmax, 0, 1) ** 0.5` 提高低权重区域可读性；
不对每帧单独拉伸、不重新归一化图像权重，色阶不改变保存的数组。

**这是离线重建，并非恢复当时未保存的 attention 张量。** 原录像有 H.264 压缩损失；
首个输入帧还需分别使用相差约 +29.9 ms（top）和 +40.7 ms（wrist）的邻近帧，
其余十次推理两路相机均匹配原采集时间戳。重算的已执行动作与日志的归一化平均绝对差
为约 0.0248，hook 开关前后的预测差为 0。热力图反映已融合上下文的 encoder 空间
token 的权重，不是物体分割，也不能独立证明某区域导致了某个动作。

## 参考资料

- [Seeed SO-ARM100 / SO-ARM101 入门教程](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)
- [本次使用的 LeRobot 仓库](https://github.com/Seeed-Projects/lerobot)
- [LeRobot 官方 SO-101 文档](https://huggingface.co/docs/lerobot/so101)
- [LeRobot 官方 ACT 文档](https://huggingface.co/docs/lerobot/act)
