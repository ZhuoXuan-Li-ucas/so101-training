# SO-101 终端采集手册

当前入口：`scripts/collect_so101.sh`。本次按用户要求保留自动复位和腕部映射，
将录制、手动跟随阶段的反馈改为 LeRobot 官方 `MotorsBus.sync_read("Present_Position")` 通道。
数据仍由官方 `LeRobotDataset` 写为 v3.0 格式。

这不是原样运行 `lerobot-record`：为了保留自动回初始姿态，仍使用本工作区的状态机和双相机窗口。
按键仍是 **S / N / R / V / M / Q**，不是官方命令的方向键流程。

## 改动范围与限制

- 录制、手动跟随和初始等待：每周期分别同步读取两臂位置，不再逐电机轮询电压、负载、
  电流或 `Status`，也不再在这个路径上因额外读到 `Status=32` 而终止整场采集。
- 没有屏蔽 SDK 抛出的异常，没有写入或取消舵机硬件保护。通信失败在官方最多 2 次额外重试后仍会抛出；
  非有限值、控制停顿检查仍保留。校准边界现使用已确认的 16 个编码器计数容差：
  双臂原始反馈与臂关节实测、映射目标、保持、复位共用对应边界，超过容差才拦截。
  夹爪归一化目标仍为 0–100；保留原有归一化方式，不为容差额外裁剪或替换观测值。
- 启动时、进入双臂保持前，以及保存等待、自动复位、复位后保持和越界暂停阶段，仍使用完整反馈检查。
  自动保持/复位中的设备故障按第 4 节进入故障暂停；启动失败、保存失败等仍退出。
  进入 R/N 的保持动作前会检查一次完整反馈。
- `health: null` / `feedback_source: official_position` 的日志表示本周期没有采集健康寄存器，
  不能当成“已检查且无故障”。完整反馈阶段标为 `feedback_source: full`。
- 不增加夹爪限差，不改变 `max_relative_target=null`，也不改 PID、速度、加速度或保护寄存器。
- 已审核的两条演示实际均约 29.88 Hz，均完成白块叠放；该结果来自容差补丁应用前的采集。
  接触物体后仍可能触发舵机自身保护，软件读取方式不能保证任意持续夹紧都可用。

## 固定参数审核单

配置模板：`/home/pangk/workspaces/so101-training/configs/so101_collect.json`。
以下相对路径均相对于工作区 `/home/pangk/workspaces/so101-training`。

| 项目 | 实际生效值 |
| --- | --- |
| 环境 | `/home/pangk/miniforge3/envs/lerobot-so101`，不安装/升级依赖 |
| 主臂端口 / ID | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF220248-if00` / `my_awesome_leader_arm` |
| 从臂端口 / ID | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9065574-if00` / `my_awesome_follower_arm` |
| 主臂校准 | `data/calibration/replacement-leader-20261014-185158/leader/my_awesome_leader_arm.json` |
| 从臂校准 | `data/calibration/recalibration-20261014-152423-manual/follower/my_awesome_follower_arm.json` |
| 腕转 / 初始姿态 | `data/motion_profiles/stack_white_on_red_home_v1.json`；当前偏移 −76.74725274725274°；按校准哈希核验 |
| 位置单位 / 频率 | 臂关节为度、夹爪 0–100 点；控制与数据均为 30 Hz |
| 正常跟随限幅 | `max_relative_target=null`；无新增夹爪限差 |
| 校准边界容差 | 双臂各关节原始反馈允许超出校准端点 16 个计数，并限制在编码器 0–4095 内；臂关节实测、映射目标、保持和复位共用对应边界，约 ±1.41°；夹爪归一化目标仍为 0–100。保留真实读数，不改校准文件或舵机限位寄存器 |
| PID / 模式 | 两臂 16/0/32、Operating_Mode=0；只读核对，不匹配则退出 |
| 电机写入 | 仅 `Torque_Enable` 和 `Goal_Position`；不调用会改配置寄存器的原生 `connect()` |
| 硬件保护 | 保持已有值；上次只读的从臂夹爪原始值为 Max_Torque_Limit=500、Protection_Current=250、Overload_Torque=25；本入口不重写 |
| top 相机 | `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0` |
| wrist 相机 | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0` |
| 相机 / 编码 | 双路 RGB、MJPG、640×480、30 fps；H.264 流式写入，2 编码线程，队列 30，CRF=23、GOP=2、ultrafast |
| 任务 | 抓起白色积木，叠在红色积木上，再松开夹爪 |
| 数量 / 时长 | 默认本批次成功 2 条；每条最多 120 秒；数量可通过 `--episodes` 修改 |
| 自动复位 | 双臂从各自当前位置返回保存姿态；先保持并倒计时 2 秒；最短运动时间 2 秒，距离远时延长 |
| 复位目标速度 / 加速度 | 臂关节 10°/s、20°/s²；夹爪 10 点/s、20 点/s²；是生成目标的约束，不是实测速度硬上限 |
| 到位 / 跟踪检查 | 到位误差 ≤5° / 5 点，稳定 0.5 秒；到位超时 5 秒；跟踪误差 >15° / 20 点持续 0.5 秒停止并故障暂停；控制间隔上限 0.5 秒 |
| 完整反馈时的低压处理 | 仅主臂力矩关闭时允许 50 ms 复核、间隔 2 ms、连续 2 次正常；手动采集位置读取路径不执行这项轮询 |
| 复位 / 保持故障暂停 | 停止目标发送，尝试关闭并核对双臂全部力矩；保留窗口及已保存数据。暂停按 0.2 秒间隔只读复查，不写目标、不反复使能或释放力矩 |
| 故障后重试 | 连续两轮新鲜完整反馈正常、全部力矩读回为 0，且保存/作废完成后，重新按 R；从两臂本次实测位置新建复位计划，沿用上列倒计时与运动参数 |
| 保存 | 新建 `data/datasets/<名称>`，repo_id=`local/<名称>`；不覆盖、不自动续录、不上传 |
| 启动 / 退出 | 启动关闭双臂力矩；S 开跟随时仅从臂使能；自动复位双臂使能；退出尝试关闭全部关节力矩 |

命令调用关系是：

```text
collect_so101.sh record --name NAME --episodes 2
  → so101_adaptive.py --config configs/so101_collect.json --mode record
      --expected-leader-pid 16 0 32 --feedback-mode official
      --dataset-name NAME --episodes 2
```

如果由代理代为执行实机命令，仍按工作区 AGENTS.md 审核本次完整命令与参数；
下列操作说明供你本人在终端执行。本次交付没有启动实机。

## 1. 选名字、检查、启动

`so101_stack_white_on_red_20261015_01` 已保存并审核通过，下列新采集示例改用 `_02`，不会覆盖已有两条数据。

先关闭其他相机预览或控制程序。名字只允许字母、数字、下划线和连字符，不要填写路径。
每次新采集建议使用新名字，例如同一天依次 `_01`、`_02`。

先执行只读检查。它不打开串口/相机、不使能力矩，也不创建数据集：

```bash
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh record \
  --name so101_stack_white_on_red_20261015_02 --episodes 2 --check
```

确认输出中的数据目录、映射和参数后，支撑双臂，执行同一命令但去掉 `--check`：

```bash
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh record \
  --name so101_stack_white_on_red_20261015_02 --episodes 2
```

窗口打开后处于等待状态，不会自动录制。连接时双臂力矩关闭；先将双臂摆到合法、映射对齐的位置。
双臂已在保存的初始姿态时可直接 S；若需要归位，先放下物体并松开夹爪、确认两臂路径空旷，再按 R。
首次跟随要求映射目标与从臂实测差在 5° / 5 点内，未对齐时会显示具体关节差异。

## 2. 每条演示的操作

快捷键仅在双相机控制窗口取得焦点时有效；先点一下窗口。也可以直接点击底部按钮。

1. **准备**：双臂在保存的初始姿态，摆好积木、检查相机，握住主臂。
2. **S 开始录制**：打开夹爪 → 接近白块 → 夹住 → 搬到红块上 → 放下 → 松开夹爪。
   夹稳后不要继续将主臂夹爪捏到底。此时记录的是从臂实测状态、实际发送目标和两路视频。
3. **N 成功保存并复位**：确认任务成功、物体已松开、双臂回程无障碍后按 N。
   保存完成后双臂保持当前位置，倒计时 2 秒，再自动返回。倒计时和回程不录制。
   倒计时期间松开主臂、让开路径，不再拖动机械臂。
4. **第 1 条复查**：出现 `FIRST_REVIEW` 后，按提示检查保存的视频、数值报告和任务结果。
   确认后按 V，再摆好积木，握住主臂并按 S 录第 2 条。不会自动开始下一条。
5. **完成**：第 2 条同样按 N 保存并复位。看到 `DONE` 后，支撑双臂，按 Q / Esc 退出。
   等终端返回命令提示符后，再执行下面的数据检查。

| 按键 | 实际作用 |
| --- | --- |
| S | 在初始姿态开始本条录制 |
| N | 明确标记本条成功、保存，再自动复位 |
| R | 当前未保存片段作废并复位；故障暂停时仅在就绪后接受新按键重试，不删除已保存条目 |
| V | 首条保存并复位后，确认人工复查完成 |
| M | 越界暂停后，在重新对齐的条件下恢复手动跟随；不会自动恢复录制 |
| Q / Esc / 关闭窗口 | 停止并释放力矩；当前尚未提交的片段不计入成功数据 |
| Ctrl+C（终端） | 请求停止并执行清理；等待退出，不要连续按或直接强杀 |

达到 120 秒时程序停止录帧并保持双臂，等待 N 保存或 R 作废，不会自动判定成功。
失败演示先安全放下物体、松开夹爪，再 R，复位后 S 重录。
若进入越界暂停，先回到合法并对齐的位置，M 恢复手动整理；需要重录时 R 回初始姿态，再 S。

## 3. 退出后检查数据

```bash
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh status \
  so101_stack_white_on_red_20261015_02
```

检查 `saved_episodes`、`saved_frames`、`parquet_frames`、`parquet_episodes` 和 `issues`。
此命令只读元数据和 Parquet 文件尾，不逐帧解码视频，也不替你判断任务成功。
在采集进程完全退出后执行，避免把尚未关闭的 Parquet 文件误判为损坏。

- `state=saved` 且 `issues=[]`：已提交条数与 Parquet 行数一致；仍应检查画面和动作质量。
- `state=empty`：初始化了目录，但没有已提交的成功演示。仅有 `meta/info.json` 不代表录制成功。
- `state=needs_inspection`：存在未完成保存、文件缺失或无法读取的迹象；先保留目录和日志。
- `state=not_created`：尚未创建该数据集。

查看视频（有成功保存后才会出现；多条 episode 可能在同一个 MP4 中）：

```bash
source /home/pangk/workspaces/so101-training/activate.sh
ffplay /home/pangk/workspaces/so101-training/data/datasets/so101_stack_white_on_red_20261015_02/videos/observation.images.top/chunk-000/file-000.mp4
ffplay /home/pangk/workspaces/so101-training/data/datasets/so101_stack_white_on_red_20261015_02/videos/observation.images.wrist/chunk-000/file-000.mp4
```

## 4. 故障暂停与异常退出

若自动保持/复位阶段出现持续低压、故障位、力矩状态不符、通信失败、反馈超时、
位置越界、倒计时中被拉动或跟踪偏差，窗口会显示红色 **故障暂停（FAULT_PAUSED）**。
程序先停止发送目标并尝试释放双臂全部力矩，窗口和采集会话保留。

1. 支撑双臂，按窗口提示排查供电、连接或姿态。若提示力矩关闭未确认，不要假定机械臂已经释放；
   必要时切断外部电源。暂停期间不会反复写舵机，也不会自动重连串口。
2. 暂停时约每 0.2 秒只读复查完整反馈。读取失败或反馈无效时，清空旧相机画面和实测表，
   避免把旧数据当成实时状态；反馈恢复后窗口可以重新显示当前画面，状态仍保持暂停。
3. 等待窗口提示“反馈正常且双臂力矩全关”。必须有连续两轮新的完整正常反馈，
   保存或作废任务也已完成。尚未就绪时按 R 会被忽略，不会排队等待恢复后执行；S/N/M/V 也不会启动运动。
4. 放下物体、松开夹爪并确认路径空旷后，**重新按 R**。程序从两臂当前实测位置重新规划，
   保持当前位置、倒计时 2 秒后复位。不会继续故障前的旧轨迹，也不会自动开始下一条录制。
   若重试再次失败，会重新进入故障暂停；也可随时按 Q / Esc 退出。

已保存条目保留。若故障发生时 N 已提交保存，后台会继续完成这次保存，并且只计数一次，
不会重复保存或在保存完成后自动复位。尚未提交保存的当前片段作废，不计成功；
首次保存后的人工 V 复查仍须在重试复位成功后完成。

启动失败、普通手动跟随中的错误、数据写入失败、界面故障或未预期的程序错误仍走退出清理。
故障暂停不会修复供电问题，也不会修改 PID、保护寄存器、`max_relative_target` 或复位运动参数。
若程序已经退出：

1. 支撑双臂，等待程序清理并返回终端。若显示 `Hardware cleanup failed` / `torque OFF unverified`，
   应支撑机械臂并切断外部电源；不要假定异常退出必然关闭了所有力矩。
2. 保存错误末尾。终端记录在 `logs/so101-collect.log`；每场配置与事件在
   `outputs/adaptive-sessions/<时间>/session.json` 和 `events.jsonl`。
3. 用上面的 `status` 确认已成功提交多少条。未按 N 提交的当前片段通常由退出清理作废；
   若异常发生在保存中，以磁盘检查结果为准，不按界面之前的计数猜测。
4. 排查具体故障后再启动。本入口不自动重启、不自动删除数据，也不修改舵机保护。

可执行已有的只读状态工具核对力矩和故障位（不开相机、不写舵机）：

```bash
source /home/pangk/workspaces/so101-training/activate.sh
uv run --no-project --python "$CONDA_PREFIX/bin/python" python \
  /home/pangk/workspaces/so101-training/scripts/so101_readonly.py \
  --config /home/pangk/workspaces/so101-training/configs/so101_collect.json --duration 0
```

切勿在其他控制进程占用设备时并行运行；工具检测到端口被占用会拒绝连接。

## 5. 清理旧目录并重新录

推荐保留旧目录，换一个新名字重录：

```bash
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh record \
  --name so101_stack_white_on_red_20261015_03 --episodes 2 --check
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh record \
  --name so101_stack_white_on_red_20261015_03 --episodes 2
```

若上一批已有 1 条经检查可用，本次可以用新名字和 `--episodes 1` 补采剩余一条。
这会得到两个独立数据集，后续需要合并；不会悄悄追加到旧文件。
此自动复位入口目前不支持 `--resume`，不要把官方 `lerobot-record --resume=true` 参数直接套用。

如果确实要释放旧名字，先退出采集、关闭查看该数据的视频播放器，再检查并归档整个目录：

```bash
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh status \
  so101_stack_white_on_red_20261015_02
bash /home/pangk/workspaces/so101-training/scripts/collect_so101.sh archive \
  so101_stack_white_on_red_20261015_02
```

`archive` 将该目录移动到 `data/dataset_archive/<原名>-<时间>`，打印实际目的地；文件内容不会删除。
检测到采集进程仍在运行或文件被占用时拒绝归档。它只接受一个数据集名字，不接受路径或通配符。
归档后可再次使用同名的 `record ... --check` 和 `record ...` 从第 0 条重新录。

**删除配置 JSON 不会清理数据集，也不会清除已保存的腕部映射和初始姿态。**
无需删除 `configs/so101_collect.json`；日常重录只换 `--name` 或归档对应数据目录。
不要清理 `data/calibration/`、`data/motion_profiles/` 来解决“数据目录已存在”。

## 文件格式与参考

软件版本为本环境 LeRobot 0.6.2，训练数据格式 v3.0。标准目录包含 `data/`、`videos/` 和 `meta/`；
`outputs/adaptive-sessions/` 是诊断记录，不是训练数据集。
一次成功片段必须完成 `save_episode`，退出时必须完成 `finalize`，目录存在本身不代表成功。

- [LeRobot v3 格式](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)
- [Seeed 的 SO-Arm 教程](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)
- [原始映射与复位说明](direct-home.md)

2026-10-15 已完成批次 `so101_stack_white_on_red_20261015_01` 的只读审核：两条分别为
1247 / 1061 帧、41.57 / 35.37 秒；两路视频各 2308 帧，状态/动作与录制日志一致，
关键画面和末帧确认白块已叠在红块上并松爪撤离，两条均保留。
5 次低压暂停均在录制之外；这不代表供电故障已修复。数据文件未修改。
详见 [项目进展](../README.md#2026-10-15-当前采集入口) 和
[审核报告](../outputs/recording-review/so101_stack_white_on_red_20261015_01/integrity-report.json)。
本次新应用的 16 计数容差已通过 134 项离线回归及 `--check`，尚未启动实机验收。
