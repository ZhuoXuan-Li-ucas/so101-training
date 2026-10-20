# SO-101 重新校验与恢复采集

> 腕转映射与复位软件现已切换为“保存初始姿态＋从当前位置直接回位”，入口为 `--mode setup`。
> 当前运行说明及待审核新参数见 [腕部映射与一键回位](direct-home.md)。下文旧 teach/入口回程命令
> 仅作历史记录，已不被活动脚本支持，不能按旧审批启动。

## 当前校准流程更新

**最新审核：两臂校准均通过。** 从臂再次采样后的腕转范围为 172..3997，约 336.26°；
中位下侧有 1060 个样本、上侧有 272 个样本，原先只覆盖一侧的问题已解决。
两臂六关节中位、实测范围、SHA-256 与最终寄存器读回一致，PID、速度、加速度、保护及 Phase 未变。
最终读回时两臂力矩均关闭、无故障码；本次审核没有额外连接设备。

恢复配置 `configs/so101_recovery.proposed.json` 已仅更新两条 calibration_dir，分别指向
`data/calibration/recalibration-20261014-152423-manual/leader` 和同级 `follower`。
`max_relative_target=null`、控制频率、PID 和数据配置均未变。审核与配置差异见
`outputs/calibration-reviews/20261014-173117-final-pair/`。
从臂最新文件 SHA-256：`69d47a966dd0283c6030d4f48988c608f3ee64c3c61204b0715c2523147c25d0`。
下文保留此前校准过程和命令，已完成的两臂无需重复校准；后续应先逐关节跟随验收。

用户已接管终端执行，助手只提供命令和最终核验。主臂本次在
`data/calibration/recalibration-20261014-152423-manual/leader` 完成了零位/行程写入和读回，
随后因人工回中位偏差（肘部约 +16.97°、腕俯仰约 −7.03°）超过旧脚本 5° 门槛而退出。
之后用户已用 --resume 完成主臂，现状态为 complete；中位、范围、文件哈希与最终寄存器读回均审核通过。
文件为该目录内 `my_awesome_leader_arm.json`，SHA-256 为
`2f8eaeebd39faca620d496d032eb9aa5d4a4286e2e5e80458f755982939b6263`。
审核报告：`outputs/calibration-reviews/repeat-check-adjustment/leader-final-audit.json`，未另外连接硬件。

新版默认只要求两步人工操作：中位设定和六关节范围采集。实测范围合法性、寄存器读回、
白名单之外参数不变、最终力矩/模式与故障/电压检查仍保留。人工回中位和单独方向确认均不再是保存门槛；
两臂对应方向留到后续跟随验收。额外诊断可显式加 `--direction-check` 或 `--repeat-check`，
默认不启用。这不修改采集起始姿态、控制限幅或舵机控制参数。

以下是主臂已经执行完成的历史恢复命令，**不要再次执行**：

```bash
bash /home/pangk/workspaces/so101-training/scripts/so101_passive_calibrate.sh \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json \
  --arm leader \
  --output /home/pangk/workspaces/so101-training/data/calibration/recalibration-20261014-152423-manual/leader \
  --resume --run
```

本命令仅连接主臂既定端口和 ID，不连接相机或从臂，**不写任何舵机寄存器**，包括不写力矩开关。
根据提示读取原始编码值，无固定采样频率；`max_relative_target=null`，无跟随。
要求当前力矩已关闭；先核对历史完成阶段与当前零位、范围、PID、速度、加速度、保护及 Phase。
任何不符都退出，不自动修复。再次读回全部控制参数；只有显式选择 --direction-check 才插入人工方向确认。
通过后新建原 ID 校准 JSON、记录哈希并更新完成状态；保留原 events.jsonl 和失败状态副本。
原校准文件不覆盖。这里只能继续已验证的人工回位失败，不能用于绕过通信、供电、部分写入或其他异常。

恢复由用户自行执行，现已审核通过。助手未代替用户操作硬件。
从臂接好后可运行以下新版默认命令，只需设置中位和采集范围：

```bash
bash /home/pangk/workspaces/so101-training/scripts/so101_passive_calibrate.sh \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json \
  --arm follower \
  --output /home/pangk/workspaces/so101-training/data/calibration/recalibration-20261014-152423-manual/follower \
  --run
```

该阶段仅连接从臂 `5AB9065574-if00`，ID `my_awesome_follower_arm`，无相机或主臂连接。
20 Hz 原始位置采样，写入白名单仍为 Torque_Enable=0、Lock=0、Operating_Mode=0，以及
Homing_Offset、Min/Max_Position_Limit；不使能力矩、不发送目标、不改 PID/速度/加速度/保护/Phase。
输出为新目录，保留旧校准。初始中位暂设范围 0..4095，随后写实测范围。

若 USB 未连接，程序会提示具体缺失串口，不写电机寄存器，也不提前创建输出目录。
此前因连接失败留下的空目录可以原样重试；目录内已有任何文件时仍拒绝覆盖。
本次从臂最初未插 USB，用户重新插入后已枚举为原稳定路径 `5AB9065574-if00` → `/dev/ttyACM1`，
无需修改端口配置。相关 10 项离线恢复/重试测试通过。

固定入口与示教回程方案已按用户要求取消为后续方向；当前位置直接回初始姿态的替代实现尚待完成。
以下保留早期配置与阶段记录，其中旧复位流程不构成当前实机启动授权。

本轮从 `outputs/handoffs/20261014-141553-baseline` 接手。设备核验结果在
`outputs/device-checks/20261014-141902/inventory.json`。用户本轮确认主臂 5 V、从臂 12 V，
并要求暂时忽略低压故障；因此两段 5 分钟巡检暂缓，供电稳定性仍未通过验收。
所换线的具体类型、电源额定电流尚未确认。程序继续保留现有保护阈值与故障停止行为。

## 已完成与待完成

- 基线包含旧配置、两臂校准、v1 回程、脚本/文档、关键日志、SHA-256 和代码差异。
- 只读核验：两臂各 6 个 STS3215（型号码 777），ID 1–6；12 个力矩读数全为 0。
  本次主臂全部 4.9 V，从臂 11.9–12.0 V，无状态/响应错误。PID 主臂 32/0/32、
  从臂 16/0/32，Operating_Mode=0，旧校准寄存器一致。
- 主臂肘部原始位置 2987，高于旧 range_max=2973；其他原始数值和控制寄存器均见清单。
  快照只能代表这次读取，不能替代稳定性巡检。
- 原 39 项离线测试重跑通过；新增后共 61 项通过。真实写入器的模拟保存验证通过：
  3 次越界演示作废，最终 3 条成功模拟段共 39 帧，两路视频各 39 帧；动作标签、
  时间戳、复位帧排除、首条复查门控及 LeRobot 重载通过。
- 尚未进行本轮校准、跟随、实机暂停恢复、三次自动回程和正式采集。新校准和 v2 回程尚不存在。
  本轮没有已批准的硬件写入命令。

## 文件与参数

恢复配置为 `configs/so101_recovery.proposed.json`。相对原基线仅把回程输出改为
`data/motion_profiles/stack_white_on_red_v2.json`，当前仍指向旧校准。每臂校准成功后，
把该臂 `calibration_dir` 改为本次新目录，展示差异，再审核下一阶段命令。

| 项目 | 生效值 |
| --- | --- |
| 主臂 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79050214-if00`，`my_awesome_leader_arm` |
| 从臂 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9065574-if00`，`my_awesome_follower_arm` |
| top | Gemini335，`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0`，本轮枚举为 `/dev/video8` |
| wrist | UGREEN，`/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0`，本轮枚举为 `/dev/video0` |
| 串口 | 主机端 1,000,000 baud，读取超时 100 ms；不改电机 Baud_Rate |
| 控制/相机 | 30 Hz；两路 RGB MJPG 640×480、30 fps；相机画面还需现场确认 |
| 单位与限幅 | 臂关节为度，夹爪为 0–100 点，`max_relative_target=null` |
| PID | 主臂 32/0/32、从臂 16/0/32，只核对不改写 |
| 回程 | 0.5 倍速度、倒计时 2 秒、衔接 1 秒；入口/起点容差 5°、夹爪 5 点 |
| 数据 | `data/datasets/so101_stack_white_on_red_batch_002`，7 条、每条最多 120 秒，成功可提前结束 |

数据目录本轮仍为空。若启动前已存在目标目录，先生成时间戳新批次配置并提交审核；程序拒绝覆盖。
不上传、不混用旧坐标数据。每次会话保存校准指纹和回程 SHA-256。

## 只读盘点与巡检

`scripts/so101_readonly.py` 仅使用底层总线 `connect(handshake=False)`，退出时明确
`disconnect(disable_torque=False)`。不调用设备连接、配置、校准、使能力矩或目标发送接口。
初次读取完整控制寄存器和反馈，记录 ID/型号、PID、Phase、保护、电压、校准、速度、加速度。
发现力矩开启会记录并停止巡检，不自动关力矩。

只读盘点命令：

```bash
/home/pangk/miniforge3/envs/lerobot-so101/bin/python \
  /home/pangk/workspaces/so101-training/scripts/so101_readonly.py \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json
```

恢复供电排查时，在此命令后分别追加 `--duration 300` 和 `--duration 300 --cameras`，
按先无相机、再双相机的顺序运行。目标 20 Hz，输出实际频率、电压、故障和通信统计；
任一故障使该段不通过。全部电压在原保护范围内，零通信错误、零故障，才可认为该段通过。
操作者需观察静止及托住机械臂缓慢改变姿态的表现。本轮按用户要求尚未运行。

## 被动校准命令审核单（尚未获批）

以下为两个独立阶段，一次只连接指定机械臂。使用固定 Conda 环境，缓存仍在工作区。
`--check` 仅输出契约，已经执行；下面带 `--run` 的命令仍需明确审核。

先主臂：

```bash
bash /home/pangk/workspaces/so101-training/scripts/so101_passive_calibrate.sh \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json \
  --arm leader \
  --output /home/pangk/workspaces/so101-training/data/calibration/recalibration-20261014-141553/leader \
  --run
```

主臂完成并核验后，再审核从臂：

```bash
bash /home/pangk/workspaces/so101-training/scripts/so101_passive_calibrate.sh \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json \
  --arm follower \
  --output /home/pangk/workspaces/so101-training/data/calibration/recalibration-20261014-141553/follower \
  --run
```

实际行为：

1. 只连接上表指定的单臂端口，沿用该臂 ID；相机关闭，另一臂不连接。
   检查全部型号码 777、ID、Phase 的角度反馈位、故障和原电压保护范围；异常即退出。
2. 写入白名单为 `Torque_Enable=0`、`Lock=0`、`Operating_Mode=0`、`Homing_Offset`、
   `Min_Position_Limit`、`Max_Position_Limit`。每次写后读回校验，无写入重试。
   不写 Goal_Position，不使能力矩；PID、速度、加速度、保护阈值、Phase 均只读。
3. 用户托住并手动摆到中位。使用本地 LeRobot 的半圈算法：暂把 offset=0、范围=0..4095，
   读取原始位置，计算 `offset=raw-2047`，写入后验证中位读数。
   这些临时范围寄存器写入也属于审核内容；机械臂全程力矩关闭。
4. 以 20 Hz 采集**全部六关节（含腕转）**的实际可用 min/max，不默认把腕转定为完整 0..4095。
   按提示缓慢手动移动，不强压端点；出现编码器绕回或未采到非零行程即失败。
   校准从臂时直接手动移动从臂，不能用主臂遥操。
5. 写入实测范围，核对所有白名单以外控制寄存器未变，并核对最终力矩、模式和故障状态。
   默认直接保存，不要求再次回中位或方向确认；额外人工诊断仅在显式选择对应参数时启用。
   校准中位以 raw≈2047 表示；最终训练坐标仍按实测 min/max 中点归一化。
6. 输出新目录内的 `<原ID>.json`、寄存器前后值、原始采样、重复回位误差、状态和 SHA-256。
   保留旧文件。任一步失败写入“incomplete”标记，不静默恢复部分寄存器。
   `data/calibration/status/<role>-<ID>.json` 阻止采集程序继续使用旧配置；成功后也必须显式选用新文件。

停止方式为终端 Ctrl+C；退出只关闭串口，不再写寄存器。若在关闭力矩途中失败，
状态记录会保留未完成，须先只读核实实际力矩。不能因脚本名为“被动”就假定断连后所有力矩均已关闭。

## 跟随、越界暂停与回程

两臂校准成功且配置指向新文件后，分别审核：

```bash
bash /home/pangk/workspaces/so101-training/scripts/so101_session.sh \
  --config /home/pangk/workspaces/so101-training/configs/so101_recovery.proposed.json \
  --mode follow --no-cameras
```

首阶段是零腕转偏移的同向 1:1 跟随，不录制、没有自动回程。移除 `--no-cameras` 是下一次
独立审核的双相机跟随阶段。两次均逐关节往返，至少连续 60 秒。连接会关闭两臂力矩；
按 `s` 对齐检查通过后，从臂先写实测位置再开启力矩，主臂自由。退出尝试关闭每个关节力矩。
只允许写 Torque_Enable、Goal_Position；旧 PID/保护等参数不改写。

每轮每个电机仅读取一个包含原始位置的反馈块，同一份原始样本派生标准归一化读数。
夹爪原始越界在 0–100 截断之前判断。界面显示“主臂实测→映射请求→已发送目标→从臂实测”，
并显示主臂坐标下的共同可用范围。靠近边界 5°/5 点提示不改变目标值。

`LIMIT_PAUSED` 只用于手动跟随的有限主臂位置/映射目标越界。进入前检查从臂合法、健康、反馈新鲜，
随后仅保持从臂当时实测位置，主臂自由。界面保留画面，显示原因、上下限、保持位置和恢复条件。
用户回到合法范围，映射目标对齐从臂 ≤5°/5 点，且异步丢弃完成后，显式按 `m` 才恢复。
暂停期间持续读健康/原始位置；从臂偏离保持目标超过 5°/5 点即停止。
恢复只使用当前样本，不缓存暂停期间的操作。

示教暂停会丢弃未保存路径，恢复到 `TEACH_READY`；录制暂停会丢弃整个未保存 episode，
计数不增加，已保存段保留。恢复后进入 `WAIT_ENTRY` 的不录制整理阶段，复位后才能按 `s` 重录。
非有限值、从臂越界、供电/通信错误、旧反馈、500 ms 控制停顿以及自动回程异常仍停止并清理。

每轮日志记录原始值、归一化值、健康、请求/发送目标及耗时；保持训练六维 state/action 的原含义。
离线分析命令：

```bash
/home/pangk/miniforge3/envs/lerobot-so101/bin/python \
  /home/pangk/workspaces/so101-training/scripts/so101_performance.py \
  /home/pangk/workspaces/so101-training/outputs/adaptive-sessions/<实际会话目录>
```

检查平均 ≥28 Hz、循环周期 P95 ≤50 ms、无 ≥500 ms 间断。分别报告主臂读取、映射、
从臂发送、从臂反馈和图像获取耗时，以及与上一轮发送目标相比的跟踪误差。
使能力矩时确认包缺失、读回成功的容错事件也会记录，并使“无通信错误”验收不通过。
串口写入耗时不代表机械响应时间；未通过时定位瓶颈，不能改限幅掩盖。

再分别审核 `--mode teach`、`--mode teleop`、`--mode record`，不能合并沿用旧审批。
teach 中按 `c` 采样“主臂竖直握把→从臂横向夹爪”，仅腕转加固定偏移；按 `a` 跟随，
到空旷入口按 `t` 示教入口→起点，到位按 `n` 保存 v2。旧 v1 与旧校准绑定，禁止改哈希冒充新产物。
teleop 中空夹爪、无障碍条件下连续三次自动回程成功后才能记为实机通过；入口条件保留。

## 7 条正式采集与首条复查

开始采集前审核新校准、新回程指纹和实际新批次路径。任务仍为抓白块叠红块后松爪。
每条最多 120 秒；`n` 确认成功、`r` 丢弃；失败重录。复位、等待、手动整理均不写训练帧。
摆好积木后用户按 `s` 开始下一条。

第一条保存后，写入线程解码两路实际视频，检查帧数/尺寸，并检查已提交数值表的 state/action、
有限值、索引和时间戳，保存抽帧及 NPZ 到 `outputs/episode-reviews/<数据目录名>/<时间>/`。
复位后进入 `FIRST_REVIEW`；核对视频中的任务成功和数值报告，按 `v` 确认后才能继续其余六条。
这里的数值来源是 LeRobot 已提交的内存表；正式 Parquet 文件在最终 finalize 后还须重新读取校验，
不能把首条检查解释为已经检查了未关闭的 Parquet 文件。程序不自动识别积木任务成功。

退出先释放双臂力矩、断开硬件，再等待写入完成；未保存片段作废，已保存成功片段保留。
`q`、Esc、关闭窗口、SIGINT/SIGTERM 均进入清理；支撑双臂后退出。

## 离线复验

```bash
cd /home/pangk/workspaces/so101-training
PYTHONDONTWRITEBYTECODE=1 /home/pangk/miniforge3/envs/lerobot-so101/bin/python \
  -m unittest discover -s scripts/tests -v
XDG_CACHE_HOME="$PWD/.cache" HF_HOME="$PWD/.cache/huggingface" \
HF_LEROBOT_HOME="$PWD/data/datasets" PYTHONDONTWRITEBYTECODE=1 \
/home/pangk/miniforge3/envs/lerobot-so101/bin/python scripts/check_adaptive_session.py
```

模拟保存测试禁止实机总线/相机连接，仅写 `outputs/adaptive-smoke/` 下的新目录。
完整本轮交接、测试输出、审核摘要及最终文件哈希见交接目录。
