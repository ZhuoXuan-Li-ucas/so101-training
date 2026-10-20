# SO-101 training workspace

- This workspace is independent of all existing projects under `/home/pangk`.
- Write project files only within this workspace. Install dependencies only into
  `/home/pangk/miniforge3/envs/lerobot-so101`.
- Do not edit other repositories, other Conda environments, shell startup files,
  global Git/Pip/Conda settings, or system drivers.
- Keep datasets, calibration files, model caches, and outputs within this workspace.
- Follow `lerobot/AGENTS.md` when working inside the cloned source repository.
- The workspace now includes real-hardware experiments. Follow the command-review
  requirements below before operating hardware.

## 重要命令执行前必须交由用户审核

- 启动或重启遥操作等重要命令，必须先向用户展示本次准备执行的完整命令及
  实际生效的关键参数，等待用户明确审核同意后才可执行。用户泛泛提出
  “测试一下”“启动遥操”，不等于已经审核了代理自行选择的参数。
- 审核范围包括：遥操作、会使能扭矩或发送运动目标的诊断、校准、电机设置、
  实机数据录制、动作回放、策略实机运行，以及修改运动限幅、PID、速度、
  加速度、扭矩等控制参数的操作。训练等长时间占用资源的任务，以及升级依赖、
  覆盖或删除数据集、模型、校准文件等重要操作，也须先交用户审核。
- 如果命令调用脚本或配置文件，不能只展示脚本名：还需说明其实际动作、
  配置路径和关键生效值，包括主从臂端口与校准 ID、控制频率、位置单位、
  `max_relative_target`、涉及的舵机参数，以及是否使能扭矩、写目标位置或
  覆盖文件。默认值和脚本内的隐含设置同样属于审核内容。
- 不得以“更安全”或“优化”为由擅自加入、修改或取消控制限幅。特别是本次
  `max_relative_target=2.0` 将臂关节目标限制在当前实测位置附近 2 度，
  夹爪为 2 个归一化百分点；它明显影响了大范围跟随，不能作为未经用户审核
  的默认实验设置，也不能将其解释为严格的速度上限。
- 用户审核后按已批准的命令、参数和执行范围操作；如关键参数或脚本行为
  发生变化，先展示差异并重新审核，不得沿用旧审批。不要重复请求已经明确
  覆盖本次具体操作的审批。
- 读取代码、日志、设备枚举、只读状态诊断，以及用户已要求的文档修改，
  可以继续执行。只读诊断不得隐含使能扭矩、写目标或修改舵机寄存器。
  用户明确要求停止当前运动时，立即执行停止指令。
