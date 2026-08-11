# PiperX 从臂外力矩估计与验证

本工具在不发送 CAN 帧、不更改机械臂状态、不修改 EvoStudio 的前提下，估计 PiperX 从臂六个关节受到的环境外力矩，并把原始量、动力学量和估计量保存为可重放的 NPZ/JSON 文件。

## 结果的物理含义

输出是六维**关节外力矩**（Nm），不是末端 Cartesian force/wrench，也不是主臂操作者施加的力。目标仅是从臂与环境接触造成的关节力矩。

采用的符号约定为：

```text
M(q) qdd + C(q, qd) qd + g(q) = tau_actuator + tau_external
tau_external = tau_model + tau_bias - tau_measured
```

正外力矩沿相应关节坐标的正方向。`tau_measured` 由 Piper 高速反馈帧中的电机电流换算；`tau_model` 由 Pinocchio RNEA 计算；`tau_bias` 是每台从臂在明确无接触数据上拟合的稳定残差。

## 安全边界

- 只打开 Linux SocketCAN 原始接收套接字；代码没有发送方法。
- 不执行 CAN 接口初始化或 `ip link`，接口必须已经由现有系统配置完成。
- 不调用 Piper SDK，不发送固件查询、角色切换、使能、运动、夹爪或 MIT 力矩命令。
- 通过 USB-CAN 序列号发现接口，不依赖可能变化的 `can2`/`can3` 名称。
- 标定工件与适配器序列号、URDF SHA-256 和基座方向绑定；不匹配时拒绝运行。
- 输入不完整、陈旧、时间倒退、采样间隔过大、加速度异常或越出标定工作区时，样本标为无效。
- 标定不会在线自更新，以免把持续真实接触吸收到零偏中。

## 环境

目标环境为 Linux、Python 3.10+、NumPy 和 Pinocchio 3.9+。在仓库根目录运行：

```bash
python -m pip install -r requirements-dev.txt
python -m pytest tests/piperx -q
python scripts/piperx_external_torque.py --help
```

PiperX URDF 必须通过 `--urdf` 显式提供。本仓库不复制上游模型；标定 JSON 会保存实际文件哈希。运行时必须使用与标定时完全相同的文件。

## 当前 `.166` 从臂身份

部署时应以实时序列号发现结果为准。前期只读检查得到：

| 从臂 | USB-CAN 序列号 | 当时接口 |
|---|---|---|
| left follower | `004B00204148570D20343133` | `can2` |
| right follower | `003F002D4148571320343133` | `can3` |

验证发现结果：

```bash
python scripts/piperx_external_torque.py discover \
  --serial 004B00204148570D20343133
```

如果序列号找不到或同时匹配多个接口，工具会失败退出，不会改用硬编码接口。

## 无接触数据、拟合和重放验证

确保从臂未接触物体、未挂载额外载荷后，才可显式加入 `--confirm-no-contact`：

```bash
python scripts/piperx_external_torque.py record \
  --arm left \
  --serial 004B00204148570D20343133 \
  --urdf /absolute/path/piper_x_description_no_gripper.urdf \
  --seconds 30 \
  --group-seconds 2 \
  --output data/left-no-contact.npz \
  --confirm-no-contact
```

`record` 是被动监听器，不会让机械臂运动。动态标定轨迹必须由现有正常主从遥操路径产生。不要为了本工具启用机械臂或发送动作。

拟合每台从臂独立工件：

```bash
python scripts/piperx_external_torque.py fit \
  --input data/left-no-contact.npz \
  --output calibration/left.json
```

数据按连续时间组切分，最后一部分完整组作为验证集，不随机打散相邻帧。JSON 会输出训练和验证的逐关节 MAE、RMSE、标准差及 p95 绝对误差。

静态无接触验收：

```bash
python scripts/piperx_external_torque.py evaluate \
  --input data/left-static-validation.npz \
  --calibration calibration/left.json \
  --stationary
```

自由运动无接触验收去掉 `--stationary`。命令通过时退出码为 0，未达到阈值时为 2。

## 实时监看

一个进程可独立监听一台或两台从臂：

```bash
python scripts/piperx_external_torque.py monitor \
  --urdf /absolute/path/piper_x_description_no_gripper.urdf \
  --arm left,004B00204148570D20343133,calibration/left.json \
  --arm right,003F002D4148571320343133,calibration/right.json
```

屏幕值限制在显示范围内，NPZ 中的原始量不裁剪。`valid=false` 时必须同时检查 `reason`，不能把显示数值当成有效测量。

## 实时网页仪表盘

网页监控与终端 `monitor` 使用相同的只读估计器，但增加了高速电流、SDK
固定系数换算的 effort、低速电机/FOC 温度、驱动状态、遥操目标关节角和
被动观察到的固件版本。服务仅绑定 `.166` 的 `127.0.0.1`，浏览器通过
SSH 本地转发访问；不会在局域网开放端口。

`.166` 上的完整启动参数为：

```bash
cd /home/dell/piperx-force-validation
/home/dell/anaconda3/bin/conda run --no-capture-output -n evo-rl \
  python scripts/piperx_torque_web.py \
  --urdf /home/dell/Evo-RL.before-pr-sync/src/lerobot/assets/piper_x_description/urdf/piper_x_description_no_gripper.urdf \
  --arm left,004B00204148570D20343133,calibration/left.json \
  --arm right,003F002D4148571320343133,calibration/right.json \
  --host 127.0.0.1 --port 18765 --ui-rate 50
```

推荐从 Windows 仓库根目录直接运行：

```powershell
.\scripts\open_piperx_monitor.ps1
```

脚本在可见 SSH 窗口中启动远端监控并建立
`127.0.0.1:8765 -> .166:127.0.0.1:18765` 转发，随后打开
`http://127.0.0.1:8765`。脚本和仓库不保存密码；首次连接需要在 SSH
窗口中输入凭据。关闭该 SSH 窗口或按 `Ctrl+C` 会同时停止转发和本次远端
监控，不安装开机服务。

远端端口使用 `18765`，是因为 `.166` 的 `0.0.0.0:8765` 已由现有
RoboClaw 服务占用；启动脚本不会操作该进程。验证证据和仍需进行的动态遥操、
独立真值验收见 `docs/piperx_dashboard_hardware_validation_2026-08-11.md`。

也可以手动使用一个 SSH 会话同时承载进程和端口转发：

```powershell
ssh -o ExitOnForwardFailure=yes `
  -L 8765:127.0.0.1:18765 dell@192.168.105.166 `
  "cd /home/dell/piperx-force-validation && exec /home/dell/anaconda3/bin/conda run --no-capture-output -n evo-rl python scripts/piperx_torque_web.py --urdf /home/dell/Evo-RL.before-pr-sync/src/lerobot/assets/piper_x_description/urdf/piper_x_description_no_gripper.urdf --arm left,004B00204148570D20343133,calibration/left.json --arm right,003F002D4148571320343133,calibration/right.json --host 127.0.0.1 --port 18765 --ui-rate 50"
```

只读诊断端点：

| 路径 | 内容 |
|---|---|
| `/healthz` | 服务和两条采集线程是否存活 |
| `/api/status` | 接口、序列号、序列计数和错误 |
| `/api/snapshot` | 两臂最新严格 JSON 快照 |
| `/stream` | 浏览器使用的 SSE 实时流 |

网页中的 `电流换算力矩（参考）` 不是独立关节扭矩传感器。Piper 官方 SDK
将反馈电流乘固定系数得到 effort；官方 Q&A 还说明底层固件 `1.8-2` 及更早
版本的 J1-J3 需要额外乘 4。监控本身不发送固件查询，也不会在固件未知时
静默改变标定。页面显示 `固件未知` 或 `旧固件` 时，应先核对版本和重新验收
力矩比例。

`趋势相关性（非精度）` 是最近窗口中 effort 与外力矩估计的 Pearson 相关系数。
两者共享电流输入，因此只能辅助观察趋势、符号、换向和异常，不能证明绝对
精度。绝对准确性仍需六维力传感器、拉压力计或已知载荷提供独立真值。

常用界面操作：数字键 `1`–`6` 选择关节，`L`/`R` 选择从臂；PAUSE 只冻结
本地显示，不会暂停远端采集或遥操。无效估计在图上断线而不是画成零，原因会
在右侧告警区显示。

### 普通遥操与 Web 联合模式

不要在 EvoStudio 遥操运行时启动上面的纯监控进程。EvoStudio 会管理并重置
PiperX CAN 接口生命周期，两个独立进程可能在启动阶段冲突。需要同时遥操和
观察 Web 时，在 Windows 仓库根目录运行：

```powershell
.\scripts\open_piperx_teleop_monitor.ps1
```

联合模式在一个进程内为四台机械臂各创建一个 Piper SDK 连接：主臂控制帧发送
到配对从臂，而从臂同一连接的反馈直接进入力矩估计，不再创建第二个从臂
SocketCAN 接收者。启动器要求 EvoStudio 状态为 `idle`，随后在可见 SSH 窗口
中停止 `evostudio-client`；退出时恢复该服务。SSH 和 sudo 可能要求交互输入
凭据，仓库不保存密码。

启动时保持主从臂静止且姿态接近。任一关节初始差超过 15 度、夹爪超过 10 mm、
固件低于 `S-V1.8-9`、六轴未使能或反馈不完整时都会拒绝控制。页面在联合模式
明确显示 `普通遥操 / CONTROL + MONITOR`，不再声称 `RX ONLY`。

主臂必须已配置为 teaching input（`0xFA`）。程序不会为读取初始姿态而把主臂
临时切成 follower（`0xFC`），因为 Piper 官方 SDK 明确说明主臂收到 follower
角色命令后需要重启机械臂才能生效。若启动窗口提示缺少 teaching frames，
先保持姿态安全，断电重启对应的主臂一次，再重新运行启动器；从臂不需要因此
重启。

## 已知载荷验证

无接触数据只能证明零残差抑制和重复性，不能证明绝对载荷幅值正确。进入 EvoStudio 集成前必须完成物理已知载荷验证。

建议使用经校准的拉力计，或使用已知质量 `m`、重力加速度 `g` 和相对目标关节轴的垂直力臂 `r`。简单单轴工况下预期力矩幅值为：

```text
|tau_expected| = m * g * r
```

方向应根据 URDF 关节轴和施力方向确定。复杂姿态应使用 `tau_expected = J(q)^T F`，而不能只写质量。载荷就绪后使用同一个只读采集入口，但必须选择 `known_load` 标签和匹配的确认参数，不能把载荷数据标成无接触：

```bash
python scripts/piperx_external_torque.py record \
  --arm left \
  --serial 004B00204148570D20343133 \
  --urdf /absolute/path/piper_x_description_no_gripper.urdf \
  --seconds 15 \
  --output data/left-known-load.npz \
  --label known_load \
  --confirm-known-load
```

评分示例：

```bash
python scripts/piperx_external_torque.py known-load \
  --input data/left-known-load.npz \
  --calibration calibration/left.json \
  --expected-torque 0,-1.20,0,0,0,0
```

已知载荷逐关节误差必须不高于 `max(0.30 Nm, 15% * |tau_expected|)`，所有非零预期关节的方向一致率必须至少 95%。

## 验收标准

| 工况 | 标准 |
|---|---|
| 静态无接触 J2/J3 | MAE ≤ 0.30 Nm |
| 静态无接触其他关节 | MAE ≤ 0.15 Nm |
| 自由运动无接触 | 各关节 p95 绝对误差 ≤ 0.50 Nm |
| 已知载荷 | MAE ≤ `max(0.30 Nm, 15%)` 且方向一致率 ≥ 95% |
| 系统影响 | EvoStudio 服务状态和约 200 Hz 从臂反馈率不得下降 |

只有全部标准通过，才能开始 EvoStudio 数据集集成。不能用训练日志本身代替独立验证，也不能用无接触结果冒充已知载荷验证。

## 文件格式和故障排查

每个记录由同名 `.npz` 与 `.json` 组成。JSON 保存身份、标签、样本数和 NPZ SHA-256；缺少任一文件、哈希不匹配或数组维度不一致都会拒绝加载。NPZ 包含：

```text
timestamp_ns, q, qd, qdd,
tau_measured, tau_model, tau_bias, tau_external,
valid, group_id, reason
```

常见失败：

- `adapter serial ... was not found`：检查 USB-CAN 连接和 udev 属性；不要把接口名直接替代序列号。
- `URDF SHA-256 does not match`：恢复标定使用的确切模型，或重新采集并标定；不要修改 JSON 哈希。
- `outside_calibrated_workspace`：当前姿态或速度超出无接触数据范围；扩展安全的无接触标定覆盖。
- `derivative_startup`：启动初始帧，属正常无效区间。
- `excessive_timestamp_gap` / `stale_feedback`：检查系统负载和 CAN 接收，不要放宽限制来掩盖丢帧。
- `acceleration_limit`：可能是速度反馈突变或不合理运动；停止把该区间用于评分并检查原始帧。

## 后续 EvoStudio 契约（当前不实现）

硬件绝对误差验证全部通过后，另开 EvoStudio 变更，将同步后的六关节外力矩写入：

```text
complementary_info.left_follower_joint_torque_external
complementary_info.right_follower_joint_torque_external
complementary_info.follower_joint_torque_external_valid
```

现有 14 维 `observation.state` 与 `action` 保持不变，避免改变当前策略输入输出和数据集兼容性。
