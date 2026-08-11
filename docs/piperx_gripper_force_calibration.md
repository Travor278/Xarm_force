# PiperX 夹爪负载与指尖力标定

## 已实现的数据链

机械臂动力学继续保持 J1-J6 六自由度。程序在 J6 上附加官方 Piper 夹爪的等效刚体负载：质量 0.500 kg，质心和惯量来自 AgileX 官方 `piper_ros` 模型中 `gripper_base`、`link7`、`link8` 的合成结果。参数及上游提交号记录在 `config/piperx_gripper_payload.json`。

夹爪 CAN ID `0x2A8` 直接提供：

- 行程：int32，0.001 mm；
- 反馈电机力矩：int16，0.001 N·m；
- 欠压、过温、过流、传感器、驱动错误、使能和回零状态。

该帧没有夹爪电流字段，因此程序不会从不存在的电流推算力矩。网页始终可以显示原始行程、反馈力矩和状态；只有加载实测标定工件后才显示 N。

## 指尖力定义

本工具中的 `force_n` 定义为放在两指之间的校准拉压力计/薄膜力传感器的压缩读数。不要把两个指尖的接触力绝对值再相加，否则会产生二倍口径差异。

标定模型是每台从臂独立的有符号仿射关系：

```text
force_n = max(0, slope * direction * feedback_torque_nm + intercept)
```

工件绑定 USB-CAN/从臂身份，并记录标定时覆盖的行程和力矩范围。遥测过期、夹爪未使能、任一故障位、设备不匹配或超出标定范围时，网页保留原始 N·m，但 N 值失效并断线。

## 用力计采点

保持正常遥操运行，把传感器稳固放在两指之间。每个力级稳定后，从 `.166` 的部署目录运行一次只读采点：

```bash
python scripts/piperx_gripper_force.py capture \
  --serial 004B00204148570D20343133 \
  --known-force 10 \
  --seconds 2 \
  --output data/left-gripper-force.csv \
  --confirm-known-force
```

建议每臂至少采集 8-12 个点，覆盖约 0、5、10、15、20、30、40 N，并在 15、35、55 mm 等不同开度重复。`capture` 只打开接收套接字，不发送 CAN 帧，也不改变遥操目标。

右臂使用其从臂序列号和独立 CSV：

```text
left  004B00204148570D20343133
right 003F002D4148571320343133
```

## 拟合和验收

```bash
python scripts/piperx_gripper_force.py fit \
  --input data/left-gripper-force.csv \
  --serial 004B00204148570D20343133 \
  --output calibration/left-gripper-force.json

python scripts/piperx_gripper_force.py evaluate \
  --input data/left-gripper-force.csv \
  --calibration calibration/left-gripper-force.json \
  --max-mae 5
```

正式验收应另外采集未参加拟合的力级/开度 CSV。建议先采用 MAE ≤ 5 N；若需要抓取脆弱物体，应改用更严格阈值和更适合低量程的传感器。

启动网页时按臂加载工件：

```bash
python scripts/piperx_teleop_web.py \
  ...原有参数... \
  --gripper-force left=calibration/left-gripper-force.json \
  --gripper-force right=calibration/right-gripper-force.json
```

在两台夹爪完成独立标定前，网页中的夹持力应为 `-- N / uncalibrated`。原始反馈力矩曲线仍可用于检查夹紧趋势，但不能当成已验收的指尖力。
