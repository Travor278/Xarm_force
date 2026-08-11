# PiperX 夹爪负载与反馈链硬件验收（2026-08-11）

## 范围

- 本地实现提交：`f8031c0`、`5c038c5`。
- 目标机：`.166`，独立部署目录 `/home/dell/piperx-force-validation`。
- 未修改 `evostudio-piperx`、EvoStudio 服务或 LeRobot 数据集。
- 实机检查仅使用 receive-only SocketCAN 和短时 loopback Web 服务；未发送运动/夹爪命令。

## 官方负载模型

来源为 `agilexrobotics/piper_ros` 提交 `ac41fcbcdda598f01b51cf6175ed9a24d0dacadc` 的 `piper_description.urdf`：

- `gripper_base`: 0.450 kg；
- `link7`: 0.025 kg；
- `link8`: 0.025 kg；
- 合计：0.500 kg。

三者在 35 mm 参考开度下合成到 `link6`，运行时负载 SHA-256 为：

```text
2d8b80465d91da8384d86f2faddfcb36330ef47a9c26e5d0e7216cb4866ad9ac
```

质量和质心对称合成后不随开度变化；开度只带来较小的等效惯量变化。

## 动力学影响

在两臂原始无接触日志上比较“无夹爪负载”与“0.500 kg 夹爪负载”的 `tau_model` 差值：

| arm | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| left mean abs (N·m) | 0.0000 | 0.5734 | 1.9589 | 0.6376 | 0.0004 | 0.0007 |
| right mean abs (N·m) | 0.0001 | 0.5779 | 1.9629 | 0.6406 | 0.0024 | 0.0007 |

该量级说明遗漏夹爪会显著改变 J2-J4 的重力模型，显式加入负载是必要的。J5 在这批姿态中影响很小，不代表其他姿态恒为零。

## 重建残差标定

原始 NPZ 未覆盖；新日志保存为 `*-with-gripper.npz`。旧标定备份：

```text
calibration/left.no-payload-20260811.json
calibration/right.no-payload-20260811.json
```

当前 `calibration/left.json` 和 `right.json` 已绑定同一负载 SHA。独立静态验证结果：

| arm | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| left MAE (N·m) | 0.00586 | 0.00354 | 0.00362 | 0.00368 | 0.00435 | 0.00388 |
| right MAE (N·m) | 0.00270 | 0.00244 | 0.00405 | 0.00487 | 0.00510 | 0.00400 |

两臂均通过既有静态阈值；右臂 2998 个有效源样本中有 3 个位于标定工作区外并被拒绝。

## 0x2A8 实机接收

同时只读监听 can2/can3 三秒：

| interface | frames | rate | latest travel | latest torque | status |
|---|---:|---:|---:|---:|---:|
| can2 | 600 | 200.0 Hz | 0.5 mm | 0.017 N·m | `0x40` |
| can3 | 600 | 200.0 Hz | 0.4 mm | 0.093 N·m | `0x40` |

`0x40` 表示夹爪已使能，欠压/过温/过流/传感器/驱动错误位均为 0；回零位未置位。

短时 Web 端到端检查返回 `piperx-monitor-v2`，两臂夹爪字段均为 fresh，原始行程和力矩有限。未加载力计工件时：

```text
force_n = null
force_valid = false
force_calibrated = false
force_reason = uncalibrated
```

检查结束后远端 `18765` 无监听进程，PiperX monitor/teleop 测试进程已退出。

## 软件回归

- `.166` Python：146 passed（包含真实 Pinocchio 负载附加测试）。
- `.166` Node：11 passed。
- 本地 `tests/`：142 passed, 4 skipped；4 个跳过项在 `.166` 已通过。
- 浏览器完整页面验收：夹爪面板、双标尺趋势图和状态布局无重叠。

## 尚未声称完成的项目

没有独立力计真值，当前不能声称“指尖力 N 已准确”。需要按 `docs/piperx_gripper_force_calibration.md` 为左右夹爪分别采集已知力点；在此之前只能使用官方反馈电机力矩 N·m 观察趋势。
