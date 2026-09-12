# JAKA 双臂遥操作：当前可验证版本

当前两个 JAKA Mini 2 尚未到货，Windows 工作区没有 ROS 2、JAKA SDK 或 CMake，因此现在先实现并验证不依赖硬件的控制核心。

## 已实现

- 六关节主从零位、方向和比例映射；
- 每周期速度、加速度和位置限幅；
- 24 ms HOLD、100 ms FAULT 的看门狗；
- deadman 松开后故障锁存；
- 真实控制器到货后可复用的 `TeleopController` 接口；
- 虚拟主臂轨迹和JSONL动作记录。

代码：

- [teleop_core.py](F:/codex/codex-jaka%20mini%202/teleop_core.py)
- [teleop_sim.py](F:/codex/codex-jaka%20mini%202/teleop_sim.py)
- [test_teleop_core.py](F:/codex/codex-jaka%20mini%202/tests/test_teleop_core.py)

## 运行无硬件仿真

```powershell
python teleop_sim.py
```

输出：

```text
bags/virtual_teleop.jsonl
```

每行包含：

```text
leader、command、command_sent、state、reason
```

## 运行测试

```powershell
python -m unittest discover -v
```

当前测试覆盖相机健康检查和遥操作核心，共21个测试。

## 机械臂到货后的替换点

虚拟流程最终替换为：

```text
真实主臂 get_joint_position()
    -> TeleopController.enable/process()
    -> 真实从臂 servo_j(ABS, step_num=1)
```

真实实现应放在 Ubuntu NUC 的 ROS 2 包中：

1. `LeaderNode` 独立连接主臂，125 Hz读取关节角；
2. `FollowerNode` 独立连接从臂，8 ms调用JAKA `servo_j`；
3. `TeleopController`保留当前映射、限速和故障逻辑；
4. 主臂、从臂和deadman状态写入MCAP；
5. 真实模式默认关闭，完成单关节±2°测试后再开放六关节。

当前版本**不会连接机器人，也不会发送任何运动指令**。
