# Gemini 215 + JAKA Mini 2 快速测试说明

## 1. 启动前

将 Gemini 215 接到 NUC 的 USB 3.0 端口，然后检查：

```bash
lsusb -t
```

相机应位于 `5000M` 或 `10000M` 总线，不应是 `480M`。

每个终端先执行：

```bash
cd ~/codex/codex-jaka_mini_2
source scripts/env.bash
```

## 2. 启动相机

推荐先关闭点云：

```bash
./scripts/start_camera.sh enable_point_cloud:=false
```

需要测试点云时：

```bash
./scripts/start_camera.sh
```

## 3. 启动机械臂

先使用JAKA自带软件完成上电、登录、使能、负载/TCP设置，并将速度限制在5%–10%。

然后在另一个终端启动ROS 2驱动：

```bash
./scripts/start_jaka.sh <机器人IP>
```

第一次测试不要直接用MoveIt控制真实机械臂。先用JAKA软件手动拖动或低速点动，同时由ROS 2读取状态。

## 4. 检查状态

```bash
ros2 topic list | sort
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /jaka_driver/joint_position
ros2 topic hz /jaka_driver/tool_position
```

如果话题名称不同，以 `ros2 topic list` 的实际结果为准。

## 5. 运行自动测试脚本

### 只测试相机

```bash
./scripts/run_gemini_jaka_test.sh --camera-only
```

### 相机和机械臂一起测试

```bash
./scripts/run_gemini_jaka_test.sh --duration 5
```

脚本会逐项输出独立结果，例如：

```text
01. Orbbec ROS package = True
02. USB 3.x link available = True
03. RGB topic = True
04. Depth topic = False
05. RGB rate >= 25 Hz = True
```

每个 `True/False` 只代表对应检查项，不是所有项目的总结果。

如果需要JSON结果：

```bash
./scripts/run_gemini_jaka_test.sh --camera-only --json
```

## 6. 录制测试数据

```bash
./scripts/record_test.sh static_01
```

建议录制三组：

```text
static_01：机械臂静止
slow_01：低速移动
task_01：模拟真实抓取
```

结束录制按 `Ctrl+C`，查看数据：

```bash
ros2 bag info ~/codex/codex-jaka_mini_2/bags/static_01
```

## 7. 最低测试标准

- RGB和Depth接近30 Hz；
- 没有频繁断流；
- 0.20–0.50 m范围内深度可用；
- RGB、Depth、IMU话题可读取；
- JAKA关节和TCP状态持续发布；
- 机械臂运动时相机、支架和线缆不碰撞；
- 能成功录制MCAP数据。

注意：JAKA运动指令通常通过服务发送，rosbag不一定自动保存动作指令。后续VLA数据还需要单独记录目标位姿、夹爪状态和动作时间戳。
