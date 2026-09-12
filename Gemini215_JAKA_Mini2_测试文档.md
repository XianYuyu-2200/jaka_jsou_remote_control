# Gemini 215 + JAKA Mini 2 测试文档

## 1. 测试目标

验证 Gemini 215 安装到 JAKA Mini 2 后，是否适合后续 VLA 数据采集和视觉闭环控制。

重点检查：

- RGB、Depth、IMU 是否稳定输出；
- 0.20–0.50 m 实际工作距离内的深度质量；
- 机械臂运动时的模糊、滚动快门变形和 RGB-D 错位；
- 相机帧能否和关节/TCP状态同步记录；
- 能否生成可回放的完整数据。

Gemini 215 是近距离主动双目相机。官方标称深度范围为 0.15–0.70 m，理想范围约 0.20–0.50 m，深度最高 1280×800@30 fps，RGB最高 1920×1080@30 fps。深度/IR为全局快门，RGB为滚动快门。

## 2. NUC环境

工作目录：

```text
~/codex/codex-jaka_mini_2
```

环境：Ubuntu 22.04.5、ROS 2 Humble、Orbbec ROS 2 Wrapper、JAKA ROS 2驱动、rosbag2 + MCAP。

每个终端先执行：

```bash
cd ~/codex/codex-jaka_mini_2
source scripts/env.bash
```

## 3. 软件分工

- JAKA自带软件：上电、登录、使能、设置负载/TCP、低速点动和第一轮安全运动。
- ROS 2 JAKA驱动：连接机器人、读取关节/TCP状态、记录状态。
- ROS 2 MoveIt：规划、RViz可视化和碰撞检查。

第一次测试不要直接用MoveIt控制真实机械臂。公开JAKA仓库中明确看到的是MiniCobo等模型，不应假定其配置可直接代替Mini 2；使用MoveIt前应向JAKA取得Mini 2专用URDF、运动学和控制器配置。

## 4. 启动顺序

### 4.1 安全和USB检查

1. 检查急停、安全开关和工作空间。
2. 相机接NUC的USB 3口，并固定支架和线缆。
3. 检查USB速度：

```bash
lsusb -t
```

相机应位于 `5000M` 或 `10000M` 总线，不应位于 `480M` 总线。机械臂速度先限制在5%–10%。

### 4.2 环境检查

```bash
./scripts/check_environment.sh
```

### 4.3 启动Gemini 215

VLA RGB-D基线测试建议关闭点云：

```bash
./scripts/start_camera.sh enable_point_cloud:=false
```

需要测试点云时：

```bash
./scripts/start_camera.sh
```

### 4.4 启动JAKA驱动

在另一个终端执行，替换为实际IP：

```bash
./scripts/start_jaka.sh <机器人IP>
```

启动后先不要调用运动服务。

## 5. 相机单机测试

查看话题：

```bash
ros2 topic list | sort
```

重点检查：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
/camera/imu
```

实际命名以现场输出为准。检查帧率：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

连续观察1–2分钟，记录实际帧率、掉帧、断流、重连和终端错误。工程初筛建议目标配置持续接近30 Hz，10分钟内无明显断流，丢帧率尽量低于1%。

查看内参、IMU和图像：

```bash
ros2 topic echo --once /camera/color/camera_info
ros2 topic echo --once /camera/depth/camera_info
ros2 topic echo --once /camera/imu
rviz2
```

## 6. 静态深度测试

将平面和目标物放在 0.20、0.30、0.50、0.65 m处，分别测试画面中心和边缘。至少包含哑光白色、黑色、金属/反光、透明/半透明和实际抓取物体。

记录：

- 深度有效像素比例；
- 平面噪声和边缘飞点；
- 空洞区域；
- RGB与Depth边缘是否对齐；
- 光照变化；
- 失败样例截图。

不要只记录“点云看起来不错”。

## 7. 机械臂测试

### 7.1 先用JAKA自带软件

1. 上电、登录、使能；
2. 设置相机、支架和夹爪的真实负载与重心；
3. 设置正确TCP；
4. 低速点动和手动拖动；
5. 回到安全位姿；
6. 检查报警、碰撞、奇异位姿和线缆拉扯。

### 7.2 ROS 2只读状态

```bash
ros2 topic list | grep jaka
ros2 topic hz /jaka_driver/joint_position
ros2 topic hz /jaka_driver/tool_position
ros2 topic echo /jaka_driver/robot_states
```

用JAKA软件移动时，同时观察ROS 2是否持续发布关节和TCP状态。不要直接复制网上的 `linear_move` 示例；坐标系、单位、速度和位姿必须根据现场状态确认。

## 8. 相机和机械臂联动

相机安装后按顺序：

1. 机械臂静止，观察RGB、Depth和机器人状态；
2. 用JAKA软件低速移动；
3. 检查RGB运动模糊、滚动快门变形和RGB-D错位；
4. 检查相机是否保持在0.20–0.50 m；
5. 检查支架、相机和线缆是否碰撞；
6. 再以未来任务的正常速度重复。

## 9. 数据录制

```bash
./scripts/record_test.sh static_01
```

建议至少录制：

```text
static_01：机械臂和目标静止
slow_01：低速移动相机
task_01：模拟真实抓取或操作
```

结束录制按 `Ctrl+C`，查看：

```bash
ros2 bag info ~/codex/codex-jaka_mini_2/bags/static_01
```

数据至少应包含RGB、Depth、camera_info、IMU、joint_position、tool_position和robot_states。

JAKA运动指令通常通过服务发送，rosbag不一定自动保存服务请求。VLA数据还要单独记录action发送时间、目标位姿、夹爪状态、episode编号和标定版本，否则可能只有观测没有动作标签。

## 10. 时间戳检查

相机启动脚本会生成时间戳CSV：

```bash
ls -lt ~/codex/codex-jaka_mini_2/logs/
tail -20 ~/codex/codex-jaka_mini_2/logs/camera_timestamps_*.csv
```

检查时间戳是否单调递增、RGB和Depth是否持续有帧、运动时是否掉帧，以及相机硬件时间、主机接收时间和机器人状态时间能否关联。

## 11. 最低通过标准

- RGB和Depth持续接近30 Hz；
- 没有频繁断流或重连；
- 0.20–0.50 m范围内深度可用；
- RGB、Depth、IMU和camera_info可读取；
- JAKA关节和TCP话题持续发布；
- 机械臂运动时相机和线缆安全；
- 能录制并回放完整MCAP；
- 静止、低速、真实任务三类数据均能保存；
- 失败样例、配置、固件和标定信息均已记录。

## 12. 常见问题

### 相机检测不到

```bash
lsusb -t
ros2 run orbbec_camera list_devices_node
```

检查USB线、USB 3端口和udev规则，必要时重新插拔相机。

### RGB有图像但Depth为空

检查距离、物体材质、光照和深度参数。黑色、透明、反光物体可能产生大量无效深度。

### JAKA驱动没有状态

检查机器人IP、网络、控制器登录状态、机器人使能状态，以及是否有其他JAKA客户端占用连接。

### MoveIt无法执行

不要直接修改真实机器人参数。先确认Mini 2专用URDF、关节限制、控制器和模型名称。
