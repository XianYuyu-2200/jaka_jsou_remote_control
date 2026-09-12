# JAKA Mini 2 只读点位记录器设计

## 目标

在 Ubuntu 上运行一个独立命令行程序，连接任务机械臂
`192.168.0.102`。用户通过 JAKA 手动点动或拖拽模式将机械臂移动到位，
程序只读取并保存实际关节位置和实际 TCP 位姿，不发送任何机械臂运动、
上电、使能或停机指令。

## 命令行接口

默认启动方式：

```bash
./waypoint_recorder \
  --robot-ip 192.168.0.102 \
  --output coffee_waypoints.yaml
```

交互命令：

- `show`：读取并显示当前状态，不保存。
- `save P1` 至 `save P11`：稳定性检查通过后新增点位。
- `overwrite P1`：明确覆盖已经存在的点位。
- `list`：列出已记录点位及确认状态。
- `delete P1`：明确删除点位。
- `quit`：保存文件、退出并注销 SDK 会话。
- `help`：显示命令说明。

点位名仅接受 `P1` 至 `P11`，普通 `save` 不允许静默覆盖已有点位。

任务点位语义为：P1 杯子下方预抓取点，P2 杯子抓取点，P3 杯子安全下降点，
P4 出水口前方预放置点，P5 出水口正下方杯子放置点，P6 按钮前方预按压点，
P7 按钮最大允许按压点，P8 按钮撤回点，P9 装满后杯子重新抓取点，P10 装满后
安全抬升点，P11 固定人手交接点。

## 读取与稳定性判定

程序使用 JAKA 官方 C++ SDK：

```cpp
get_actual_joint_position(...)
get_actual_tcp_position(...)
```

保存时以 50 Hz 连续采样 1 秒，即 50 组样本。以下条件全部满足才允许保存：

- 50 次关节和 TCP 读取全部成功。
- 任一关节在采样窗口内的最大值与最小值之差不超过 `0.003 rad`。
- TCP 的 X、Y、Z 每一轴最大值与最小值之差不超过 `1.0 mm`。

保存值采用每一维样本的中位数。RPY 只保存中位数，不用于稳定性拒绝，避免
角度在 `-pi/pi` 附近换向造成误判。每个点初始写入 `confirmed: false`。

## 输出文件

输出为无需第三方 YAML 库即可生成的 UTF-8 YAML：

```yaml
metadata:
  robot_ip: "192.168.0.102"
  coordinate_frame: "current_controller_user_frame"
  tcp_frame: "current_controller_tool_tcp"
  joint_units: "rad"
  translation_units: "mm"
  rotation_units: "rad"
  hand_state_available: false

waypoints:
  P1:
    name: "cup_below_pregrasp"
    captured_at: "2026-08-28T00:00:00+08:00"
    joint_position_rad: [0, 0, 0, 0, 0, 0]
    tcp_pose:
      xyz_mm: [0, 0, 0]
      rpy_rad: [0, 0, 0]
    hand_position: null
    confirmed: false
```

现阶段 Ubuntu 尚未接管 O6 Modbus，因此不能把图形化程序中的灵巧手位置伪装成
同步读数，`hand_position` 固定为 `null`。完成 O6 驱动后再扩展记录接口。

## 安全约束

生产源码和启动脚本中禁止出现以下调用：

- `power_on`、`power_off`
- `enable_robot`、`disable_robot`
- `servo_j`、`servo_move_enable`
- `joint_move`、`linear_move`、`circular_move`
- `motion_abort`
- 数字或模拟输出写入

程序只允许登录、读取实际状态、写本地 YAML 和注销。登录失败、读取失败、文件
解析失败或保存失败时，程序报告错误并保持原文件不被部分覆盖。文件更新先写临时
文件，再原子替换正式文件。

## 测试

离线单元测试覆盖：

1. 稳定样本能够生成中位数点位。
2. 关节波动超限时拒绝保存。
3. TCP 波动超限时拒绝保存。
4. 读取失败时拒绝保存。
5. `save` 不覆盖已有点位，`overwrite` 可以明确覆盖。
6. YAML 可由 Python `yaml.safe_load` 解析并保持数值、布尔值和空值类型。
7. 静态扫描确认生产源码不包含任何禁用 SDK 调用。

真机阶段只执行登录、`show`、保存一个测试点及注销；不自动移动机械臂。
