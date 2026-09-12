# JAKA JSOU Remote Control

JAKA Mini2 双臂遥操作、关节控制与关节轨迹录制/回放控制台。

## 当前功能

- Windows 原生 GUI，支持真实运动授权和 Dry-run 安全模式；
- 操作臂拖拽，跟随臂按相对关节映射跟随；
- 选择操作臂或跟随臂进行关节 +/- 点动；
- 操作臂和跟随臂分别保存安全姿态参数；
- 同臂关节轨迹录制与相对起点回放；
- LPF/NLF 滤波、速度、加速度和每周期步长限制；
- JAKA Mini2 官方分轴关节范围；
- GUI 同目录日志和关节轨迹 CSV 输出。

## 目录

- `windows_jaka/`：Windows GUI、operator、follower 和测试源码；
- `jaka_dual_readonly/`：原只读/单关节验证代码；
- `docs/`：设计和实现文档；
- `nuc_setup/`：辅助环境脚本；
- `启动控制台.cmd`：自动设置当前进程真实运动授权并启动 GUI；
- `launch-real-motion-gui.cmd`：兼容旧入口。

## 多机器人配置

多机器人管理的数据模型位于 `windows_jaka/include/robot_registry.hpp`，示例配置为 `config/robots.example.ini`。

配置支持：

- 多台机器人名称、型号、IP 和使能状态；
- 每台机器人独立的滤波器、速度、加速度、关节限位、关节方向和安全姿态；
- 一个遥操作组配置 1 台操作臂和多台跟随臂；
- 配置加载、保存与完整性校验。

当前已经完成多机器人配置数据层和自动化测试；GUI 的机器人列表、添加/编辑/删除、分组选择和 1:N 遥操作运行管理器将在后续提交中继续实现。
## 构建

需要：

- 64 位 Windows；
- Visual Studio 2022 C++ Build Tools；
- CMake 3.20 或更高版本；
- 官方 JAKA Windows x64 SDK，包含 `JAKAZuRobot.h`、`jakaAPI.lib` 和 `jakaAPI.dll`。

示例：

    cmake -S windows_jaka -B build/windows_jaka-vs -G "Visual Studio 17 2022" -A x64 -DJAKA_SDK_ROOT="C:/path/to/JAKA_SDK"
    cmake --build build/windows_jaka-vs --config Release
    ctest --test-dir build/windows_jaka-vs -C Release --output-on-failure

仓库不包含 JAKA 厂商 SDK、`jakaAPI.dll` 或已编译 exe。请在目标机使用官方 SDK 和编译产物。

## 安全默认值

- GUI 默认 Dry-run；
- 真实运动需要 `JAKA_ENABLE_MOTION=1`、GUI 授权复选框和控制模式同时满足；
- 程序不会自动 power_on、enable_robot 或解除急停；
- 回放采用相对起点，不保证回到录制时的绝对姿态；
- 首次连接新机械臂必须重新核对关节方向、零位、限位、速度和安全姿态。
