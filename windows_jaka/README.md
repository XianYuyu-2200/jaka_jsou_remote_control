# Windows 原生 JAKA Mini 2 六轴遥操作

此目录是独立的 Windows 实现，不修改 `jaka_dual_readonly/` 中的 Linux 代码。

目标数据流：

```text
操作臂 192.168.0.101 <--SDK只读/关节伺服--> windows_operator.exe
                                      --UDP 127.0.0.1:30001-->
跟随臂 192.168.0.102 <--SDK servo_j------> windows_follower.exe
```

## 安全默认值

- 默认只运行 dry-run，`servo_j` 不会被调用。
- 真实运动必须同时满足：命令行 `--arm-motion`，以及环境变量 `JAKA_ENABLE_MOTION=1`。
- 关节控制和安全姿态可以选择操作臂或跟随臂；同一会话只让选中的对象进入 `joint` 模式，另一对象保持 `idle`。
- 操作臂和跟随臂的安全姿态默认参数彼此独立，切换“关节/安全姿态对象”时会分别保存和恢复各自的输入值。
- 程序不会自动 `power_on`、`enable_robot` 或解除急停。
- 首次拖动时捕获操作臂和跟随臂零点，目标为相对映射。
- UDP 接收线程只保留最大 `sequence` 的最新报文；历史目标不排队执行。
- follower 的唯一 SDK 线程使用 8 ms 周期 waitable timer；SDK 状态读取只在该线程的启动阶段执行，servo 循环不做阻塞状态查询。
- `alpha=0.85`；J1–J3 每周期最大目标变化 `0.008 rad`，J4–J6 为 `0.006 rad`，并保留速度/加速度限制。默认软件上限为 `1.0 rad/s`、`8.0 rad/s²`，可用 follower 的 `--max-velocity` 与 `--max-acceleration` 调低或调高；每周期位移上限始终生效。
- JAKA servo filter 可用 `none`、`lpf` 或 `nlf` 三选一；启动时只配置一次，不能把 LPF 和 NLF 同时启用。
- 数据年龄达到 40 ms 后保持最后安全目标；达到 100 ms 停止并执行急停清理。
- 非法 UDP 包、非有限关节值、序列倒退、SDK 错误和超出软件安全范围都会终止跟随。

当前默认软件关节范围按 JAKA 官方《JAKA2026选型册》（Mini2 章节）配置为：J1 ±360°、J2 ±125°、J3 ±130°、J4 ±360°、J5 ±120°、J6 ±360°。控制器内实际配置的软限位可能更窄，应以现场机器人配置为准。关节限位器只在真正进入运动模式时初始化，idle 和 Dry-run 不会因当前姿态被判超限而退出。若发生超限，日志会输出 `follower_zero_raw_rad` 或 `operator_zero_raw_rad` 的六个原始值以及按角度换算的参考值。

follower 运行结束时会输出 `receive_avg_ms`、`receive_max_ms`、操作端时间戳到 servo tick 的 `packet_age_avg_ms`/`packet_age_max_ms`、`servo_interval_avg_ms`、`servo_call_max_ms`、`dropped_packets`、`hold_ticks`、`watchdog_ticks`、每轴 `max_target_delta_rad` 以及 `max_command_error_rad`。后者可直接显示软件滤波/限速造成的目标滞后。

推荐的第一轮低通配置：

```powershell
.\windows_jaka\run_windows_teleop.ps1 -DryRun -Filter lpf -LpfCutoff 2.5
```

如需现场调低/调高软件响应速度，可传入：

```powershell
.\windows_jaka\run_windows_teleop.ps1 -DryRun -MaxVelocity 1.0 -MaxAcceleration 8.0
```

`--filter nlf` 使用 JAKA SDK 的 joint NLF 参数 `(45.0, 600.0, 6000.0)`。这里的滤波器是 `servo_j` 关节伺服滤波，不是 torque sensor filter。当前实机验证较好的 LPF 截止参数为 `2.5`，因此 Windows 脚本和 follower 默认值均为 `2.5`。

## SDK 路径

使用官方 Windows x64 SDK，例如：

```text
SDK V2.3.1_beta3/Windows/WINDOWS_MSVC_V142_x64
```

该目录下应能找到：

```text
Windows/c&c++/inc_of_c++/JAKAZuRobot.h
Windows/c&c++/x64/jakaAPI.lib
Windows/c&c++/x64/jakaAPI.dll
```

## 配置和构建

在 x64 Native Tools Command Prompt for VS 2022 或已加载 MSVC 环境中执行：

```powershell
cmake -S windows_jaka -B build/windows_jaka `
  -G "Visual Studio 17 2022" `
  -A x64 `
  -DJAKA_SDK_ROOT="C:/path/to/SDK V2.3.1_beta3/Windows/WINDOWS_MSVC_V142_x64"

cmake --build build/windows_jaka --config Release
ctest --test-dir build/windows_jaka -C Release --output-on-failure
```

如果用户级 Build Tools 没有被 Visual Studio 生成器注册，可使用已经加载
`VsDevCmd.bat` 的 x64 开发命令行：

```bat
cmake -S windows_jaka -B build/windows_jaka-release ^
  -G "NMake Makefiles" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DJAKA_SDK_ROOT="C:/path/to/SDK V2.3.1_beta3/Windows/WINDOWS_MSVC_V142_x64"
cmake --build build/windows_jaka-release
ctest --test-dir build/windows_jaka-release --output-on-failure
```

测试目标包括：

- `packet_test`：80 字节报文、magic/version、非有限值拒绝
- `mapping_test`：六轴相对零位映射
- `dry_run_test`：低通、速度和加速度限制，验证首帧不会跳变
- `udp_loopback_test`：Winsock2 localhost UDP 回环（Windows 构建时启用）

如果暂时只运行不依赖 SDK 的主机测试：

```powershell
cmake -S windows_jaka -B build/windows_jaka-tests `
  -G "Visual Studio 17 2022" -A x64 `
  -DWINDOWS_JAKA_TEST_ONLY=ON
cmake --build build/windows_jaka-tests --config Release
ctest --test-dir build/windows_jaka-tests -C Release --output-on-failure
```

## Dry-run 启动

Dry-run 会启动 follower/operator、登录两台机器人、读取状态、接收 UDP 并运行目标映射与安全检查，但**不会调用 `servo_j`**。因此它不会让跟随臂运动，也不能用来验证关节是否真的会动。

```powershell
.\windows_jaka\run_windows_teleop.ps1 -DryRun -Filter lpf -LpfCutoff 2.5
```

Dry-run 下关节 `+/-` 和拖拽跟随都保持禁用，这是安全设计，不是按钮故障。

## GUI 控制台

Release 构建会生成：

```text
build/windows_jaka-verify/Release/windows_teleop_gui.exe
```

GUI 将控制方式设计为互斥模式：

- **遥操作**：操作臂在拖拽模式下运动，跟随臂按相对关节映射跟随；关节 `+/-` 被隐藏并锁定。
- **关节控制**：先在“关节/安全姿态对象”中选择跟随臂或操作臂，再按住 J1–J6 的 `+/-` 点动选中对象；遥操作映射在该模式下不参与控制。
- **Dry-run 检查**：只验证登录、UDP、状态读取和映射逻辑，不调用 `servo_j`，不会产生跟随或点动动作。
- **停止**：结束当前会话并回到待机；只有待机状态才能切换到另一种控制模式。

推荐操作顺序：

1. 确认两台机器人已上电、使能，现场人员可立即按下急停，并确保运动路径无障碍。
2. 双击仓库根目录的 `启动控制台.cmd`。该入口只会对本次 GUI 进程自动设置 `JAKA_ENABLE_MOTION=1`，不需要手工输入 PowerShell 命令。
3. GUI 仍默认使用 Dry-run，确认现场安全后再勾选“真实运动授权（现场确认安全）”。
4. 点击“遥操作”或“关节控制”。当前模式按钮会变为青绿色选中状态，页面也会切换到对应说明。
5. 需要切换控制方式时，先点击“停止”，等待日志出现“会话进程已退出”和“待机”，再选择另一模式。

没有勾选 GUI 授权，或 GUI 不是通过自动授权入口启动的，真实运动按钮不会进入可控状态。环境变量只在当前 GUI 进程内生效，不写入系统。

GUI 使用两条本地命名管道传递点动、停止和安全姿态命令：`\\.\pipe\jaka_dual_teleop` 对应跟随臂 `windows_follower.exe`，`\\.\pipe\jaka_operator_teleop` 对应操作臂 `windows_operator.exe`。命名管道线程只解析命令，实际 JAKA SDK 调用由对应进程的 SDK 线程执行。关节按钮采用“按住持续点动、松开立即停止”，GUI 每 8 ms 刷新一次当前点动目标；两个进程都设置了 250 ms 无刷新自动过期保护。

安全姿态输入单位始终为角度（°），范围 -180° 到 180°。跟随臂和操作臂的默认值分别由 `windows_jaka/src/windows_teleop_gui.cpp` 中的 `kFollowerDefaultPoseDegrees` 和 `kOperatorDefaultPoseDegrees` 控制；切换“关节/安全姿态对象”时，GUI 会分别保存并恢复两套输入值。当前两套默认值都初始化为 [-80, 45, -90, 0, -90, -90]，如果两臂实际参数不同，请分别修改这两个数组后重新编译。执行前仍必须由现场人员确认，程序会在发送命令前自动完成底层单位换算。已有会话运行时，GUI 会阻止启动另一种模式。GUI 不会自动替代硬件急停，也不会在未授权时执行运动。

## 关节轨迹录制与回放

GUI 的“轨迹录制”和“轨迹回放”用于同一条机械臂的示教与复现：

- “关节/安全姿态/轨迹对象”选择跟随臂或操作臂。
- “轨迹录制”只让选中机械臂对应的进程进入 record 模式，另一条臂保持 idle。
- 录制时可物理缓慢拖动该臂，也可在录制页面按住 J1-J6 的 `+/-` 点动。
- 轨迹保存到 GUI 同目录的 `logs` 文件夹，文件名格式为 `trajectory_operator_YYYYMMDD_HHMMSS.csv` 或 `trajectory_follower_YYYYMMDD_HHMMSS.csv`。
- 文件格式为 `time_ms,j1,j2,j3,j4,j5,j6`，关节角单位为弧度。
- “轨迹回放”选择 CSV 后，只启动选中机械臂的 playback 模式，另一条臂保持 idle。
- 回放采用相对起点方式：从该臂当前实际关节位置建立偏移，因此不会跳到录制时的绝对起点。
- 回放时间轴按 `time_ms` 插值到 8 ms 控制周期，速度和加速度仍经过现有 `JointLimiter`、低通滤波与每周期步长限制。
- 当前 GUI 固定按 1 倍速回放；需要暂停、变速或循环时，后续可继续增加。
- 回放结束后进程自动退出；回放过程中点击“停止”会立即请求停止并执行 servo 清理。

回放前仍需勾选真实运动授权，且 GUI 会再次弹出确认窗口。程序不会自动使能机器人，也不会替代硬件急停。

注意：轨迹回放是相对运动复现，不是恢复到录制时的绝对安全姿态。若要求绝对位姿复现，应另行增加绝对轨迹校验和起点对准流程。
## 多组会话

机器人管理界面支持同时运行多个互不共享机器人的会话：

- 每个遥操作组独立进程树；
- 每组自动分配不同的 UDP 端口段；
- 同一台机器人不能同时加入两个运行会话；
- 选择任意组或机器人可单独停止；
- “全部停止”一次停止所有会话；
- 每台机器人状态独立显示。

## 多机器人管理 GUI

`windows_robot_manager.exe` 提供初始机器人/分组管理界面：

- 添加、编辑、删除机器人；
- 编辑名称、型号、IP、滤波器、速度、加速度、关节限位、方向和安全姿态；
- 添加、编辑、删除遥操作组；
- 配置一个操作臂和多个跟随臂；
- 读写 `config/robots.ini`。

仓库根目录可双击 `启动机器人管理.cmd`。选择遥操作组后，管理器可以直接启动 Dry-run 或真实运动组会话，并可停止当前组。组启动器 `run_group_from_registry.ps1` 会从 `robots.ini` 读取每台机器人的 IP 和伺服参数；未选中的机器人不会启动。运行期间每个 worker 会周期性写入 `status/<robot-id>.status`，管理界面显示模式、连接、上电、使能、拖动、伺服、故障、数据年龄和序列号。

## 单台机器人控制

机器人管理界面可以在机器人列表中选择一台机械臂，然后启动：

- 单臂关节控制；
- 单臂轨迹录制；
- 单臂轨迹回放；
- 停止当前单台会话。

单台模式只启动选中的机械臂，不会启动其他机器人。底层脚本为 `run_single_robot.ps1`，会从 `robots.ini` 读取该机器人的 IP、滤波器、速度、加速度和安全参数，并写入独立状态文件。关节控制/录制模式下，管理界面提供 J1-J6 按住点动和“执行安全姿态”；回放模式锁定点动，避免控制冲突。

## 一台操作臂驱动多台跟随臂

`run_windows_multi_teleop.ps1` 提供 1:N 遥操作运行链路：

- 一个 `windows_operator.exe` 进程读取操作臂；
- 同一个 80 字节关节数据包同时发送到 N 个本地 UDP 端口；
- 每台 `windows_follower.exe` 使用独立端口和独立命名管道；
- 未列出的机械臂不会启动运动控制。

示例：

    $env:JAKA_ENABLE_MOTION = "1"
    .\windows_jaka\run_windows_multi_teleop.ps1 `
      -OperatorIp 192.168.1.101 `
      -FollowerIps 192.168.1.102,192.168.1.103 `
      -BasePort 30001 `
      -ArmMotion

Dry-run 示例：

    .\windows_jaka\run_windows_multi_teleop.ps1 `
      -OperatorIp 192.168.1.101 `
      -FollowerIps 192.168.1.102,192.168.1.103 `
      -DryRun

该脚本已经具备 1:N 数据分发能力；机器人 GUI 管理、分组选择和进程状态汇总仍在继续开发。
## 命令行控制模式（不使用 GUI）

`run_windows_teleop.ps1` 新增：

```text
-ControlMode idle|teleop|joint
-OperatorControlMode idle|teleop|joint
```

含义如下：

- `-ControlMode` 控制跟随臂 `windows_follower.exe`；`teleop` 是命令行默认值。
- `-OperatorControlMode` 控制操作臂 `windows_operator.exe`；`idle` 是命令行默认值。
- `idle`：该臂不接受关节 JOG；遥操作时操作臂只负责提供拖拽状态和关节数据。
- `teleop`：操作臂和跟随臂组成相对映射；GUI 的“遥操作”模式使用该组合。
- `joint`：该臂接受 J1–J6 关节 JOG；同一会话不要把两个对象都设为 `joint`。
- 如果使用 `-DryRun`，以上模式都不会调用 `servo_j`。

真实遥操作示例：

```powershell
$env:JAKA_ENABLE_MOTION = "1"
.\windows_jaka\run_windows_teleop.ps1 `
  -OperatorIp 192.168.0.101 `
  -FollowerIp 192.168.0.102 `
  -Port 30001 `
  -ArmMotion `
  -ControlMode teleop `
  -OperatorControlMode teleop
```

真实跟随臂关节控制示例：

```powershell
$env:JAKA_ENABLE_MOTION = "1"
.\windows_jaka\run_windows_teleop.ps1 `
  -OperatorIp 192.168.0.101 `
  -FollowerIp 192.168.0.102 `
  -Port 30001 `
  -ArmMotion `
  -ControlMode joint `
  -OperatorControlMode idle
```

真实操作臂关节控制示例：

```powershell
$env:JAKA_ENABLE_MOTION = "1"
.\windows_jaka\run_windows_teleop.ps1 `
  -OperatorIp 192.168.0.101 `
  -FollowerIp 192.168.0.102 `
  -Port 30001 `
  -ArmMotion `
  -ControlMode idle `
  -OperatorControlMode joint
```

## 真实运动启动条件

只在完成编译、packet/mapping/dry-run/UDP loopback、两台机器人只读登录和现场安全检查后执行：

```powershell
$env:JAKA_ENABLE_MOTION = "1"
.\windows_jaka\run_windows_teleop.ps1 `
  -OperatorIp 192.168.0.101 `
  -FollowerIp 192.168.0.102 `
  -Port 30001 `
  -ArmMotion `
  -ControlMode teleop`
  -OperatorControlMode teleop
```

启动时必须看到：

```text
REAL ROBOT MOTION ENABLED
```

未同时满足两个授权条件时，两个进程都只做 dry-run，不调用 `servo_j`。

## 本机实际编译记录（2026-09-09）

已安装并验证：

- Visual Studio 2022 Build Tools 17.14.40
- MSVC 19.44.35228.0 x64
- Windows SDK 10.0.26100.0
- CMake 3.31.6
- 官方 JAKA SDK V2.3.1_beta3 Windows MSVC V142 x64

实际使用的配置命令：

```powershell
cmake -S windows_jaka -B build/windows_jaka-vs `
  -G "Visual Studio 17 2022" -A x64 `
  -DJAKA_SDK_ROOT="C:/path/to/SDK V2.3.1_beta3/Windows/WINDOWS_MSVC_V142_x64"

cmake --build build/windows_jaka-vs --config Release
ctest --test-dir build/windows_jaka-vs -C Release --output-on-failure
```

实际结果：

```text
windows_operator.exe       Release 构建成功
windows_follower.exe      Release 构建成功
packet_test                Passed
mapping_test               Passed
dry_run_test               Passed
udp_loopback_test          Passed
100% tests passed, 0 tests failed
```

Release 输出位于：

```text
build/windows_jaka-vs/Release/
```

其中包含 `jakaAPI.dll`。已用 `dumpbin /DEPENDENTS` 确认可执行文件依赖 `jakaAPI.dll` 和 `WS2_32.dll`。本次验证没有连接机械臂，也没有产生真实运动。

