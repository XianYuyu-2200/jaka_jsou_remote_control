# JAKA Mini 2 Dual-Arm Teleoperation Implementation Plan

> **最终角色定义（2026-08-27）：** 运行时代码不再使用容易产生歧义的
> `leader/master`、`follower/slave`。人工操作的 `operator_arm` 地址为
> `192.168.0.101`；自动跟随的 `tracking_arm` 地址为 `192.168.0.102`；
> 数据及未来运动命令方向为 `.101 -> .102`。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于 JAKA 公开 C++ SDK，实现一套“主臂拖拽、从臂 125 Hz 关节伺服跟随”的基础遥操作系统，并为后续 VLA 数据采集发布完整、可同步记录的动作与状态话题。

**Architecture:** 主臂和从臂分别由独立进程持有一个 JAKA SDK 会话，以避免 SDK 全局状态或线程安全问题。主臂节点以 125 Hz 读取关节角并发布；从臂节点使用本机单调时钟运行 8 ms 控制循环，完成零位映射、速度/加速度限制、关节限位、看门狗和 `servo_j(ABS)` 下发。ROS 2 只承担本机进程间通信、控制开关、监控和录制，不使用逐周期 ROS 服务调用控制从臂。

**Tech Stack:** Ubuntu 22.04、ROS 2 Humble、C++17、JAKA C++ SDK、ament_cmake、GoogleTest、rosbag2/MCAP、yaml-cpp。

---

## 0. 实施边界和已知约束

本计划只实现基础六关节遥操作和可录制接口，不实现 Gemini 305/335L、灵巧手、ACT、LingBot 或倒水状态机。这些系统通过本计划预留的 ROS 话题在后续接入。

公开 SDK 已确认提供：

- `login_in()` / `login_out()`
- `get_joint_position()`
- `drag_mode_enable()`
- `servo_move_enable()`
- `servo_j(..., ABS, step_num)`
- `motion_abort()`
- `is_in_collision()`
- `get_robot_status_simple()`

JAKA 文档说明 `servo_j` 周期为 `step_num × 8 ms`。本计划使用 `step_num=1`，因此控制线程周期为 8 ms。

现有官方 ROS 2 驱动不用于高速遥操作闭环，因为当前公开实现以约 100 ms 周期读取状态，并且 `servo_j` ROS 服务回调采用增量模式，示例还逐次等待服务返回。基础遥操作直接调用 SDK 的绝对关节伺服接口。

当前工作目录不是 Git 仓库。下面的提交步骤仅在工程被纳入 Git 后执行；不要为了执行本计划自动覆盖或移动用户现有文件。

## 0.1 物理连接和网络拓扑

```text
JAKA主臂控制柜 ── 千兆以太网 ──┐
                               ├── 千兆交换机 ── 千兆以太网 ── Ubuntu控制电脑
JAKA从臂控制柜 ── 千兆以太网 ──┘

USB脚踏开关 ── USB ── Ubuntu控制电脑
```

连接规则：

- 两台机器人使用各自原厂控制柜、电源和急停。
- 两台控制柜与控制电脑连接同一个独立千兆交换机。
- 不使用Wi-Fi，不把机器人控制流量与大规模数据下载共用网络。
- 不把两台控制柜的24V、普通I/O或急停线路自行并联。
- 如果需要一个硬件急停同时切断两台机器人，必须依据两台控制柜的安全电气手册由合格集成商实现；本计划的软件停止不具备安全认证。

推荐静态地址：

```text
Ubuntu控制电脑：10.5.5.10/24
主臂控制柜：    10.5.5.101/24
从臂控制柜：    10.5.5.102/24
```

两台机器人如果出厂默认IP相同，必须一次只连接一台完成改址，再同时接入交换机。

## 1. 目标文件结构

在 NUC 的 Linux 工程根目录 `~/codex/codex-jaka_mini_2` 下新增独立工作空间：

```text
teleop_ws/
└── src/
    └── jaka_dual_teleop/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── msg/
        │   ├── JointSample.msg
        │   └── TeleopStatus.msg
        ├── include/jaka_dual_teleop/
        │   ├── robot_api.hpp
        │   ├── joint_mapper.hpp
        │   ├── joint_limiter.hpp
        │   ├── watchdog.hpp
        │   └── steady_loop.hpp
        ├── src/
        │   ├── jaka_robot_api.cpp
        │   ├── leader_node.cpp
        │   ├── follower_node.cpp
        │   └── sdk_smoke_test.cpp
        ├── config/
        │   └── mini2_same_mount.yaml
        ├── launch/
        │   └── dual_teleop.launch.py
        ├── scripts/
        │   ├── check_teleop_network.sh
        │   ├── deadman_evdev.py
        │   ├── start_teleop.sh
        │   └── record_teleop.sh
        └── test/
            ├── test_joint_mapper.cpp
            ├── test_joint_limiter.cpp
            └── test_watchdog.cpp
```

文件职责：

- `robot_api.*`：隔离厂商 SDK，便于单元测试和未来 SDK 升级。
- `joint_mapper.*`：主从零位、方向和比例映射。
- `joint_limiter.*`：关节限位、速度和加速度限制。
- `watchdog.*`：主臂状态超时和故障锁存。
- `steady_loop.hpp`：基于 `steady_clock` 的 8 ms 周期调度。
- `leader_node.cpp`：持有主臂 SDK，会话、拖拽状态和关节发布。
- `follower_node.cpp`：持有从臂 SDK，会话、遥操作状态机和 ServoJ 控制线程。
- `sdk_smoke_test.cpp`：不运动；每个进程只验证一台机器人登录和状态读取。
- `JointSample.msg`：主臂高频关节采样。
- `TeleopStatus.msg`：用于 UI、MCAP 和故障分析的统一状态。

## Task 1: 创建 ROS 2 包并验证 SDK 链接

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/package.xml`
- Create: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`
- Create: `teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/robot_api.hpp`
- Create: `teleop_ws/src/jaka_dual_teleop/src/jaka_robot_api.cpp`
- Create: `teleop_ws/src/jaka_dual_teleop/src/sdk_smoke_test.cpp`

- [ ] **Step 1: 创建包清单**

`package.xml` 使用以下依赖：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>jaka_dual_teleop</name>
  <version>0.1.0</version>
  <description>JAKA Mini 2 leader-follower teleoperation using the public JAKA C++ SDK.</description>
  <maintainer email="robotics@example.com">robotics</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>builtin_interfaces</depend>
  <depend>yaml_cpp_vendor</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>python3-evdev</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: 定义 SDK 抽象接口**

`robot_api.hpp`：

```cpp
#pragma once

#include <array>
#include <memory>
#include <string>

namespace jaka_dual_teleop {

using JointArray = std::array<double, 6>;

struct RobotHealth {
  bool powered_on{false};
  bool enabled{false};
  bool collision{false};
  int sdk_code{0};
};

class RobotApi {
 public:
  virtual ~RobotApi() = default;
  virtual int login(const std::string &ip) = 0;
  virtual int logout() = 0;
  virtual int get_joint_position(JointArray &position) = 0;
  virtual int get_health(RobotHealth &health) = 0;
  virtual int set_drag_mode(bool enabled) = 0;
  virtual int set_servo_mode(bool enabled) = 0;
  virtual int servo_j_absolute(const JointArray &target, unsigned int step_num) = 0;
  virtual int motion_abort() = 0;
};

std::unique_ptr<RobotApi> make_jaka_robot_api();

}  // namespace jaka_dual_teleop
```

- [ ] **Step 3: 实现厂商 SDK 适配器**

`jaka_robot_api.cpp`：

```cpp
#include "jaka_dual_teleop/robot_api.hpp"

#include "JAKAZuRobot.h"
#include "jktypes.h"

namespace jaka_dual_teleop {
namespace {

class JakaRobotApi final : public RobotApi {
 public:
  int login(const std::string &ip) override {
    return robot_.login_in(ip.c_str(), false);
  }

  int logout() override { return robot_.login_out(); }

  int get_joint_position(JointArray &position) override {
    JointValue value{};
    const int code = robot_.get_joint_position(&value);
    if (code == 0) {
      for (std::size_t i = 0; i < position.size(); ++i) {
        position[i] = value.jVal[i];
      }
    }
    return code;
  }

  int get_health(RobotHealth &health) override {
    RobotStatus_simple status{};
    BOOL collision = FALSE;
    const int status_code = robot_.get_robot_status_simple(&status);
    if (status_code != 0) {
      health.sdk_code = status_code;
      return status_code;
    }
    const int collision_code = robot_.is_in_collision(&collision);
    if (collision_code != 0) {
      health.sdk_code = collision_code;
      return collision_code;
    }
    health.powered_on = status.powered_on != 0;
    health.enabled = status.enabled != 0;
    health.collision = collision != FALSE;
    health.sdk_code = 0;
    return 0;
  }

  int set_drag_mode(bool enabled) override {
    return robot_.drag_mode_enable(enabled ? TRUE : FALSE);
  }

  int set_servo_mode(bool enabled) override {
    return robot_.servo_move_enable(enabled ? TRUE : FALSE);
  }

  int servo_j_absolute(const JointArray &target, unsigned int step_num) override {
    JointValue value{};
    for (std::size_t i = 0; i < target.size(); ++i) {
      value.jVal[i] = target[i];
    }
    return robot_.servo_j(&value, MoveMode::ABS, step_num);
  }

  int motion_abort() override { return robot_.motion_abort(); }

 private:
  JAKAZuRobot robot_;
};

}  // namespace

std::unique_ptr<RobotApi> make_jaka_robot_api() {
  return std::make_unique<JakaRobotApi>();
}

}  // namespace jaka_dual_teleop
```

- [ ] **Step 4: 添加只读 SDK 冒烟测试程序**

`sdk_smoke_test.cpp` 每个进程只连接一台机器人，不假定当前 SDK 能在同一进程安全维持两个会话：

```cpp
#include "jaka_dual_teleop/robot_api.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: sdk_smoke_test <robot_ip>\n";
    return 2;
  }

  auto robot = jaka_dual_teleop::make_jaka_robot_api();
  const int login_code = robot->login(argv[1]);
  if (login_code != 0) {
    std::cerr << "login failed code=" << login_code << '\n';
    return 1;
  }

  for (int sample = 0; sample < 100; ++sample) {
    jaka_dual_teleop::JointArray q{};
    const int code = robot->get_joint_position(q);
    if (code != 0) {
      std::cerr << "read failed code=" << code << '\n';
      return 1;
    }
    std::cout << std::fixed << std::setprecision(6)
              << "sample=" << sample
              << " q0=" << q[0] << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(8));
  }

  robot->logout();
  return 0;
}
```

- [ ] **Step 5: 编写 CMake SDK 发现和链接规则**

`CMakeLists.txt` 必须要求用户设置 `JAKA_SDK_ROOT`，不得静默链接未知版本：

```cmake
cmake_minimum_required(VERSION 3.16)
project(jaka_dual_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT DEFINED ENV{JAKA_SDK_ROOT})
  message(FATAL_ERROR "Set JAKA_SDK_ROOT to the extracted official JAKA SDK directory")
endif()

find_path(JAKA_SDK_INCLUDE_DIR JAKAZuRobot.h
  HINTS
    $ENV{JAKA_SDK_ROOT}/include
    $ENV{JAKA_SDK_ROOT}/inc_of_c++
    "$ENV{JAKA_SDK_ROOT}/c&c++/inc_of_c++")
find_library(JAKA_SDK_LIBRARY jakaAPI
  HINTS
    $ENV{JAKA_SDK_ROOT}/lib
    $ENV{JAKA_SDK_ROOT}/x86_64-linux-gnu/shared
    "$ENV{JAKA_SDK_ROOT}/c&c++/x86_64-linux-gnu/shared")

if(NOT JAKA_SDK_INCLUDE_DIR OR NOT JAKA_SDK_LIBRARY)
  message(FATAL_ERROR "JAKA headers or libjakaAPI.so not found under JAKA_SDK_ROOT")
endif()

add_library(jaka_robot_api src/jaka_robot_api.cpp)
target_include_directories(jaka_robot_api PUBLIC include ${JAKA_SDK_INCLUDE_DIR})
target_link_libraries(jaka_robot_api PUBLIC ${JAKA_SDK_LIBRARY} pthread)

add_executable(sdk_smoke_test src/sdk_smoke_test.cpp)
target_link_libraries(sdk_smoke_test PRIVATE jaka_robot_api)

install(TARGETS jaka_robot_api sdk_smoke_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES ${JAKA_SDK_LIBRARY} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()
```

- [ ] **Step 6: 构建并确认 SDK 可链接**

Run:

```bash
cd ~/codex/codex-jaka_mini_2
source /opt/ros/humble/setup.bash
export JAKA_SDK_ROOT="$HOME/vendor/jaka_sdk"
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
```

Expected: `Finished <<< jaka_dual_teleop`，且没有 `JAKAZuRobot.h not found` 或 `libjakaAPI.so not found`。

- [ ] **Step 7: 在两台机器人已上电但不运动时执行只读测试**

Run:

```bash
source teleop_ws/install/setup.bash
ros2 run jaka_dual_teleop sdk_smoke_test 10.5.5.101
ros2 run jaka_dual_teleop sdk_smoke_test 10.5.5.102
```

Expected: 两次运行均连续输出100个样本并以退出码0结束；机器人不发生运动。正式遥操作仍使用两个独立进程分别持有 SDK 会话。

- [ ] **Step 8: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop
git commit -m "build: scaffold JAKA dual teleop SDK package"
```

## Task 2: 定义高频消息和遥操作状态

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/msg/JointSample.msg`
- Create: `teleop_ws/src/jaka_dual_teleop/msg/TeleopStatus.msg`

- [ ] **Step 1: 定义主臂关节采样消息**

`JointSample.msg`：

```text
builtin_interfaces/Time ros_stamp
uint64 monotonic_ns
uint64 sequence
float64[6] position
bool drag_enabled
bool valid
int32 sdk_code
```

`monotonic_ns` 用于本机看门狗；`ros_stamp` 用于 rosbag 与相机数据的近似同步。不得使用 ROS 系统时间判断控制超时。

- [ ] **Step 2: 定义统一状态消息**

`TeleopStatus.msg`：

```text
uint8 DISCONNECTED=0
uint8 STANDBY=1
uint8 ARMED=2
uint8 RUNNING=3
uint8 HOLDING=4
uint8 FAULT=5

builtin_interfaces/Time ros_stamp
uint64 monotonic_ns
uint8 state
string reason
uint64 leader_sequence
float64 leader_age_ms
float64[6] leader_position
float64[6] follower_measured
float64[6] mapped_target
float64[6] sent_command
bool command_sent
float64[6] tracking_error
bool follower_powered
bool follower_enabled
bool follower_collision
bool deadman_active
int32 sdk_code
```

- [ ] **Step 3: 构建接口**

在 `CMakeLists.txt` 的库目标之前加入：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointSample.msg"
  "msg/TeleopStatus.msg"
  DEPENDENCIES builtin_interfaces)
```

Run:

```bash
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
source teleop_ws/install/setup.bash
ros2 interface show jaka_dual_teleop/msg/JointSample
ros2 interface show jaka_dual_teleop/msg/TeleopStatus
```

Expected: 两个接口完整显示且数组长度均为6。

- [ ] **Step 4: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/msg teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: define teleoperation messages"
```

## Task 3: 实现并测试主从关节映射

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/joint_mapper.hpp`
- Create: `teleop_ws/src/jaka_dual_teleop/test/test_joint_mapper.cpp`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

`test_joint_mapper.cpp`：

```cpp
#include <gtest/gtest.h>
#include "jaka_dual_teleop/joint_mapper.hpp"

using jaka_dual_teleop::JointArray;
using jaka_dual_teleop::JointMapper;

TEST(JointMapper, PreservesFollowerPoseAtArmTime) {
  const JointArray sign{1, 1, 1, 1, 1, 1};
  const JointArray scale{1, 1, 1, 1, 1, 1};
  JointMapper mapper(sign, scale);
  const JointArray leader_zero{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const JointArray follower_zero{-0.1, -0.2, -0.3, -0.4, -0.5, -0.6};
  mapper.arm(leader_zero, follower_zero);
  EXPECT_EQ(mapper.map(leader_zero), follower_zero);
}

TEST(JointMapper, AppliesSignAndScaleToLeaderDelta) {
  const JointArray sign{1, -1, 1, -1, 1, -1};
  const JointArray scale{1, 1, 0.5, 0.5, 2, 2};
  JointMapper mapper(sign, scale);
  mapper.arm(JointArray{0, 0, 0, 0, 0, 0}, JointArray{1, 1, 1, 1, 1, 1});
  const auto target = mapper.map(JointArray{0.1, 0.1, 0.2, 0.2, 0.1, 0.1});
  const JointArray expected{1.1, 0.9, 1.1, 0.9, 1.2, 0.8};
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(target[i], expected[i], 1e-12);
  }
}

TEST(JointMapper, RejectsInvalidSign) {
  EXPECT_THROW(
    JointMapper(JointArray{1, 1, 0, 1, 1, 1}, JointArray{1, 1, 1, 1, 1, 1}),
    std::invalid_argument);
}

TEST(JointMapper, ZeroScaleFreezesAJointAtFollowerZero) {
  JointMapper mapper(
    JointArray{1, 1, 1, 1, 1, 1},
    JointArray{1, 0, 1, 1, 1, 1});
  mapper.arm(JointArray{0, 0, 0, 0, 0, 0}, JointArray{1, 2, 3, 4, 5, 6});
  const auto target = mapper.map(JointArray{0.5, 0.5, 0, 0, 0, 0});
  EXPECT_DOUBLE_EQ(target[1], 2.0);
}
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_joint_mapper --output-on-failure
```

Expected: FAIL，因为 `joint_mapper.hpp` 尚不存在。

- [ ] **Step 3: 实现最小关节映射器**

`joint_mapper.hpp`：

```cpp
#pragma once

#include "jaka_dual_teleop/robot_api.hpp"

#include <cmath>
#include <stdexcept>

namespace jaka_dual_teleop {

class JointMapper {
 public:
  JointMapper(const JointArray &sign, const JointArray &scale)
      : sign_(sign), scale_(scale) {
    for (std::size_t i = 0; i < 6; ++i) {
      if (std::abs(std::abs(sign_[i]) - 1.0) > 1e-12) {
        throw std::invalid_argument("joint sign must be +1 or -1");
      }
      if (scale_[i] < 0.0) {
        throw std::invalid_argument("joint scale must be non-negative");
      }
    }
  }

  void arm(const JointArray &leader_zero, const JointArray &follower_zero) {
    leader_zero_ = leader_zero;
    follower_zero_ = follower_zero;
    armed_ = true;
  }

  JointArray map(const JointArray &leader) const {
    if (!armed_) {
      throw std::logic_error("joint mapper is not armed");
    }
    JointArray target{};
    for (std::size_t i = 0; i < 6; ++i) {
      target[i] = follower_zero_[i] + sign_[i] * scale_[i] *
        (leader[i] - leader_zero_[i]);
    }
    return target;
  }

 private:
  JointArray sign_{};
  JointArray scale_{};
  JointArray leader_zero_{};
  JointArray follower_zero_{};
  bool armed_{false};
};

}  // namespace jaka_dual_teleop
```

- [ ] **Step 4: 注册并运行测试**

在 `CMakeLists.txt` 的 `ament_package()` 前加入：

```cmake
if(BUILD_TESTING)
  ament_add_gtest(test_joint_mapper test/test_joint_mapper.cpp)
  target_include_directories(test_joint_mapper PRIVATE include)
endif()
```

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_joint_mapper --output-on-failure
```

Expected: `100% tests passed`。

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/joint_mapper.hpp \
        teleop_ws/src/jaka_dual_teleop/test/test_joint_mapper.cpp \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add zero-relative joint mapping"
```

## Task 4: 实现关节限位、速度和加速度限制

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/joint_limiter.hpp`
- Create: `teleop_ws/src/jaka_dual_teleop/test/test_joint_limiter.cpp`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

`test_joint_limiter.cpp`：

```cpp
#include <gtest/gtest.h>
#include "jaka_dual_teleop/joint_limiter.hpp"

using jaka_dual_teleop::JointArray;
using jaka_dual_teleop::JointLimiter;

TEST(JointLimiter, LimitsVelocityPerCycle) {
  JointLimiter limiter(
    JointArray{-3, -3, -3, -3, -3, -3},
    JointArray{ 3,  3,  3,  3,  3,  3},
    JointArray{0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    JointArray{2.0, 2.0, 2.0, 2.0, 2.0, 2.0});
  limiter.reset(JointArray{0, 0, 0, 0, 0, 0});
  const auto command = limiter.step(JointArray{1, 1, 1, 1, 1, 1}, 0.008);
  for (double value : command) {
    EXPECT_NEAR(value, 0.000128, 1e-12);
  }
}

TEST(JointLimiter, NeverExceedsJointLimits) {
  JointLimiter limiter(
    JointArray{-1, -1, -1, -1, -1, -1},
    JointArray{ 1,  1,  1,  1,  1,  1},
    JointArray{10, 10, 10, 10, 10, 10},
    JointArray{100, 100, 100, 100, 100, 100});
  limiter.reset(JointArray{0.99, 0.99, 0.99, 0.99, 0.99, 0.99});
  auto command = JointArray{};
  for (int i = 0; i < 100; ++i) {
    command = limiter.step(JointArray{2, 2, 2, 2, 2, 2}, 0.008);
  }
  for (double value : command) {
    EXPECT_LE(value, 1.0);
  }
}
```

第一周期速度从0开始，最大加速度为2 rad/s²，因此速度增量为 `2×0.008=0.016 rad/s`，位置增量为 `0.016×0.008=0.000128 rad`。

- [ ] **Step 2: 运行测试确认失败**

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_joint_limiter --output-on-failure
```

Expected: FAIL，因为 `joint_limiter.hpp` 尚不存在。

- [ ] **Step 3: 实现限制器**

`joint_limiter.hpp`：

```cpp
#pragma once

#include "jaka_dual_teleop/robot_api.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace jaka_dual_teleop {

class JointLimiter {
 public:
  JointLimiter(
      const JointArray &lower,
      const JointArray &upper,
      const JointArray &max_velocity,
      const JointArray &max_acceleration)
      : lower_(lower), upper_(upper), max_velocity_(max_velocity),
        max_acceleration_(max_acceleration) {
    for (std::size_t i = 0; i < 6; ++i) {
      if (!(lower_[i] < upper_[i]) || max_velocity_[i] <= 0.0 ||
          max_acceleration_[i] <= 0.0) {
        throw std::invalid_argument("invalid joint limit configuration");
      }
    }
  }

  void reset(const JointArray &position) {
    command_ = position;
    velocity_.fill(0.0);
    initialized_ = true;
  }

  JointArray step(const JointArray &desired, double dt) {
    if (!initialized_ || dt <= 0.0) {
      throw std::logic_error("joint limiter is not initialized or dt is invalid");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      const double bounded_desired = std::clamp(desired[i], lower_[i], upper_[i]);
      const double requested_velocity = std::clamp(
        (bounded_desired - command_[i]) / dt,
        -max_velocity_[i], max_velocity_[i]);
      const double dv = std::clamp(
        requested_velocity - velocity_[i],
        -max_acceleration_[i] * dt,
         max_acceleration_[i] * dt);
      velocity_[i] += dv;
      command_[i] = std::clamp(
        command_[i] + velocity_[i] * dt,
        lower_[i], upper_[i]);
    }
    return command_;
  }

  const JointArray &command() const { return command_; }

 private:
  JointArray lower_{};
  JointArray upper_{};
  JointArray max_velocity_{};
  JointArray max_acceleration_{};
  JointArray command_{};
  JointArray velocity_{};
  bool initialized_{false};
};

}  // namespace jaka_dual_teleop
```

- [ ] **Step 4: 注册并运行测试**

在 `CMakeLists.txt` 测试区加入：

```cmake
ament_add_gtest(test_joint_limiter test/test_joint_limiter.cpp)
target_include_directories(test_joint_limiter PRIVATE include)
```

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_joint_limiter --output-on-failure
```

Expected: `100% tests passed`。

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/joint_limiter.hpp \
        teleop_ws/src/jaka_dual_teleop/test/test_joint_limiter.cpp \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add joint motion limits"
```

## Task 5: 实现看门狗和故障锁存

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/watchdog.hpp`
- Create: `teleop_ws/src/jaka_dual_teleop/test/test_watchdog.cpp`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

`test_watchdog.cpp`：

```cpp
#include <gtest/gtest.h>
#include "jaka_dual_teleop/watchdog.hpp"

using jaka_dual_teleop::Watchdog;
using jaka_dual_teleop::WatchdogResult;

TEST(Watchdog, TransitionsFromFreshToHoldingToFault) {
  Watchdog watchdog(24.0, 100.0);
  EXPECT_EQ(watchdog.evaluate(10.0), WatchdogResult::FRESH);
  EXPECT_EQ(watchdog.evaluate(30.0), WatchdogResult::HOLD);
  EXPECT_EQ(watchdog.evaluate(101.0), WatchdogResult::FAULT);
}

TEST(Watchdog, FaultIsLatchedUntilReset) {
  Watchdog watchdog(24.0, 100.0);
  EXPECT_EQ(watchdog.evaluate(101.0), WatchdogResult::FAULT);
  EXPECT_EQ(watchdog.evaluate(1.0), WatchdogResult::FAULT);
  watchdog.reset();
  EXPECT_EQ(watchdog.evaluate(1.0), WatchdogResult::FRESH);
}
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_watchdog --output-on-failure
```

Expected: FAIL，因为 `watchdog.hpp` 尚不存在。

- [ ] **Step 3: 实现看门狗**

`watchdog.hpp`：

```cpp
#pragma once

#include <stdexcept>

namespace jaka_dual_teleop {

enum class WatchdogResult { FRESH, HOLD, FAULT };

class Watchdog {
 public:
  Watchdog(double hold_after_ms, double fault_after_ms)
      : hold_after_ms_(hold_after_ms), fault_after_ms_(fault_after_ms) {
    if (!(hold_after_ms_ > 0.0 && fault_after_ms_ > hold_after_ms_)) {
      throw std::invalid_argument("invalid watchdog thresholds");
    }
  }

  WatchdogResult evaluate(double age_ms) {
    if (fault_latched_ || age_ms >= fault_after_ms_) {
      fault_latched_ = true;
      return WatchdogResult::FAULT;
    }
    if (age_ms >= hold_after_ms_) {
      return WatchdogResult::HOLD;
    }
    return WatchdogResult::FRESH;
  }

  void reset() { fault_latched_ = false; }

 private:
  double hold_after_ms_;
  double fault_after_ms_;
  bool fault_latched_{false};
};

}  // namespace jaka_dual_teleop
```

- [ ] **Step 4: 注册并运行测试**

在 `CMakeLists.txt` 测试区加入：

```cmake
ament_add_gtest(test_watchdog test/test_watchdog.cpp)
target_include_directories(test_watchdog PRIVATE include)
```

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --ctest-args -R test_watchdog --output-on-failure
```

Expected: `100% tests passed`。

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/watchdog.hpp \
        teleop_ws/src/jaka_dual_teleop/test/test_watchdog.cpp \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add teleoperation watchdog"
```

## Task 6: 实现精确 8 ms 调度工具

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/steady_loop.hpp`

- [ ] **Step 1: 添加基于绝对截止时间的循环调度器**

`steady_loop.hpp`：

```cpp
#pragma once

#include <chrono>
#include <thread>

namespace jaka_dual_teleop {

class SteadyLoop {
 public:
  explicit SteadyLoop(std::chrono::nanoseconds period)
      : period_(period), next_(std::chrono::steady_clock::now() + period_) {}

  double wait_next() {
    std::this_thread::sleep_until(next_);
    const auto now = std::chrono::steady_clock::now();
    const auto scheduled = next_;
    do {
      next_ += period_;
    } while (next_ <= now);
    return std::chrono::duration<double>(now - scheduled).count();
  }

 private:
  std::chrono::nanoseconds period_;
  std::chrono::steady_clock::time_point next_;
};

}  // namespace jaka_dual_teleop
```

使用绝对截止时间而非 `sleep_for(8ms)`，避免每周期执行时间累积成长期漂移。

- [ ] **Step 2: 添加一个非实时调度检查到冒烟程序**

在 `sdk_smoke_test.cpp` 中把固定 `sleep_for` 替换为：

```cpp
#include "jaka_dual_teleop/steady_loop.hpp"

jaka_dual_teleop::SteadyLoop loop(std::chrono::milliseconds(8));
double worst_lateness_s = 0.0;
for (int sample = 0; sample < 100; ++sample) {
  jaka_dual_teleop::JointArray q{};
  const int code = robot->get_joint_position(q);
  if (code != 0) {
    std::cerr << "read failed code=" << code << '\n';
    return 1;
  }
  std::cout << std::fixed << std::setprecision(6)
            << "sample=" << sample << " q0=" << q[0] << '\n';
  worst_lateness_s = std::max(worst_lateness_s, loop.wait_next());
}
std::cout << "worst_lateness_ms=" << worst_lateness_s * 1000.0 << '\n';
```

- [ ] **Step 3: 构建并执行**

Run:

```bash
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
source teleop_ws/install/setup.bash
ros2 run jaka_dual_teleop sdk_smoke_test 10.5.5.101
```

Expected: 完成100次读取，并输出 `worst_lateness_ms`。普通Linux调试阶段允许偶发抖动，但若经常超过8 ms，应先排查CPU负载和SDK调用耗时，不进入运动测试。

- [ ] **Step 4: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/include/jaka_dual_teleop/steady_loop.hpp \
        teleop_ws/src/jaka_dual_teleop/src/sdk_smoke_test.cpp
git commit -m "feat: add steady 8 ms loop scheduler"
```

## Task 7: 实现主臂读取节点

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/src/leader_node.cpp`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 实现主臂节点参数和控制服务**

`leader_node.cpp` 的行为必须满足：

- 参数 `robot_ip`，默认空字符串，空值时拒绝启动。
- 参数 `publish_rate_hz=125.0`。
- 参数 `manage_drag_mode=true`。
- 发布相对话题 `joint_sample`，放入 `/leader` 命名空间后解析为 `/leader/joint_sample`。
- 提供相对服务 `set_drag`，放入 `/leader` 命名空间后解析为 `/leader/set_drag`。
- 服务启用前检查机器人已上电且使能。
- 进程退出时尝试关闭拖拽并退出登录。
- SDK连续3次读取失败后发布 `valid=false`，不得伪造旧数据为新样本。

核心节点代码：

```cpp
#include "jaka_dual_teleop/msg/joint_sample.hpp"
#include "jaka_dual_teleop/robot_api.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace jaka_dual_teleop {

class LeaderNode final : public rclcpp::Node {
 public:
  LeaderNode() : Node("leader") {
    const auto ip = declare_parameter<std::string>("robot_ip", "");
    const double rate = declare_parameter<double>("publish_rate_hz", 125.0);
    manage_drag_ = declare_parameter<bool>("manage_drag_mode", true);
    if (ip.empty() || rate <= 0.0) {
      throw std::runtime_error("robot_ip and positive publish_rate_hz are required");
    }
    api_ = make_jaka_robot_api();
    const int code = api_->login(ip);
    if (code != 0) {
      throw std::runtime_error("leader login failed: " + std::to_string(code));
    }
    publisher_ = create_publisher<msg::JointSample>(
      "joint_sample", rclcpp::SensorDataQoS().keep_last(1));
    service_ = create_service<std_srvs::srv::SetBool>(
      "set_drag",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        RobotHealth health{};
        int code = api_->get_health(health);
        if (code == 0 && request->data && (!health.powered_on || !health.enabled)) {
          response->success = false;
          response->message = "leader must be powered and enabled";
          return;
        }
        code = manage_drag_ ? api_->set_drag_mode(request->data) : 0;
        response->success = code == 0;
        response->message = response->success ? "drag state updated" :
          "SDK error " + std::to_string(code);
        if (response->success) {
          drag_enabled_.store(request->data);
        }
      });
    worker_ = std::thread([this, rate] { run(rate); });
  }

  ~LeaderNode() override {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    if (api_) {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      if (manage_drag_ && drag_enabled_.load()) api_->set_drag_mode(false);
      api_->logout();
    }
  }

 private:
  static uint64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void run(double rate) {
    const auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1e9 / rate));
    auto next = std::chrono::steady_clock::now();
    uint64_t sequence = 0;
    int consecutive_errors = 0;
    while (rclcpp::ok() && !stop_.load()) {
      next += period;
      JointArray q{};
      int code = 0;
      {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        code = api_->get_joint_position(q);
      }
      consecutive_errors = code == 0 ? 0 : consecutive_errors + 1;
      msg::JointSample sample;
      sample.ros_stamp = now().to_msg();
      sample.monotonic_ns = monotonic_ns();
      sample.sequence = sequence++;
      sample.position = q;
      sample.drag_enabled = drag_enabled_.load();
      sample.valid = code == 0 && consecutive_errors < 3;
      sample.sdk_code = code;
      publisher_->publish(sample);
      std::this_thread::sleep_until(next);
    }
  }

  std::unique_ptr<RobotApi> api_;
  rclcpp::Publisher<msg::JointSample>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> drag_enabled_{false};
  std::mutex sdk_mutex_;
  bool manage_drag_{true};
};

}  // namespace jaka_dual_teleop

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jaka_dual_teleop::LeaderNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("leader"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: 添加构建规则**

在 `CMakeLists.txt` 中加入：

```cmake
add_executable(leader_node src/leader_node.cpp)
target_include_directories(leader_node PRIVATE include)
target_link_libraries(leader_node PRIVATE jaka_robot_api)
ament_target_dependencies(leader_node rclcpp std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(leader_node PRIVATE ${cpp_typesupport_target})
install(TARGETS leader_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: 构建并做只读话题测试**

Run:

```bash
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
source teleop_ws/install/setup.bash
ros2 run jaka_dual_teleop leader_node --ros-args -p robot_ip:=10.5.5.101
```

另一个终端：

```bash
ros2 topic hz /leader/joint_sample
ros2 topic echo --once /leader/joint_sample
```

Expected: 接近125 Hz，`valid=true`；尚未调用服务时 `drag_enabled=false`。

- [ ] **Step 4: 单独测试主臂拖拽开关**

机器人周围清空并有人持急停后运行：

```bash
ros2 service call /leader/set_drag std_srvs/srv/SetBool '{data: true}'
```

Expected: 服务成功，主臂可安全手动拖动，从臂仍不连接、不运动。

关闭：

```bash
ros2 service call /leader/set_drag std_srvs/srv/SetBool '{data: false}'
```

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/src/leader_node.cpp \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: publish leader joint samples"
```

## Task 8: 实现从臂安全伺服节点

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/src/follower_node.cpp`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 实现从臂状态机**

状态定义：

```text
DISCONNECTED：SDK未登录。
STANDBY：已登录，仅监控，从臂servo关闭。
ARMED：捕获主从零位，等待新鲜主臂样本。
RUNNING：8 ms发送绝对ServoJ目标。
HOLDING：主臂样本24–100 ms陈旧，保持最后命令。
FAULT：超时、碰撞、SDK错误、跟踪误差或限位异常锁存。
```

相对服务 `set_enabled` 放入 `/follower` 命名空间后解析为 `/follower/set_enabled`。服务规则：

- `true`：只有主臂最新消息 `valid=true`、`drag_enabled=true`、消息年龄小于24 ms、从臂上电且使能、无碰撞、deadman心跳有效时才允许；捕获当时主从关节位置作为零位，然后调用 `servo_move_enable(true)`。
- `false`：调用 `motion_abort()`、`servo_move_enable(false)`，清除运行状态并回到 `STANDBY`。
- `FAULT` 只能通过 `false` 清除；不得收到新数据后自动恢复运动。

服务回调只把请求写入原子变量，SDK调用统一由8 ms控制线程执行，避免同一个SDK会话被ROS回调线程和控制线程并发调用。服务返回“请求已排队”；调用者以 `/follower/status` 确认实际状态。

- [ ] **Step 2: 实现8 ms控制循环**

`follower_node.cpp` 中必须遵循以下完整循环顺序：

```cpp
while (rclcpp::ok() && !stop_requested) {
  wait_until_next_8ms_deadline();
  read_latest_leader_sample_from_mutex_protected_buffer();
  read_follower_joint_position();
  every_10_cycles_read_follower_health();

  if (collision || !powered || !enabled || sdk_read_failed) {
    enter_fault_and_abort();
    continue;
  }

  if (!teleop_enabled) {
    publish_standby_status();
    continue;
  }

  const double leader_age_ms = steady_now_minus_sample_monotonic_ns();
  const WatchdogResult watchdog_result = watchdog.evaluate(leader_age_ms);
  if (watchdog_result == WatchdogResult::FAULT || !sample.valid ||
      !sample.drag_enabled) {
    enter_fault_and_abort();
    continue;
  }
  if (watchdog_result == WatchdogResult::HOLD) {
    servo_j_absolute(last_command, 1);
    publish_holding_status();
    continue;
  }

  mapped_target = mapper.map(sample.position);
  command = limiter.step(mapped_target, 0.008);
  if (max_abs(follower_measured - command) > tracking_error_limit) {
    increment_tracking_error_counter();
  } else {
    reset_tracking_error_counter();
  }
  if (tracking_error_counter >= 13) {  // 约104 ms
    enter_fault_and_abort();
    continue;
  }

  const int code = servo_j_absolute(command, 1);
  if (code != 0) {
    increment_servo_error_counter();
  } else {
    reset_servo_error_counter();
  }
  if (servo_error_counter >= 3) {
    enter_fault_and_abort();
    continue;
  }
  publish_running_status();
}
```

首轮参数：

```text
hold_after_ms: 24
fault_after_ms: 100
tracking_error_limit_rad: 0.15
tracking_error_cycles: 13
servo_error_cycles: 3
control_period_ms: 8
```

故障处理函数只执行一次：

```cpp
void enter_fault(const std::string &reason, int sdk_code) {
  if (state_ == State::FAULT) return;
  state_ = State::FAULT;
  fault_reason_ = reason;
  fault_sdk_code_ = sdk_code;
  api_->motion_abort();
  api_->set_servo_mode(false);
  teleop_enabled_ = false;
}
```

- [ ] **Step 3: 从配置读取映射和限制参数**

节点参数必须包括：

```text
robot_ip: string
leader_topic: string
deadman_topic: string
deadman_timeout_ms: 100
control_period_ms: 8
joint_sign: double[6]
joint_scale: double[6]
joint_lower_rad: double[6]
joint_upper_rad: double[6]
max_deviation_from_arm_rad: double[6]
max_velocity_rad_s: double[6]
max_acceleration_rad_s2: double[6]
hold_after_ms: 24
fault_after_ms: 100
tracking_error_limit_rad: 0.15
tracking_error_cycles: 13
servo_error_cycles: 3
dry_run: true
limits_verified: false
```

`dry_run=true` 时完成映射、限幅和状态发布，但禁止调用 `set_servo_mode(true)` 和 `servo_j_absolute()`。这是第一次双臂联调的默认值。

- [ ] **Step 4: 写入完整从臂节点实现**

`follower_node.cpp`：

```cpp
#include "jaka_dual_teleop/joint_limiter.hpp"
#include "jaka_dual_teleop/joint_mapper.hpp"
#include "jaka_dual_teleop/msg/joint_sample.hpp"
#include "jaka_dual_teleop/msg/teleop_status.hpp"
#include "jaka_dual_teleop/robot_api.hpp"
#include "jaka_dual_teleop/steady_loop.hpp"
#include "jaka_dual_teleop/watchdog.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace jaka_dual_teleop {

namespace {

JointArray as_joint_array(const std::vector<double> &values, const char *name) {
  if (values.size() != 6) {
    throw std::runtime_error(std::string(name) + " must contain 6 values");
  }
  JointArray result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

double max_abs_error(const JointArray &a, const JointArray &b) {
  double result = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    result = std::max(result, std::abs(a[i] - b[i]));
  }
  return result;
}

uint64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class FollowerNode final : public rclcpp::Node {
 public:
  FollowerNode()
      : Node("follower"),
        mapper_(
          as_joint_array(declare_parameter<std::vector<double>>(
            "joint_sign", {1, 1, 1, 1, 1, 1}), "joint_sign"),
          as_joint_array(declare_parameter<std::vector<double>>(
            "joint_scale", {1, 1, 1, 1, 1, 1}), "joint_scale")),
        limiter_(
          as_joint_array(declare_parameter<std::vector<double>>(
            "joint_lower_rad", {-3.1415926, -3.1415926, -3.1415926,
                                -3.1415926, -3.1415926, -3.1415926}),
            "joint_lower_rad"),
          as_joint_array(declare_parameter<std::vector<double>>(
            "joint_upper_rad", {3.1415926, 3.1415926, 3.1415926,
                                3.1415926, 3.1415926, 3.1415926}),
            "joint_upper_rad"),
          as_joint_array(declare_parameter<std::vector<double>>(
            "max_velocity_rad_s", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}),
            "max_velocity_rad_s"),
          as_joint_array(declare_parameter<std::vector<double>>(
            "max_acceleration_rad_s2", {0.3, 0.3, 0.3, 0.3, 0.3, 0.3}),
            "max_acceleration_rad_s2")),
        watchdog_(
          declare_parameter<double>("hold_after_ms", 24.0),
          declare_parameter<double>("fault_after_ms", 100.0)) {
    const std::string ip = declare_parameter<std::string>("robot_ip", "");
    const std::string leader_topic = declare_parameter<std::string>(
      "leader_topic", "/leader/joint_sample");
    const std::string deadman_topic = declare_parameter<std::string>(
      "deadman_topic", "/teleop/deadman");
    deadman_timeout_ms_ = declare_parameter<double>("deadman_timeout_ms", 100.0);
    control_period_ms_ = declare_parameter<int>("control_period_ms", 8);
    tracking_error_limit_rad_ = declare_parameter<double>(
      "tracking_error_limit_rad", 0.15);
    enable_freshness_ms_ = get_parameter("hold_after_ms").as_double();
    tracking_error_cycles_limit_ = declare_parameter<int>(
      "tracking_error_cycles", 13);
    servo_error_cycles_limit_ = declare_parameter<int>(
      "servo_error_cycles", 3);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    limits_verified_ = declare_parameter<bool>("limits_verified", false);
    max_deviation_from_arm_ = as_joint_array(
      declare_parameter<std::vector<double>>(
        "max_deviation_from_arm_rad", {0.035, 0.035, 0.035, 0.035, 0.035, 0.035}),
      "max_deviation_from_arm_rad");

    if (ip.empty() || control_period_ms_ != 8 ||
        tracking_error_cycles_limit_ <= 0 || servo_error_cycles_limit_ <= 0) {
      throw std::runtime_error("invalid follower configuration");
    }

    api_ = make_jaka_robot_api();
    const int code = api_->login(ip);
    if (code != 0) {
      throw std::runtime_error("follower login failed: " + std::to_string(code));
    }

    status_pub_ = create_publisher<msg::TeleopStatus>(
      "status", rclcpp::SensorDataQoS().keep_last(10));
    leader_sub_ = create_subscription<msg::JointSample>(
      leader_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](msg::JointSample::ConstSharedPtr sample) {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        latest_sample_ = *sample;
        have_sample_ = true;
      });
    deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
      deadman_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        deadman_active_.store(message->data);
        deadman_stamp_ns_.store(monotonic_ns());
      });
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "set_enabled",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        pending_enable_request_.store(request->data ? 1 : -1);
        response->success = true;
        response->message = request->data ?
          "enable request queued; inspect /follower/status" :
          "disable/reset request queued; inspect /follower/status";
      });

    state_ = msg::TeleopStatus::STANDBY;
    worker_ = std::thread([this] { run(); });
  }

  ~FollowerNode() override {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    if (api_) {
      api_->motion_abort();
      api_->set_servo_mode(false);
      api_->logout();
    }
  }

 private:
  msg::JointSample snapshot_sample() {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    return latest_sample_;
  }

  double sample_age_ms(const msg::JointSample &sample) const {
    if (!have_sample_.load() || sample.monotonic_ns == 0) {
      return std::numeric_limits<double>::infinity();
    }
    const uint64_t now_ns = monotonic_ns();
    if (sample.monotonic_ns > now_ns) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - sample.monotonic_ns) / 1e6;
  }

  JointArray bounded_to_arm_window(const JointArray &desired) const {
    JointArray bounded{};
    for (std::size_t i = 0; i < 6; ++i) {
      bounded[i] = std::clamp(
        desired[i],
        follower_zero_[i] - max_deviation_from_arm_[i],
        follower_zero_[i] + max_deviation_from_arm_[i]);
    }
    return bounded;
  }

  bool deadman_is_fresh() const {
    const uint64_t stamp = deadman_stamp_ns_.load();
    if (!deadman_active_.load() || stamp == 0) return false;
    const uint64_t now_ns = monotonic_ns();
    if (stamp > now_ns) return false;
    return static_cast<double>(now_ns - stamp) / 1e6 < deadman_timeout_ms_;
  }

  void disable_and_reset() {
    if (!dry_run_) {
      api_->motion_abort();
      api_->set_servo_mode(false);
    }
    teleop_active_ = false;
    fault_reason_.clear();
    fault_sdk_code_ = 0;
    tracking_error_cycles_ = 0;
    servo_error_cycles_ = 0;
    watchdog_.reset();
    state_ = msg::TeleopStatus::STANDBY;
  }

  void enter_fault(const std::string &reason, int sdk_code) {
    if (state_ == msg::TeleopStatus::FAULT) return;
    fault_reason_ = reason;
    fault_sdk_code_ = sdk_code;
    state_ = msg::TeleopStatus::FAULT;
    teleop_active_ = false;
    if (!dry_run_) {
      api_->motion_abort();
      api_->set_servo_mode(false);
    }
  }

  void process_enable_request(
      const msg::JointSample &sample,
      const JointArray &follower_measured,
      const RobotHealth &health) {
    const int request = pending_enable_request_.exchange(0);
    if (request == 0) return;
    if (request < 0) {
      disable_and_reset();
      return;
    }
    if (state_ == msg::TeleopStatus::FAULT) {
      return;
    }
    const double age_ms = sample_age_ms(sample);
    if (!sample.valid || !sample.drag_enabled || age_ms >= enable_freshness_ms_ ||
        !deadman_is_fresh() ||
        !health.powered_on || !health.enabled || health.collision) {
      enter_fault("enable precondition failed", 0);
      return;
    }
    if (!dry_run_ && !limits_verified_) {
      enter_fault("joint limits are not verified", 0);
      return;
    }

    mapper_.arm(sample.position, follower_measured);
    follower_zero_ = follower_measured;
    limiter_.reset(follower_measured);
    sent_command_ = follower_measured;
    watchdog_.reset();
    tracking_error_cycles_ = 0;
    servo_error_cycles_ = 0;
    state_ = msg::TeleopStatus::ARMED;
    if (!dry_run_) {
      const int code = api_->set_servo_mode(true);
      if (code != 0) {
        enter_fault("servo mode enable failed", code);
        return;
      }
    }
    teleop_active_ = true;
    state_ = msg::TeleopStatus::RUNNING;
  }

  void publish_status(
      const msg::JointSample &sample,
      const JointArray &measured,
      const JointArray &mapped,
      double age_ms,
      const RobotHealth &health,
      bool command_sent) {
    msg::TeleopStatus status;
    status.ros_stamp = now().to_msg();
    status.monotonic_ns = monotonic_ns();
    status.state = state_;
    status.reason = fault_reason_;
    status.leader_sequence = sample.sequence;
    status.leader_age_ms = age_ms;
    status.leader_position = sample.position;
    status.follower_measured = measured;
    status.mapped_target = mapped;
    status.sent_command = sent_command_;
    status.command_sent = command_sent;
    for (std::size_t i = 0; i < 6; ++i) {
      status.tracking_error[i] = measured[i] - sent_command_[i];
    }
    status.follower_powered = health.powered_on;
    status.follower_enabled = health.enabled;
    status.follower_collision = health.collision;
    status.deadman_active = deadman_is_fresh();
    status.sdk_code = fault_sdk_code_;
    status_pub_->publish(status);
  }

  void run() {
    SteadyLoop loop(std::chrono::milliseconds(control_period_ms_));
    RobotHealth health{};
    JointArray measured{};
    JointArray mapped{};
    int read_errors = 0;
    int cycle = 0;

    while (rclcpp::ok() && !stop_.load()) {
      loop.wait_next();
      ++cycle;
      const msg::JointSample sample = snapshot_sample();
      const double age_ms = sample_age_ms(sample);

      const int read_code = api_->get_joint_position(measured);
      read_errors = read_code == 0 ? 0 : read_errors + 1;
      if (read_errors >= 3) {
        enter_fault("follower joint read failed", read_code);
      }

      if (cycle == 1 || cycle % 10 == 0) {
        const int health_code = api_->get_health(health);
        if (health_code != 0) {
          enter_fault("follower health read failed", health_code);
        } else if (health.collision ||
                   (teleop_active_ && (!health.powered_on || !health.enabled))) {
          enter_fault("follower is not healthy", 0);
        }
      }

      process_enable_request(sample, measured, health);
      bool command_sent = false;

      if (teleop_active_ && state_ != msg::TeleopStatus::FAULT) {
        if (!deadman_is_fresh()) {
          enter_fault("deadman released or timed out", 0);
        }
        if (state_ == msg::TeleopStatus::FAULT) {
          publish_status(sample, measured, mapped, age_ms, health, false);
          continue;
        }
        const WatchdogResult result = watchdog_.evaluate(age_ms);
        if (!sample.valid || !sample.drag_enabled ||
            result == WatchdogResult::FAULT) {
          enter_fault("leader sample invalid or timed out", sample.sdk_code);
        } else if (result == WatchdogResult::HOLD) {
          state_ = msg::TeleopStatus::HOLDING;
          if (!dry_run_) {
            const int code = api_->servo_j_absolute(sent_command_, 1);
            command_sent = code == 0;
            if (code != 0) enter_fault("hold command failed", code);
          }
        } else {
          state_ = msg::TeleopStatus::RUNNING;
          mapped = bounded_to_arm_window(mapper_.map(sample.position));
          sent_command_ = limiter_.step(mapped, control_period_ms_ / 1000.0);
          if (max_abs_error(measured, sent_command_) > tracking_error_limit_rad_) {
            ++tracking_error_cycles_;
          } else {
            tracking_error_cycles_ = 0;
          }
          if (tracking_error_cycles_ >= tracking_error_cycles_limit_) {
            enter_fault("tracking error exceeded", 0);
          } else if (!dry_run_) {
            const int code = api_->servo_j_absolute(sent_command_, 1);
            command_sent = code == 0;
            servo_error_cycles_ = code == 0 ? 0 : servo_error_cycles_ + 1;
            if (servo_error_cycles_ >= servo_error_cycles_limit_) {
              enter_fault("servo command failed repeatedly", code);
            }
          }
        }
      }

      publish_status(sample, measured, mapped, age_ms, health, command_sent);
    }
  }

  std::unique_ptr<RobotApi> api_;
  JointMapper mapper_;
  JointLimiter limiter_;
  Watchdog watchdog_;

  rclcpp::Subscription<msg::JointSample>::SharedPtr leader_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Publisher<msg::TeleopStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

  std::mutex sample_mutex_;
  msg::JointSample latest_sample_{};
  std::atomic<bool> have_sample_{false};
  std::atomic<int> pending_enable_request_{0};
  std::atomic<bool> deadman_active_{false};
  std::atomic<uint64_t> deadman_stamp_ns_{0};
  std::atomic<bool> stop_{false};
  std::thread worker_;

  JointArray follower_zero_{};
  JointArray max_deviation_from_arm_{};
  JointArray sent_command_{};
  int control_period_ms_{8};
  int tracking_error_cycles_{0};
  int tracking_error_cycles_limit_{13};
  int servo_error_cycles_{0};
  int servo_error_cycles_limit_{3};
  double tracking_error_limit_rad_{0.15};
  double enable_freshness_ms_{24.0};
  double deadman_timeout_ms_{100.0};
  bool dry_run_{true};
  bool limits_verified_{false};
  bool teleop_active_{false};
  uint8_t state_{msg::TeleopStatus::DISCONNECTED};
  std::string fault_reason_;
  int fault_sdk_code_{0};
};

}  // namespace jaka_dual_teleop

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jaka_dual_teleop::FollowerNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("follower"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: 发布用于数据采集的状态**

每个控制周期发布相对话题 `status`，放入 `/follower` 命名空间后解析为 `/follower/status`，至少包含：

- 主臂原始关节角；
- 从臂实测关节角；
- 映射目标；
- 实际发送命令；
- 跟踪误差；
- 主臂数据年龄；
- 状态和故障原因。

数据集动作标签只使用 `command_sent=true` 的 `sent_command`，不使用未经限幅的 `mapped_target`，也不把干跑阶段“本来会发送”的命令混入训练集。

- [ ] **Step 6: 添加构建规则**

在 `CMakeLists.txt` 中加入：

```cmake
add_executable(follower_node src/follower_node.cpp)
target_include_directories(follower_node PRIVATE include)
target_link_libraries(follower_node PRIVATE jaka_robot_api yaml-cpp ${cpp_typesupport_target})
ament_target_dependencies(follower_node rclcpp std_msgs std_srvs)
install(TARGETS follower_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 7: 构建并以干跑模式验证**

Run:

```bash
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
source teleop_ws/install/setup.bash
ros2 run jaka_dual_teleop follower_node --ros-args \
  -p robot_ip:=10.5.5.102 \
  -p leader_topic:=/leader/joint_sample \
  -p dry_run:=true
```

Expected: 从臂不运动；`/follower/status` 接近125 Hz，移动主臂时 `mapped_target` 变化，`sent_command` 按限速缓慢变化。

- [ ] **Step 8: 测试看门狗**

在 `dry_run=true`、遥操作已启用时停止主臂节点。

Expected:

- 约24 ms后状态变为 `HOLDING`；
- 约100 ms后状态变为 `FAULT`；
- 重启主臂节点后从臂不会自动恢复，必须先调用 `set_enabled=false` 再重新启用。

- [ ] **Step 9: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/src/follower_node.cpp \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add safe follower servo controller"
```

## Task 9: 添加首轮保守配置和启动文件

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/config/mini2_same_mount.yaml`
- Create: `teleop_ws/src/jaka_dual_teleop/launch/dual_teleop.launch.py`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 添加保守配置**

`mini2_same_mount.yaml`：

```yaml
/leader/leader:
  ros__parameters:
    robot_ip: "10.5.5.101"
    publish_rate_hz: 125.0
    manage_drag_mode: true

/follower/follower:
  ros__parameters:
    robot_ip: "10.5.5.102"
    leader_topic: "/leader/joint_sample"
    deadman_topic: "/teleop/deadman"
    deadman_timeout_ms: 100.0
    control_period_ms: 8
    joint_sign: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    joint_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    joint_lower_rad: [-3.1415926, -3.1415926, -3.1415926, -3.1415926, -3.1415926, -3.1415926]
    joint_upper_rad: [ 3.1415926,  3.1415926,  3.1415926,  3.1415926,  3.1415926,  3.1415926]
    max_deviation_from_arm_rad: [0.035, 0.035, 0.035, 0.035, 0.035, 0.035]
    limits_verified: false
    max_velocity_rad_s: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
    max_acceleration_rad_s2: [0.30, 0.30, 0.30, 0.30, 0.30, 0.30]
    hold_after_ms: 24.0
    fault_after_ms: 100.0
    tracking_error_limit_rad: 0.15
    dry_run: true
```

这里的 `[-π, π]` 只是干跑配置的解析默认值，不代表 Mini 2 官方机械极限。`limits_verified=false` 时节点必须拒绝进入真实Servo模式；同时，首轮实机测试还被 `max_deviation_from_arm_rad=0.035` 限制在捕获零位的±2°。启用真实运动前，必须从对应 Mini 2 控制器/厂家资料读取每轴限制，写入上下限并把 `limits_verified` 改为 `true`。

- [ ] **Step 2: 添加启动文件**

`dual_teleop.launch.py`：

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("jaka_dual_teleop"))
        / "config"
        / "mini2_same_mount.yaml"
    )
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="jaka_dual_teleop",
            executable="leader_node",
            namespace="leader",
            name="leader",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="jaka_dual_teleop",
            executable="follower_node",
            namespace="follower",
            name="follower",
            parameters=[config],
            output="screen",
        ),
    ])
```

- [ ] **Step 3: 安装配置和启动文件**

在 `CMakeLists.txt` 加入：

```cmake
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 4: 启动干跑系统**

Run:

```bash
colcon build --base-paths teleop_ws/src --packages-select jaka_dual_teleop --symlink-install
source teleop_ws/install/setup.bash
ros2 launch jaka_dual_teleop dual_teleop.launch.py
```

Expected: 两台SDK均登录成功；从臂不运动；两个高频话题存在：

```text
/leader/joint_sample
/follower/status
```

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/config \
        teleop_ws/src/jaka_dual_teleop/launch \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add conservative dual teleop launch"
```

## Task 10: 添加网络预检、deadman、启动和记录脚本

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/scripts/check_teleop_network.sh`
- Create: `teleop_ws/src/jaka_dual_teleop/scripts/deadman_evdev.py`
- Create: `teleop_ws/src/jaka_dual_teleop/scripts/start_teleop.sh`
- Create: `teleop_ws/src/jaka_dual_teleop/scripts/record_teleop.sh`
- Modify: `teleop_ws/src/jaka_dual_teleop/CMakeLists.txt`

- [ ] **Step 1: 添加网络预检脚本**

`check_teleop_network.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

LEADER_IP="${1:?usage: $0 <leader_ip> <follower_ip>}"
FOLLOWER_IP="${2:?usage: $0 <leader_ip> <follower_ip>}"

for ip in "$LEADER_IP" "$FOLLOWER_IP"; do
  echo "checking $ip"
  ping -c 20 -W 1 "$ip" >/tmp/jaka_ping_"${ip//./_}".txt
  loss=$(awk -F', ' '/packet loss/ {print $3}' /tmp/jaka_ping_"${ip//./_}".txt)
  echo "$ip $loss"
  if ! grep -q '0% packet loss' /tmp/jaka_ping_"${ip//./_}".txt; then
    echo "network check failed for $ip" >&2
    exit 1
  fi
done

echo "network check passed"
```

- [ ] **Step 2: 添加USB脚踏deadman节点**

`deadman_evdev.py`：

```python
#!/usr/bin/env python3
import threading

from evdev import InputDevice, categorize, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DeadmanEvdev(Node):
    def __init__(self) -> None:
        super().__init__("deadman_evdev")
        device_path = self.declare_parameter("device_path", "").value
        self.key_code = int(self.declare_parameter("key_code", ecodes.KEY_SPACE).value)
        if not device_path:
            raise RuntimeError("device_path must point to /dev/input/by-id/...-event-kbd")
        self.publisher = self.create_publisher(Bool, "/teleop/deadman", 1)
        self.pressed = False
        self.failed = False
        self.device = InputDevice(device_path)
        self.thread = threading.Thread(target=self.read_events, daemon=True)
        self.thread.start()
        self.timer = self.create_timer(0.05, self.publish_state)

    def read_events(self) -> None:
        try:
            for event in self.device.read_loop():
                if event.type != ecodes.EV_KEY:
                    continue
                key = categorize(event)
                if key.scancode == self.key_code:
                    self.pressed = key.keystate in (key.key_down, key.key_hold)
        except Exception as error:  # Device loss must fail closed.
            self.failed = True
            self.pressed = False
            self.get_logger().error(f"deadman device failed: {error}")

    def publish_state(self) -> None:
        message = Bool()
        message.data = self.pressed and not self.failed
        self.publisher.publish(message)

    def close(self) -> None:
        try:
            self.device.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = DeadmanEvdev()
    try:
        rclpy.spin(node)
    finally:
        false_message = Bool()
        false_message.data = False
        node.publisher.publish(false_message)
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

查找稳定设备路径：

```bash
ls -l /dev/input/by-id/
python3 -c 'from evdev import InputDevice; import glob; [print(p, InputDevice(p).name) for p in glob.glob("/dev/input/by-id/*event*")]'
```

真实运动必须运行该节点或功能等价的物理按住式开关发布器。`/teleop/deadman` 是操作许可，不是安全认证急停。

- [ ] **Step 3: 添加启动脚本**

`start_teleop.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT="${JAKA_MINI2_ROOT:-$HOME/codex/codex-jaka_mini_2}"
source /opt/ros/humble/setup.bash
source "$ROOT/teleop_ws/install/setup.bash"

exec ros2 launch jaka_dual_teleop dual_teleop.launch.py "$@"
```

- [ ] **Step 4: 添加基础MCAP记录脚本**

`record_teleop.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

NAME="${1:-teleop_$(date +%Y%m%d_%H%M%S)}"
ROOT="${JAKA_MINI2_ROOT:-$HOME/codex/codex-jaka_mini_2}"
source /opt/ros/humble/setup.bash
source "$ROOT/teleop_ws/install/setup.bash"
mkdir -p "$ROOT/bags"

exec ros2 bag record -s mcap -o "$ROOT/bags/$NAME" \
  /teleop/deadman \
  /leader/joint_sample \
  /follower/status
```

后续相机和灵巧手接入后，只扩展该脚本的话题列表，不改变基础遥操作消息定义。

- [ ] **Step 5: 安装脚本**

在 `CMakeLists.txt` 加入：

```cmake
install(PROGRAMS
  scripts/check_teleop_network.sh
  scripts/deadman_evdev.py
  scripts/start_teleop.sh
  scripts/record_teleop.sh
  DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 6: 运行网络预检**

Run:

```bash
bash teleop_ws/src/jaka_dual_teleop/scripts/check_teleop_network.sh \
  10.5.5.101 10.5.5.102
```

Expected: 两台机器人均显示 `0% packet loss`，最后输出 `network check passed`。

- [ ] **Step 7: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/scripts \
        teleop_ws/src/jaka_dual_teleop/CMakeLists.txt
git commit -m "feat: add teleop preflight and recording scripts"
```

## Task 11: 完成无运动干跑验收

**Files:**

- Create: `teleop_ws/src/jaka_dual_teleop/README.md`

- [ ] **Step 1: 在机器人上电、使能但从臂不进入Servo模式时启动**

保持配置 `dry_run: true`，执行：

```bash
bash teleop_ws/src/jaka_dual_teleop/scripts/check_teleop_network.sh \
  10.5.5.101 10.5.5.102
ros2 launch jaka_dual_teleop dual_teleop.launch.py
```

- [ ] **Step 2: 开启主臂拖拽**

```bash
ros2 service call /leader/set_drag std_srvs/srv/SetBool '{data: true}'
```

- [ ] **Step 3: 启用从臂干跑映射**

先启动脚踏节点并保持按下：

```bash
ls -l /dev/input/by-id/
read -r -p 'Paste the foot-pedal /dev/input/by-id/... event path: ' PEDAL_DEVICE
test -e "$PEDAL_DEVICE"
ros2 run jaka_dual_teleop deadman_evdev.py --ros-args \
  -p device_path:="$PEDAL_DEVICE"
```

```bash
ros2 service call /follower/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Expected: 从臂不运动；拖动主臂时 `/follower/status.mapped_target` 连续变化。

- [ ] **Step 4: 验证方向映射**

每次只移动主臂一个关节约2°，记录：

```text
J1主臂正向 -> 从臂目标方向
J2主臂正向 -> 从臂目标方向
...
J6主臂正向 -> 从臂目标方向
```

若任一方向错误，只修改对应 `joint_sign`。不要用同时移动多个关节的方法猜测符号。

- [ ] **Step 5: 验证超时故障锁存**

停止 leader 节点，检查 `/follower/status`：

```bash
ros2 topic echo /follower/status
```

Expected: `HOLDING` 后进入 `FAULT`，重启 leader 后不自动恢复。

- [ ] **Step 6: 验证记录结果**

```bash
ros2 bag record -s mcap -o bags/teleop_dry_run \
  /teleop/deadman /leader/joint_sample /follower/status
ros2 bag info bags/teleop_dry_run
```

Expected: 两个话题均存在且消息数量接近录制时长×125。

- [ ] **Step 7: 在 README 记录验收表**

README 必须包含以下表格并填写实测值：

```markdown
| Check | Required | Measured | Pass |
|---|---:|---:|---|
| Leader publish rate | >= 120 Hz | | |
| Follower control/status rate | >= 120 Hz | | |
| 1000 ping packet loss | 0% | | |
| Leader stale to HOLD | 24-40 ms | | |
| Leader stale to FAULT | 100-130 ms | | |
| Dry-run unexpected follower motion | 0 | | |
| Six joint mapping signs verified | 6/6 | | |
```

- [ ] **Step 8: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/README.md
git commit -m "docs: record teleop dry-run acceptance"
```

## Task 12: 逐轴低速实机跟随

**Files:**

- Modify: `teleop_ws/src/jaka_dual_teleop/config/mini2_same_mount.yaml`
- Modify: `teleop_ws/src/jaka_dual_teleop/README.md`

- [ ] **Step 1: 建立实机测试前提**

必须全部满足：

- 两臂工作空间物理分离，互相无法碰撞。
- 从臂不安装水壶；首次最好不安装灵巧手和相机负载。
- 两台机器人各自急停可达。
- 一名操作者拖动主臂，另一名观察从臂并持急停。
- 两台机器人额定速度比例设置为5%。
- USB脚踏deadman节点工作正常，松开脚踏可使从臂进入FAULT。
- 两臂都位于远离奇异位姿和关节限位的中间姿态。
- 已验证负载、安装方式和TCP配置。

- [ ] **Step 2: 仅开放J1小范围测试**

先把经过厂家资料和实际控制器确认的六轴关节上下限写入配置，将 `limits_verified: true`、`dry_run: false`，并只开放J1：

```yaml
joint_scale: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
max_velocity_rad_s: [0.05, 0.01, 0.01, 0.01, 0.01, 0.01]
max_acceleration_rad_s2: [0.15, 0.03, 0.03, 0.03, 0.03, 0.03]
```

启用顺序：

```bash
ros2 service call /leader/set_drag std_srvs/srv/SetBool '{data: true}'
ros2 service call /follower/set_enabled std_srvs/srv/SetBool '{data: true}'
```

主臂J1只移动±2°。Expected: 从臂J1同方向、缓慢跟随，其余关节基本保持。

- [ ] **Step 3: 重复J2至J6**

每轮只把一个关节 `joint_scale` 设为1.0。任一关节出现方向错误、阶跃、抖动或跟踪误差故障，立即禁用并返回干跑检查。

- [ ] **Step 4: 测试松开软件使能**

运动过程中调用：

```bash
ros2 service call /follower/set_enabled std_srvs/srv/SetBool '{data: false}'
```

Expected: 从臂平稳停止，不继续追赶主臂后续位置。

- [ ] **Step 5: 测试网络/主臂节点故障**

只在从臂低速、小幅、远离障碍物时终止 leader 节点。

Expected: 从臂短暂保持后退出Servo模式进入FAULT，不发生持续运动。

- [ ] **Step 6: 恢复六关节低速配置**

逐轴全部通过后恢复：

```yaml
joint_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
max_velocity_rad_s: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
max_acceleration_rad_s2: [0.30, 0.30, 0.30, 0.30, 0.30, 0.30]
```

进行10分钟空载跟随，记录MCAP。

- [ ] **Step 7: 更新验收结果**

README 添加：

```markdown
| Hardware check | Required | Measured | Pass |
|---|---:|---:|---|
| Unexpected motion on enable | 0 | | |
| Single-joint direction tests | 6/6 | | |
| Network-loss stop tests | 3/3 | | |
| Software-disable stop tests | 3/3 | | |
| 10-minute SDK servo errors | 0 | | |
| 10-minute collision/limit events | 0 | | |
| Max tracking error | < 0.15 rad | | |
```

- [ ] **Step 8: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/config/mini2_same_mount.yaml \
        teleop_ws/src/jaka_dual_teleop/README.md
git commit -m "test: validate low-speed dual-arm teleoperation"
```

## Task 13: 基础遥操作完成验收和后续接口冻结

**Files:**

- Modify: `teleop_ws/src/jaka_dual_teleop/README.md`

- [ ] **Step 1: 执行全部软件测试**

Run:

```bash
colcon test --base-paths teleop_ws/src --packages-select jaka_dual_teleop \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: mapper、limiter、watchdog全部通过，无失败测试。

- [ ] **Step 2: 执行30分钟空载遥操作稳定性测试**

记录：

```bash
ros2 bag record -s mcap -o bags/teleop_acceptance_30min \
  /teleop/deadman /leader/joint_sample /follower/status
```

验收标准：

- 两个话题平均频率均不低于120 Hz。
- SDK发送错误为0。
- 意外FAULT为0。
- 碰撞和软限位触发为0。
- 最大跟踪误差小于0.15 rad。
- 从臂无明显持续振荡。

- [ ] **Step 3: 冻结后续数据采集接口**

以下字段确定后不再随意更名：

```text
/leader/joint_sample.position
/follower/status.follower_measured
/follower/status.mapped_target
/follower/status.sent_command
/follower/status.state
/follower/status.reason
/follower/status.monotonic_ns
```

VLA动作标签统一使用 `/follower/status.sent_command`。观测状态使用同一消息中的 `follower_measured`，避免跨话题时间差。

- [ ] **Step 4: 给后续子项目列出明确入口**

README 添加：

```text
Gemini 335L -> /camera_global/color/image_raw
Gemini 305  -> /camera_wrist/color/image_raw
灵巧手状态  -> /hand/state
灵巧手命令  -> /hand/command
Episode控制 -> /dataset/episode_event
遥操作动作  -> /follower/status.sent_command
```

- [ ] **Step 5: Commit**

```bash
git add teleop_ws/src/jaka_dual_teleop/README.md
git commit -m "docs: freeze teleop interfaces for dataset collection"
```

## 2. 实施完成后的运行流程

每次运行按以下顺序：

```text
1. 检查两台控制柜、急停、负载、TCP和工作空间
2. 运行网络预检
3. 启动 leader/follower 节点，初始为 STANDBY
4. 确认两个125 Hz话题健康
5. 开启主臂拖拽
6. 确认从臂周围无人后启用 follower
7. 操作者按计划进行遥操作
8. 先禁用 follower
9. 再关闭主臂拖拽
10. 停止MCAP录制并检查 bag info
```

禁止的操作顺序：

- 不允许先启用从臂、后确认主臂数据。
- 不允许在主臂未进入拖拽时启用从臂。
- 不允许发生FAULT后自动恢复。
- 不允许绕开限幅器直接调用 `servo_j`。
- 不允许用无线网络运行控制闭环。

## 3. 第一阶段最终交付物

- 可构建的 `jaka_dual_teleop` ROS 2 包。
- 两个独立JAKA SDK进程。
- 125 Hz主臂关节采样。
- 125 Hz从臂绝对关节ServoJ控制。
- 零位、方向和比例映射。
- 速度、加速度、关节限位和跟踪误差保护。
- 主臂数据超时HOLD/FAULT机制。
- 软件启用/禁用服务和故障锁存。
- 干跑模式。
- MCAP可记录的原始目标、实际动作和状态。
- 单元测试、只读SDK测试、逐轴实机验收和30分钟稳定性报告。

通过这些验收后，再进入独立的“双相机与灵巧手数据采集”实施计划。
