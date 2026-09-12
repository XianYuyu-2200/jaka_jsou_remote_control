# Gemini 215 + JAKA Mini 2 NUC workspace

## Open a terminal

```bash
source ~/codex/codex-jaka_mini_2/scripts/env.bash
```

## Check the NUC and connected devices

```bash
cd ~/codex/codex-jaka_mini_2
./scripts/check_environment.sh
```

Connect Gemini 215 directly to a USB 3 port. Confirm `lsusb -t` shows the camera under a `5000M` or `10000M` bus, not a `480M` bus.

## Start Gemini 215

```bash
cd ~/codex/codex-jaka_mini_2
./scripts/start_camera.sh
```

For a lighter VLA RGB-D recording without the point-cloud topic:

```bash
./scripts/start_camera.sh enable_point_cloud:=false
```

In a second terminal:

```bash
source ~/codex/codex-jaka_mini_2/scripts/env.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

## Start JAKA

Replace the IP with the actual robot controller IP:

```bash
cd ~/codex/codex-jaka_mini_2
./scripts/start_jaka.sh <ROBOT_IP>
```

Then inspect:

```bash
ros2 topic hz /jaka_driver/joint_position
ros2 topic hz /jaka_driver/tool_position
ros2 topic echo /jaka_driver/robot_states
```

## Record a test

First inspect the topic names, then adjust `scripts/record_test.sh` if the camera namespace differs:

```bash
./scripts/record_test.sh static_01
```

The script records MCAP under `bags/`. Keep a separate log of commanded actions because ROS bag recording of robot state does not automatically preserve service requests used to command motion.

## Current build scope

- Orbbec SDK ROS 2 wrapper: `v2-main`, built successfully.
- JAKA ROS 2: official `main`, `jaka_msgs` and `jaka_driver` built successfully.
- Full JAKA Gazebo/MoveIt dependency installation was intentionally not completed; it is not needed for camera data collection.
- The public JAKA repository currently exposes MiniCobo MoveIt files, not a confirmed Mini 2 model. Obtain the Mini 2 URDF/MoveIt package from JAKA before planning with MoveIt 2.

## Dry-run teleoperation before robot arrival

The Windows workspace can generate the same 125 Hz logical control records
without connecting to a robot. Copy `teleop_core.py` and
`nuc_setup/teleop_dryrun.py` to the NUC workspace, then run:

```bash
cd ~/codex/codex_jaka_mini_2
python3 nuc_setup/teleop_dryrun.py \
  --steps 1250 \
  --output bags/nuc_teleop_dryrun.jsonl \
  --realtime
```

This does not call JAKA SDK or move a robot. It validates the mapping,
acceleration/velocity limits, deadman fault behavior, and the record schema.
To test a released deadman at step 300:

```bash
python3 nuc_setup/teleop_dryrun.py \
  --steps 600 \
  --deadman-release-step 300 \
  --output bags/nuc_deadman_fault.jsonl
```

The real NUC ROS 2/JAKA SDK node is not enabled by this script; it must be
added after the JAKA SDK version and controller behavior are verified on the
delivered robots.
