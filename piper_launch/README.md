# piper_launch

`piper_launch` 是 Piper 电梯按键任务的总启动功能包。它不实现新的控制
算法，而是将以下已有功能包按照统一参数组合起来：

```text
Gemini 335L RGB-D 相机
        ↓
YOLO 按键检测 + 手眼 TF
        ↓
MoveIt 粗定位 + PBVS 对准/按压
        ↓
Piper 实机驱动
```

主入口 `all.launch.py` 会启动：

- `orbbec_camera/gemini_330_series.launch.py`
- `piper/start_single_piper.launch.py`
- `piper_with_gripper_moveit/real_feedback_demo.launch.py`
- `piper_pbvs_control/elevator_press.launch.py`

所有物理运动开关默认关闭。默认启动不会自动使能机械臂，也不会发布
PBVS 真机运动或按压命令。

## 1. 前置条件

- ROS 2 Humble 工作区已经完成编译。
- Gemini 335L 通过 USB 3.x 连接。
- `can0` 已设置为 `1 Mbps` 并处于 `UP` 状态。
- Conda 环境 `yolo11` 能导入 `ultralytics`、`rclpy` 和
  `cv_bridge`。
- YOLO Detect 模型存在，并通过 `PIPER_MODEL_PATH` 环境变量或
  `model_path` 启动参数指定，不要将模型文件提交到 Git。
- `piper_tf/config/handeye.yaml` 与当前相机安装位置一致。
- 驱动和 PBVS 使用相同的标定 TCP 偏置，默认 `z=0.1468 m`。

主入口默认使用 `640×360@30 Y16` 原生深度和 `High Density` 设备预设。
相比相机自动选择的 `848×480`，该配置更适合约 `0.3～0.5 m` 的眼在手上
按钮任务。软件深度对齐仍输出彩色图尺寸，YOLO 会继续使用对应的
`camera_color_optical_frame` 和实时 CameraInfo。

检查硬件：

```bash
ip -details link show can0
bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/list_ob_devices.sh
```

检查模型：

```bash
conda run -n yolo11 python -c \
  "from ultralytics import YOLO; print(YOLO('best.pt').names)"
```

当前 `best.pt` 的类别为 `key_0`～`key_9`、`key_Back`、`key_B` 和
`key_ok`。Action 目标必须使用这些精确名称。

## 2. 编译

```bash
cd /path/to/elevate_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_launch
source install/setup.bash
```

## 3. 安全 dry-run

确认机械臂工作空间无人、急停可用，然后执行：

```bash
ros2 launch piper_launch all.launch.py
```

等价的显式写法是：

```bash
ros2 launch piper_launch all.launch.py \
  model_path:=/path/to/best.pt \
  auto_enable:=false \
  enable_motion:=false \
  enable_press:=false \
  orientation_mode:=preserve_current_roll
```

默认会打开 RViz，但 Piper 保持未自动使能。开始测试前确认：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /arm_status
ros2 topic echo --once /tcp_pose
ros2 action list
```

发送按键规划请求，例如按数字 3：

```bash
ros2 action send_goal /press_button \
  piper_msgs/action/PressButton \
  "{target_name: 'key_3'}" --feedback
```

dry-run 应经过：

```text
WAIT_TARGET → COARSE_APPROACH → DONE → IDLE
```

此阶段只调用 MoveIt 规划，不应发布 `/pos_cmd`。

## 4. 分阶段启动

所有组件都可以单独关闭，便于排查。

只启动相机：

```bash
ros2 launch piper_launch all.launch.py \
  start_piper:=false \
  start_moveit:=false \
  start_pbvs:=false
```

不重复启动已有相机：

```bash
ros2 launch piper_launch all.launch.py start_camera:=false
```

不启动 RViz：

```bash
ros2 launch piper_launch all.launch.py use_rviz:=false
```

不重复启动已有手眼 TF：

```bash
ros2 launch piper_launch all.launch.py \
  enable_handeye_tf:=false
```

禁止同时运行两套 `link6 → camera_link` 手眼 TF 发布器。

## 5. 当前可用模式

默认仍是安全的 MoveIt plan-only：

```bash
ros2 launch piper_launch all.launch.py \
  auto_enable:=false \
  enable_motion:=false \
  enable_press:=false
```

`real_feedback_demo.launch.py` 会启动
`piper_trajectory_controller`，提供真实
`/arm_controller/follow_joint_trajectory` 和
`/gripper_controller/follow_joint_trajectory`。控制器从真机
`/joint_states` 闭环，并把插值后的完整七关节命令发布到
`/joint_ctrl_single`。默认不允许 MoveIt 执行；完成单关节、小范围、低速验收
后才设置 `enable_motion:=true`。

默认姿态策略为 `orientation_mode:=preserve_current_roll`。控制器在每次
Action 获取稳定目标后读取最新 `/tcp_pose`，保持当时的末端滚转，只让
TCP `+Z` 轴以最短旋转对齐按钮按压方向。该姿态贯穿 MoveIt 粗定位、
PBVS、按压和回撤，可避免视觉完整姿态使末端额外旋转 90 度。设置
`orientation_mode:=world_up` 可恢复旧行为。

`real_feedback_demo.launch.py` 不启动
`mock_components/GenericSystem`、`ros2_control_node` 或模拟
`joint_state_broadcaster`。`/joint_states` 的唯一预期发布者是
`piper_ctrl_single_node`，MoveIt、`robot_state_publisher` 和 RViz 都使用
这一路真机反馈。

## 6. 完整按压参数

真实 Piper `FollowJointTrajectory` 控制器经过单独验收后，完整按压流程才可
使用。任务参数包括：

- 粗定位距离：`80 mm`
- 预按压距离：`5 mm`
- 按压过行程：`3 mm`
- 最大按压轴向行程：`12 mm`
- 最大横向偏移：`4 mm`

完整执行时，粗定位后的状态为：

```text
COARSE_APPROACH → REACQUIRE_TARGET → PBVS_ALIGN
```

视觉节点会在机械臂移动前用 5 个稳定平面锁定电梯面板。控制器清除旧的
按钮位姿后，在新视角用 RGB 按钮中心射线与锁定面板求交，重新取得 5 个
新鲜、稳定的样本后才进入 PBVS。回撤使用粗定位结束时实测的
`/tcp_pose`；PBVS 同时保持该实测姿态，只细化位置。
终端会为每个状态输出中文的 `【阶段开始】`、`【阶段成功】` 或
`【阶段失败】` 提示；`/pbvs/state` 仍保留英文值供程序订阅。

如果停在 `REACQUIRE_TARGET`，先观察 YOLO 是否仍有目标二维框。面板锁定
后近距离不再依赖中心深度；出现射线无法安全求交或 B1 法向漂移超过
`20 mm` 时，应检查手眼 TF、按钮框和初始面板锁定，不要增大 PBVS 位移
限制绕过保护。

如果在最初的 `WAIT_TARGET` 就出现 `No valid aligned depth`，说明相机还
没取得按钮三维位置，MoveIt 不会启动。先确认相机距离面板至少约
`0.35 m`，并使用主入口默认近距离配置：

```bash
ros2 launch piper_launch all.launch.py \
  camera_depth_width:=640 \
  camera_depth_height:=360 \
  camera_depth_fps:=30 \
  camera_device_preset:="High Density"
```

此时降低 `plane_min_points` 不能修复按钮框深度全为零的问题；若有效点
只有个位数，应增大相机距离或检查面板反光、遮挡和红外投射，而不是继续
放宽 PBVS 安全门限。

控制器没有力传感反馈。真机应使用柔性按键触头，并由操作员全程手持
急停。

## 7. 启动参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `model_path` | 工作区 `best.pt` | YOLO Detect 模型绝对路径 |
| `conda_env` | `yolo11` | YOLO Conda 环境 |
| `device` | 空 | 自动选择推理设备，也可设 `cpu`、`cuda:0` |
| `can_port` | `can0` | Piper CAN 接口 |
| `auto_enable` | `false` | 启动时自动使能机械臂 |
| `tcp_offset_z` | `0.1468` | J6 到标定按压 TCP 的 Z 偏置，单位米 |
| `start_camera` | `true` | 启动 Gemini 335L |
| `camera_depth_width` / `camera_depth_height` | `640` / `360` | 近距离原生深度分辨率 |
| `camera_depth_fps` | `30` | 原生深度帧率 |
| `camera_device_preset` | `High Density` | 提高深度有效像素填充率 |
| `start_piper` | `true` | 启动 Piper 实机驱动 |
| `start_moveit` | `true` | 启动 MoveIt 和 RViz |
| `start_pbvs` | `true` | 启动视觉、手眼 TF 和 PBVS |
| `use_rviz` | `true` | 随 MoveIt 启动 RViz |
| `enable_motion` | `false` | 真机运动开关；开启前必须完成轨迹控制器验收 |
| `enable_press` | `false` | 预留接触按压开关，要求运动已开启 |
| `orientation_mode` | `preserve_current_roll` | 保持任务起始滚转；`world_up` 为旧行为 |
| `coarse_horizontal_offset` | `0.0` | 粗定位水平补偿，单位米；面向电梯时正值向左、负值向右，范围 ±0.05 |
| `coarse_lateral_tolerance` | `0.007` | 粗定位面板切平面最大误差，单位米 |
| `enable_handeye_tf` | `true` | 发布当前功能包配置的手眼 TF |

查看完整参数：

```bash
ros2 launch piper_launch all.launch.py --show-args
```

## 8. 诊断

在主流程运行时，另开终端执行：

```bash
ros2 launch piper_launch diagnostic.launch.py duration:=8
```

该入口只测量关键话题频率，不启动或控制硬件：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/arm_status`
- `/joint_states`
- `/tcp_pose`
- `/piper_vision/button_pose`

进一步检查：

```bash
ros2 topic echo /pbvs/state
ros2 topic echo /piper_vision/button_pose
ros2 topic echo /pbvs/desired_tcp_pose
ros2 topic echo /pbvs/commanded_flange_pose
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 9. MoveIt 真机反馈与执行限制

当前已经移除任务入口中的模拟 `/joint_states`：

- Piper 驱动发布 `/joint_states`，独立关节名为 `joint1`～`joint7`。
- URDF 的 `joint8` 是 `joint7` 的反向 mimic 手指，不单独发布状态。
- `/joint_states_single` 继续保留旧版 `joint0`～`joint6` 接口。
- `/joint_ctrl_single` 不再从 `/joint_states` 重映射，避免反馈回灌成命令。
- MoveIt、TF 和 RViz 都以真机反馈作为当前状态。

重启后必须确认：

```bash
ros2 topic info /joint_states --verbose
ros2 topic echo --once /joint_states
```

`/joint_states` 应只有 `piper_ctrl_single_node` 一个发布者，且数值应随真机
关节运动变化。

任务入口不使用 `GenericSystem`。`piper_trajectory_controller` 是真实
`FollowJointTrajectory` 桥接，执行前会检查：

- `/joint_states` 恰好只有 `piper_ctrl_single_node` 一个发布者；
- `0x2A5`～`0x2A7` 关节反馈、`0x261`～`0x266` 六轴低速反馈均新鲜；
- 六个关节都处于使能状态；
- 轨迹关节、时间、数值和硬限位合法；
- 路径误差和终点误差没有超过 Action 容差。

启动后检查：

```bash
ros2 action info /arm_controller/follow_joint_trajectory
ros2 action info /gripper_controller/follow_joint_trajectory
ros2 topic info /joint_ctrl_single --verbose
```

两个 Action 都应只有 `piper_trajectory_controller` 一个服务端，命令话题也应
只有该节点一个发布者。仍应保持 `auto_enable:=false`，并先使用
`enable_motion:=false` 做规划验证。

## 10. 停止

正常情况下，等待 Action 完成回撤后再停止 launch。失能机械臂：

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable \
  "{enable_request: false}"
```

发生不可控运动、碰撞风险或硬件错误时，优先按下物理急停，不要依赖软件
回撤。
