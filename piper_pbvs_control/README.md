# piper_pbvs_control

该包把按钮完整位姿转换为 Piper 的安全按键流程：

```text
WAIT_TARGET → COARSE_APPROACH → REACQUIRE_TARGET → PBVS_ALIGN
→ PRESS → HOLD → RETRACT → DONE
```

## 前置节点

先启动 Piper 实机驱动和当前可控制实机的 MoveIt `move_group`。不要同时
启动两套手眼 TF 发布器。随后启动视觉和控制：

```bash
source install/setup.bash
ros2 launch piper_pbvs_control elevator_press.launch.py \
  model_path:=/absolute/path/to/best.pt
```

默认 `enable_motion=false`、`enable_press=false`，只进行 MoveIt
`plan_only` 和目标计算，不会发布 `/pos_cmd`。

模型必须为每个可操作按钮提供唯一类别，例如 `up`、`down`、`floor_3`。
触发 dry-run：

```bash
ros2 action send_goal /press_button piper_msgs/action/PressButton \
  "{target_name: 'up'}" --feedback
```

仅开放粗定位和 PBVS 对准、禁止按压：

```bash
ros2 launch piper_pbvs_control elevator_press.launch.py \
  model_path:=/absolute/path/to/best.pt \
  enable_motion:=true \
  enable_press:=false
```

完成 dry-run 和仅对准验收后，才可显式设置 `enable_press:=true`。

按钮位姿来自 `/piper_vision/button_pose`，使用 `base_link` 表达：
位置是按钮表面中心，姿态 `+Z` 是按压方向。默认
`orientation_mode:=preserve_current_roll`：每次 Action 获取到稳定目标后，
控制器从最新 `/tcp_pose` 锁定任务起始滚转，只用最短旋转将 TCP 的 `+Z`
对齐按压方向，不再采用视觉姿态中可能导致末端额外旋转 90 度的 X/Y 轴。
这一个姿态基准会贯穿 MoveIt 粗定位、PBVS 对准、按压和回撤。碰撞面板仍
使用视觉原始姿态，不受该选项影响。

控制器先让 MoveIt 将 `tcp_link` 移到按压方向反向 `80 mm` 的粗定位点，
并用按钮局部坐标验收真机位置：面板切平面横向误差不超过 `5 mm`，
沿按压轴的按钮距离为 `80 ± 10 mm`。首次验收超差时保持同一个首次按钮
位置 B0 和粗定位目标 C0，再执行一次 MoveIt 校正；第二次仍超差则中止。
验收通过后清除移动前的旧视觉样本，并在新相机视角下重新获取稳定按钮位姿；
只有重新获取成功才用实时视觉闭环对准到 `5 mm`。最后冻结目标执行
`3 mm` 过行程按压、保持和回撤。

回撤目标不是 MoveIt 请求中的理想粗位姿，而是 MoveIt 粗定位完成时
`/tcp_pose` 报告的真实位姿。因为 MoveIt 允许在位置/姿态容差区域内
结束，真实位姿可能和请求值相差若干毫米；使用实测位姿可避免回撤时一直
追逐一个从未真正到达的目标。

如需复现旧版完整采用视觉姿态的行为，可以启动：

```bash
ros2 launch piper_pbvs_control elevator_press.launch.py \
  model_path:=/absolute/path/to/best.pt \
  orientation_mode:=world_up
```

`preserve_current_roll` 在 Action 开始时要求 `/tcp_pose` 是新鲜数据；若
反馈超时，任务会安全中止，不会发送运动命令。

当前安全默认值包括目标丢失超时、TCP 反馈超时、单周期平移/旋转限幅、
最大 PBVS 位移 `100 mm`、最大按压轴向行程 `12 mm` 和最大横向偏移
`4 mm`。任一机械臂错误码、关节限位或关节通信异常都会中止任务。

主要接口：

- Action：`/press_button`
- 输入：`/piper_vision/button_pose`、`/tcp_pose`、`/arm_status`
- 控制：`/move_action`、`/pos_cmd`
- 调试：`/pbvs/state`、`/pbvs/desired_tcp_pose`、
  `/pbvs/commanded_flange_pose`

`/pbvs/state` 继续使用英文状态值，便于其他节点稳定解析；终端会同时输出
中文阶段结果，例如：

```text
【阶段开始】MoveIt 粗定位（COARSE_APPROACH）
【阶段成功】MoveIt 粗定位（COARSE_APPROACH）：MoveIt 返回成功，实测 TCP 已进入按钮局部坐标粗定位容差（第 1 次）
【阶段失败】粗定位后重新获取按钮位姿（REACQUIRE_TARGET）：...
```

Action 的最终 `message` 也会说明是规划成功、对准成功、完整按压成功，
还是在哪个阶段中止。

主要启动参数：

- `orientation_mode:=preserve_current_roll`：默认，仅对齐按压轴，保持任务
  开始时的末端滚转。
- `orientation_mode:=world_up`：兼容旧行为，使用视觉输出的完整四元数。
- `enable_motion`：允许 MoveIt 执行及 PBVS 发布真机运动命令。
- `enable_press`：允许接触按压，要求 `enable_motion:=true`。
- `coarse_standoff`：MoveIt 粗定位距离，默认 `0.08 m`；近距离深度出现
  大面积无效时可先增至 `0.12 m` 验证。
- `coarse_horizontal_offset`：粗定位目标相对按钮的水平补偿，默认 `0 m`，
  面向电梯时正值向左、负值向右，允许范围为 `-0.05~+0.05 m`。若实机
  固定向右偏 `30 mm`，设置为 `+0.03`。
- `coarse_lateral_tolerance`：按钮局部 XY/面板切平面的最大误差，默认
  `0.007 m`。
- `coarse_axial_tolerance`：实际按钮法向距离相对 `coarse_standoff` 的允许
  误差，默认 `0.01 m`。
- `coarse_correction_attempts`：首次粗定位验收超差后的 MoveIt 校正次数，
  默认 `1`；校正前不重新采集视觉目标。
- `target_reacquire_timeout`：粗定位后在新视角重新获取稳定位姿的等待
  时间，默认 `8 s`。不要用增大 `target_abort_age` 代替视觉修复。
- `reacquire_max_normal_drift`：B1 相对 B0 的最大法向漂移，默认
  `0.02 m`；超限时不发送 PBVS 命令并安全回撤。

例如，粗定位后实机固定向右偏 `30 mm`，启动时加入：

```bash
ros2 launch piper_launch all.launch.py \
  coarse_horizontal_offset:=0.03
```

`/pos_cmd` 控制的是 J6 法兰。节点会根据 `link6 → tcp_link` 的
`0.1468 m` 标定偏置自动把 TCP 目标转换成法兰目标。

建议依次检查：

```bash
ros2 topic echo /piper_vision/button_pose
ros2 topic echo /pbvs/desired_tcp_pose
ros2 topic echo /pbvs/commanded_flange_pose
```

只有前三维位姿、按压方向和 MoveIt dry-run 都正确，才启用机械臂运动。

面板会在 MoveIt 运动前锁定到 `base_link`。粗定位后，B1 使用当前 RGB
按钮中心射线与锁定面板求交，因此近距离中心深度无效不会直接阻止重新
定位。PBVS 保持 T0 实测姿态不变，只细化位置，避免接近 `pitch=90°` 时
RPY 表示跳变。

若结果为 `stable button pose reacquisition timed out at the coarse
viewpoint`，检查 YOLO 终端是否仍能识别目标二维框，以及是否出现：

- `Button center ray does not safely intersect locked panel`：当前射线与
  锁定面板近平行、交点在相机后方或超过深度范围。
- `reacquired button pose exceeds normal drift limit`：B1 法向变化超过
  `20 mm`，任务会拒绝错误目标并回撤。

```
YOLO + 深度图
        ↓
  /piper_vision/button_pose
        ↓
  MoveIt 粗定位 + PBVS 计算
        ↓
  /pos_cmd
        ↓
  Piper 驱动
        ↓
  机械臂运动
```
