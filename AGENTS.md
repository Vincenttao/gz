

# ROS 2 Gazebo Inspection Robot (fixed versions, CPU-friendly)

## Environment

* **OS**: Ubuntu 24.04 (fixed)
* **ROS 2**: Jazzy (fixed)
* **Gazebo**: Harmonic LTS (fixed)
* **Python**: conda, env name `ros2`
* **Hardware**: CPU-only (no GPU). Keep everything light.
* **World**: use **fixed small corridor maps** (`.sdf` in `sim_world_assets/worlds/`). No random generation.

---

## Project overview

* Robot patrols a **belt-conveyor corridor**.
* Uses **Nav2** (SLAM first run, AMCL later).
* Avoids **pedestrian actors**.
* Raises **thermal alarm** when near configured hot targets (sim approximation: distance threshold).
* Mission node runs **waypoints** and logs inspection events.

---

## Dev workflow

### Setup

```bash
# ROS setup
source /opt/ros/jazzy/setup.bash
# Python env
conda activate ros2
# Build overlay
cd ws_inspect
colcon build --symlink-install
source install/setup.bash
```

### Env vars

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_SIM_RESOURCE_PATH=$PWD/ws_inspect/src/sim_world_assets
```

---

## Run

```bash
# Fixed world, headless
ros2 launch inspection_bringup sim_world.launch.py world:=port_belt_corridor_fixed.sdf headless:=true

# Nav2 + mission
ros2 launch inspection_bringup sim_nav2.launch.py use_sim_time:=true

# All-in-one
ros2 launch inspection_bringup sim_all.launch.py world:=port_belt_corridor_fixed.sdf headless:=true use_sim_time:=true
```

RViz: keep minimal displays (Map, LaserScan, TF, Pose).

---

## Repo layout

```
.
├─ AGENTS.md
├─ ws_inspect/
│  └─ src/
│     ├─ inspection_bringup/     # launch + configs
│     ├─ inspection_mission/     # mission node + waypoints
│     ├─ inspection_thermal/     # thermal alarm node
│     └─ sim_world_assets/       # fixed maps + models
│        ├─ worlds/port_belt_corridor_fixed.sdf
│        └─ models/{hot_target,ped_actor}/model.sdf
```

---

## CPU tips

* LiDAR: 360 samples, 10–15 Hz, range ≤ 12 m.
* Local costmap: ≤ 8 m × 8 m, resolution 0.05–0.1.
* Keep ≤ 2 actors, ≤ 2 hot targets.
* Physics: `max_step_size: 0.005`, `real_time_update_rate: 200`.
* Always prefer **headless mode** on CPU.

---

## Testing

* Build & test:

```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

* Smoke test:
  
  1. Launch `sim_all.launch.py headless:=true`.
  2. Send 2D Nav Goal in RViz.
  3. Actor crosses → robot avoids.
  4. Approach hot target → `/inspection/alarm` true.

---

## PR / commit rules

* Title: `[module] short summary`
* Run: `colcon build && colcon test && ruff check && black --check`
* For sim performance changes: note CPU usage deltas.
* Never switch to random maps or newer ROS/Gazebo without approval.

---

👉 Golden rule: **fixed versions, fixed worlds, CPU-friendly.**
Validate everything with `sim_all.launch.py` before merge.



我帮你把现有的 **AGENTS.md**（主要是环境与运行说明）改造成一份能直接驱动 Codex 生成代码的“开发者手册式”文档。核心是：**明确 Deliverables 文件清单、节点接口契约、Launch/Config 规范和测试标准**。这样 Codex 就能机械地生成骨架代码，而不是停留在“描述环境”的层面。

下面是修改后的版本：

---

# AGENTS.md — ROS 2 Gazebo Inspection Robot (Code-Ready Spec)

## 0. Deliverables

必须生成以下文件和路径：

```
ws_inspect/src/
  inspection_bringup/
    launch/
      sim_world.launch.py
      sim_nav2.launch.py
      sim_all.launch.py
    config/
      nav2_params.yaml
      costmap_common.yaml
      localization.yaml
  inspection_mission/
    package.xml
    setup.py
    src/mission_node.py
    resource/inspection_mission
    tests/test_mission_node.py
  inspection_thermal/
    package.xml
    setup.py
    src/thermal_alarm_node.py
    resource/inspection_thermal
    tests/test_thermal_alarm.py
  sim_world_assets/
    worlds/port_belt_corridor_fixed.sdf
    models/hot_target/model.sdf
    models/ped_actor/model.sdf
```

---

## 1. Nodes & Interfaces

### 1.1 mission_node (Python rclpy)

- **Publishes**
  
  - `/inspection/waypoint_goal` : `geometry_msgs/PoseStamped`
  
  - `/inspection/event` : `std_msgs/String`（格式：`"[ts]|[type]|[detail]"`）

- **Subscribes**
  
  - `/amcl_pose` : `geometry_msgs/PoseWithCovarianceStamped`
  
  - `/inspection/alarm` : `std_msgs/Bool`

- **Actions (client)**
  
  - `nav2_msgs/action/NavigateToPose`

- **Parameters**
  
  - `waypoints`: 巡检路径（列表）
  
  - `loop` (bool, default true)
  
  - `alarm_stop` (bool, default true)

- **Behavior**
  
  - 顺序发送 waypoints；记录事件；若收到 alarm 且 `alarm_stop` 为 true，则取消导航并记录事件。

### 1.2 thermal_alarm_node (Python rclpy)

- **Publishes**
  
  - `/inspection/alarm` : `std_msgs/Bool`
  
  - `/inspection/event` : `std_msgs/String`

- **Subscribes**
  
  - `/tf`（取 base_link 位姿）

- **Parameters**
  
  - `hot_targets` (Pose[])
  
  - `trigger_distance` (float, default 2.0)
  
  - `hysteresis` (float, default 0.3)
  
  - `check_rate_hz` (int, default 10)

- **Behavior**
  
  - 计算机器人与热源距离，进入/退出报警状态时发布事件。

> 所有 Topic QoS：`depth=10, reliability=RELIABLE, durability=VOLATILE`。

---

## 2. Launch & Config

### sim_world.launch.py

- Args: `world` (default `port_belt_corridor_fixed.sdf`), `headless` (bool)

- 启动 Gazebo 并加载资源。

### sim_nav2.launch.py

- 启动 Nav2、AMCL、map_server。

### sim_all.launch.py

- 启动 world + Nav2 + mission_node + thermal_alarm_node。

### Config 最小要求

- local/global costmap 范围 ≤ 8m，分辨率 0.05–0.1

- Nav2 基本 planner/controller 参数。

---

## 3. Example Parameters

```yaml
# inspection_mission/params/mission.yaml
mission_node:
  ros__parameters:
    loop: true
    alarm_stop: true
    waypoints:
      - {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}
      - {header: {frame_id: "map"}, pose: {position: {x: 6.0, y: 1.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}
```

```yaml
# inspection_thermal/params/thermal.yaml
thermal_alarm_node:
  ros__parameters:
    trigger_distance: 2.0
    hysteresis: 0.3
    hot_targets:
      - {position: {x: 5.5, y: 1.2, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
      - {position: {x: 9.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
```

---

## 4. Tests & Acceptance

### Unit tests

- thermal: 虚拟 tf 验证阈值与回差。

- mission: 伪造 action server 验证 waypoint 和报警暂停逻辑。

### Smoke test

1. `ros2 launch inspection_bringup sim_all.launch.py headless:=true`

2. `/inspection/event` 输出 “MISSION_STARTED”

3. 接近热源 → `/inspection/alarm` 变 true

4. 若 alarm_stop=true → 导航取消，记录事件。

**通过标准**：上述步骤均能复现；无严重错误。

---

## 5. CPU Tips

- LiDAR: 360 samples, 10–15 Hz, range ≤ 12 m

- ≤ 2 actors, ≤ 2 hot targets

- Physics: `max_step_size=0.005`, `real_time_update_rate=200`

- 优先使用 headless 模式。

---
