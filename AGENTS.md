明白 ✅ 你希望我把刚才写的 **AGENTS.md**，改写成跟你提供的 **简洁样例风格**一致（分段更紧凑，语气更直接，不是报告体，而是“给 agent 开发者的指令手册”风格）。
下面是对照样例后的修改版：

---

# AGENTS.md — ROS 2 Gazebo Inspection Robot (fixed versions, CPU-friendly)

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
