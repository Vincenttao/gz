# ROS 2 Gazebo Inspection Robot 项目分析学习笔记

## 1. 项目概览

这是一个基于ROS 2 Jazzy和Gazebo Harmonic的巡检机器人仿真项目，专为CPU友好的环境设计。项目实现了以下核心功能：

- 机器人在传送带走廊环境中巡逻
- 使用Nav2进行SLAM建图和AMCL定位
- 避免行人演员（pedestrian actors）
- 当接近热源目标时触发热警报
- 任务节点执行预设的航路点并记录巡检事件

## 2. 目录结构分析

### 2.1 总体结构
ws_inspect/src/ 
├── inspection_bringup/ # 系统启动和配置 
├── inspection_mission/ # 任务管理节点 
├── inspection_thermal/ # 热警报检测节点 
└── sim_world_assets/ # 仿真世界和模型资源


### 2.2 inspection_bringup 包

这是系统的启动和配置包，包含：

1. **Launch文件** (`launch/`目录)：
   - `sim_world.launch.py`: 启动Gazebo仿真环境
   - `sim_nav2.launch.py`: 启动Nav2导航栈
   - `sim_all.launch.py`: 一站式启动所有组件

2. **配置文件** (`config/`目录)：
   - `nav2_params.yaml`: Nav2导航参数配置
   - `costmap_common.yaml`: 代价地图通用配置
   - `localization.yaml`: 定位参数配置

### 2.3 inspection_mission 包

任务管理节点包，负责航路点导航和事件记录：

1. **核心实现** (`src/inspection_mission/`目录)：
   - `mission_node.py`: 主要的任务节点实现

2. **参数配置** (`params/`目录)：
   - `mission.yaml`: 任务参数配置，包括航路点列表

3. **测试** (`tests/`目录)：
   - `test_mission_node.py`: 任务节点单元测试

### 2.4 inspection_thermal 包

热警报检测节点包，负责监控热源并触发警报：

1. **核心实现** (`src/inspection_thermal/`目录)：
   - `thermal_alarm_node.py`: 热警报节点实现

2. **参数配置** (`params/`目录)：
   - `thermal.yaml`: 热警报参数配置，包括热源位置

3. **测试** (`tests/`目录)：
   - `test_thermal_alarm.py`: 热警报节点单元测试

### 2.5 sim_world_assets 包

仿真资源包，包含Gazebo模型和世界文件：

1. **世界文件** (`worlds/`目录)：
   - `port_belt_corridor_fixed.sdf`: 固定的传送带走廊世界

2. **模型** (`models/`目录)：
   - `hot_target/`: 热源目标模型
   - `inspection_bot/`: 巡检机器人模型
   - `ped_actor/`: 行人演员模型

## 3. 核心组件详细分析

### 3.1 Mission Node (任务节点)

**功能**：
- 管理航路点导航序列
- 记录巡检事件
- 响应热警报以取消导航

**接口**：
- 发布：
  - `/inspection/waypoint_goal`: 目标航路点
  - `/inspection/event`: 事件日志
- 订阅：
  - `/amcl_pose`: 机器人定位
  - `/inspection/alarm`: 热警报状态
- Action客户端：
  - `nav2_msgs/action/NavigateToPose`: 导航到指定位置

**参数**：
- `waypoints`: 巡检路径点列表
- `loop`: 是否循环执行任务
- `alarm_stop`: 收到警报时是否停止导航

### 3.2 Thermal Alarm Node (热警报节点)

**功能**：
- 监控机器人与热源的距离
- 当距离低于阈值时触发警报
- 实现回差机制防止警报抖动

**接口**：
- 发布：
  - `/inspection/alarm`: 热警报状态
  - `/inspection/event`: 事件日志
- 订阅：
  - `/tf`: 机器人位姿变换

**参数**：
- `hot_targets`: 热源目标位置列表
- `trigger_distance`: 触发距离阈值
- `hysteresis`: 回差距离
- `check_rate_hz`: 检查频率

### 3.3 Launch系统

项目提供了三个层次的启动文件：

1. **sim_world.launch.py**: 仅启动Gazebo仿真环境
2. **sim_nav2.launch.py**: 启动Nav2导航栈
3. **sim_all.launch.py**: 一站式启动所有组件，包括世界、导航、任务和热警报节点

## 4. 配置和参数

### 4.1 Nav2配置

配置文件包含了完整的Nav2参数设置：
- AMCL定位参数
- 控制器参数（速度、加速度限制等）
- 路径规划器参数
- 代价地图参数（局部和全局）
- 行为服务器参数

### 4.2 任务参数

任务参数定义了巡检路径：
- 航路点列表（地图坐标系中的位置和方向）
- 循环执行选项
- 警报响应选项

### 4.3 热警报参数

热警报参数定义了监控规则：
- 热源目标位置
- 触发距离阈值
- 回差距离
- 检查频率

## 5. 测试策略

项目采用了单元测试策略：
- 使用模拟对象测试任务节点的航路点导航逻辑
- 使用模拟位姿提供器测试热警报节点的距离计算和阈值判断
- 验证事件发布和状态转换

## 6. CPU优化考虑

根据AGENTS.md文档，项目在设计上考虑了CPU优化：
- LiDAR设置为360样本，10-15Hz频率，最大距离12米
- 代价地图范围限制为8m×8m，分辨率0.05-0.1
- 限制演员和热源数量不超过2个
- 物理引擎参数：最大步长0.005，更新率200Hz
- 优先使用无头模式运行

## 7. 开发工作流

项目的标准开发工作流包括：
1. 设置ROS 2环境和Python虚拟环境
2. 使用colcon构建工作空间
3. 运行不同的launch文件进行测试
4. 执行单元测试验证功能
5. 进行冒烟测试验证整体功能

## 8. 总结

这个项目结构清晰，模块化程度高，各组件职责明确，便于维护和扩展。通过合理的参数配置和测试策略，确保了系统的可靠性和性能。项目特别针对CPU友好的环境进行了优化，适用于资源受限的计算平台。