#### 1. `cav_traj_gen` 的输入消息来源
`cav_traj_gen` 主要通过 `rosNode/pub.cpp`（或相似命名的节点文件）订阅以下消息来获取状态和环境信息：

车辆自身状态：订阅了 /VehicleState 话题，类型为 cav_msgs/VehicleState。该消息包含了车辆的实时位置（x, y, z）、速度（speed_x/y/z）、姿态（heading, pitch, roll）以及当前的档位（gear_pos）等。
环境信息（障碍物）：订阅了 /Obstacle 话题，类型为 cav_msgs/ObstacleVec (基于 grep 结果推测)。
目标信息：订阅了 /Goal 话题，类型为 cav_msgs/Goal。
#### 2. `cav_control` 的修改方案：支持前进/后退及档位切换
`cav_control` 目前订阅 /PlanedPath 和 /VehicleState 进行控制。若要支持前进/后退切换，方案如下：

大致方案：

1. 识别轨迹方向：在 `cav_control` 接收到路径的 pathCB 回调中，检查 `cav_msgs/PlanedPath` 中的 forward_flag 和 backward_flag。或者通过计算路径点序列中速度 v 的正负号（在 Parking 项目中，这通常体现为速度符号）。
2. 状态机控制档位：
增加一个 target_gear 变量。
当需要从前进切换到后退时，控制信号应先将速度降为 0，然后发送特定的档位指令（如 cav_msgs/VehicleState 定义的 GEAR_REVERSE=2）。
3. 修改控制指令发布：
在发布 cav_msgs/Control（包含 throttle, brake, steer）的同时，确保输出的加速度或速度指令与当前档位匹配。
如果底层驱动支持，可以在 Control.msg 中增加档位字段，或者通过独立话题发布档位变更。
#### 3. 多项目集成方案建议：traj_planner 结果转换
由于 Parking 项目（基于 vehicle_msgs）和 TrajPlanRace 项目（基于 cav_msgs）的数据结构差异巨大（特别是坐标系：Parking 通常使用本地 UTM/平直坐标系，而 cav_msgs 的 PlanedPath 显式提到 x, y 为 degree 经纬度），我建议采取 方案 B：新增加一个功能包（转换节点）。

推荐方案：增加 traj_converter 节点
原因：

解耦性：不破坏 `cav_control` 原有的竞速/控制逻辑，保持控制器的通用性。
坐标转换职责清晰：Parking 的轨迹通常是局部米制单位，而 `cav_msgs/PlanedPath` 要求的是经纬度或特定的 GNSS 全球帧。这种“地图转换”逻辑放在独立的 converter 中更为妥当。
协议适配：Parking 的 ReferenceTrajectory 是一次性发布的完整路径，而 `cav_control` 可能期望的是 `cav_msgs/PlanedPath` 格式。
具体集成路线：

1. 创建一个新包 traj_bridge。
2. **订阅**：Parking 输出的 ref_traj (vehicle_msgs/ReferenceTrajectory)。
3. **处理**：
   1. **坐标转换**：将 `vehicle_msgs/State` 中的 x, y 从本地坐标系转回 `cav_msgs` 期望的经纬度（利用项目的 UTM 偏移量）。
   2. **标志位转换**：根据 `State.velocity` 的正负号，自动设置 `cav_msgs/PlanedPath` 中的 forward_flag 或 backward_flag。
   3. **重采样**：`cav_msgs/PlanedPath` 明确要求 t 必须是 0.04s 一个点（25Hz），转换节点可以进行线性插值以满足此要求。
4. **发布**：发布格式为 `cav_msgs/PlanedPath` 的话题（如 /PlanedPath），供 `cav_control` 直接使用。
这种做法能让你在不大幅改动 TrajPlanRace 核心控制代码的情况下，快速验证 Parking 规划器的算法效果。
