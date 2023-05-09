# pred_occ_planner_plus
这项工作并没有改变原项目规划算法的核心思想，而是扩展出了一些功能，在接口和模型参数层面做出了一些调整：

1. 扩展了应用场景，使用动捕做位姿反馈并引入 px4 控制，使其可以用于物理世界中的无人机飞行
2. 扩展了仿真环境，在 Gazebo 中搭建 dynamic 环境，使其可以适配 Gazebo 的物理引擎和无人机模型
3. 评估了模型性能，提出4个指标来评价规划器的规划效果

## 安装
参考 [pred-occ-planner](https://github.com/edmundwsy/pred-occ-planner)

**测试环境**: Ubuntu 20.04 + ROS Noetic

---

## 新增功能1：基于动捕系统的无人机实际飞行（px4）
利用 [Optitrack](https://optitrack.com/software/motive/) 动作捕捉系统，将障碍物的真实位置速度信息通过 [vrpn_client_ros](https://github.com/ros-drivers/vrpn_client_ros) 传输到机载电脑上，使用 [pred-occ-planner](https://github.com/edmundwsy/pred-occ-planner) 中对障碍物的处理方式构造点云地图，往复飞行测试规划器效果
### 1. 启动方式

**terminal 1**

启动动捕系统，规划器和 mavros

   ```shell
   cd pred_occ_planner_plus_ws
   source devel/setup.zsh
   sh ./src/pred-occ-planner/plan_manager/scripts/px4_mocap.sh
   ```

**terminal 2**

启动 px4 控制

   ```shell
   cd pred_occ_planner_plus_ws
   source devel/setup.zsh
   roslaunch planner fly_G305_real.launch
   ```


### 2. 飞行前需要指定的参数

fly_G305_real.launch
   ```XML
   <!-- 轨迹起点的positon和orintation -->
   <arg name="init_x"  default="0.894"/>
   <arg name="init_y"  default="-0.393"/>
   <arg name="init_z"  default="1.300"/>
   <arg name="init_qx" default="0"/>
   <arg name="init_qy" default="0"/>
   <arg name="init_qz" default="0"/>
   <arg name="init_qw" default="1"/> 
   ```

occ_plan_real.launch
   ```XML
   <!-- 轨迹起点和终点的position -->
   <arg name="init_x" value="-1.1" />
   <arg name="init_y" value="1.1" />
   <arg name="init_z" value="1.1" />
   <arg name="goal_x" value="8" />
   <arg name="goal_y" value="-3" />
   <arg name="goal_z" value="1" />
   ```

drone_fake_perception.xml
   ```XML
   <!-- 轨迹起点的orintation -->
   <arg name="init_qx" default="0"/>
   <arg name="init_qy" default="0"/>
   <arg name="init_qz" default="0"/>
   <arg name="init_qw" default="1"/>
   ```
px4_ctl_real.cpp
   ```C++
   //轨迹终点的position和orientation
   ps.pose.orientation.w = 0;
   ps.pose.orientation.x = 0;
   ps.pose.orientation.y = -1;
   ps.pose.orientation.z = 0;
   ps.pose.position.x    = -3;
   ps.pose.position.y    = 0;
   ps.pose.position.z    = 1;
   ```

moving_cylinder_real.h
   ```C++
   //圆柱体障碍物的高h和宽w
   h = _rand_h(eng);
   w = _rand_w(eng);
   ```

---

## 新增功能2：基于Gazebo的仿真（px4）
在 Gazebo 中搭建了一个简单的 dynamic 环境，由Gazebo节点发布障碍物的位置速度信息，使用 [pred-occ-planner](https://github.com/edmundwsy/pred-occ-planner) 中对障碍物的处理方式构造点云地图，往复飞行测试规划器效果

![test](./images/test.gif)

### 1. 启动方式
按123顺序启动即可，无需等待终端

**terminal 1**

启动 Gazebo 仿真环境和规划器

   ```shell
   cd pred_occ_planner_plus_ws
   source devel/setup.zsh
   sh ./src/pred-occ-planner/plan_manager/scripts/px4_gazebo.sh
   ```

**terminal 2**

启动飞控仿真，这里需要先将 [px4](https://github.com/PX4/PX4-Autopilot) 克隆到工作空间并调整 release 版本为 1.12，我们要利用其中的仿真飞控

   ```shell
   cd PX4-Autopilot
   no_sim=1 make px4_sitl gazebo
   ```

**terminal 3**

启动 px4 控制

   ```shell
   cd pred_occ_planner_plus_ws
   source devel/setup.zsh
   roslaunch planner fly_G305_sim.launch
   ```

### 2. 可调参数
   sim_fake.yaml
   ```yaml
   # 无人机的尺寸
   drone_size_x: 0.8
   drone_size_y: 0.8
   drone_size_z: 0.1
   # A star算法搜索最大时间
   max_tau: 1.8
   # 单步规划的最大步长
   horizon: 6.0
   # 单步规划的最大容忍时间
   goal_tolerance: 1.0
   ```

   simulator_fake.launch
   ```XML
	<!-- 无人机感知障碍物的范围 -->
   <param name="sensing/radius" value="10.0"/>
   <!-- 无人机感知障碍物的频率 -->
	<param name="sensing/rate" value="20.0"/>
   <!-- 行人的直径（膨胀后） -->
	<param name="obs1w" value="0.85"/>
   <!-- 圆柱1的直径（膨胀后） -->
	<param name="obs2w" value="1.34"/>
   <!-- 圆柱2的直径（膨胀后）-->
	<param name="obs3w" value="1.4"/>
   ```

### 3. 注意
其他参数不要动，可能会导致意想不到的后果

---

## 新增功能3：规划器的性能评估
构造了4个指标来评估规划器的规划效果（在给定的最大 vel 和最大 acc 条件下）：
1. 单次飞行时间；无人机从轨迹起点规划路径飞行到轨迹终点所需的时间
2. 行程最大 acc：单次飞行过程中所规划的加速度指令中的最大加速度值，包括 x 方向和 y 方向
3. 行程最大 vel：单次飞行过程中所规划的速度指令中的最大速度值，包括 x 方向和 y 方向
4. 规划成功率：多次往复飞行中的失败飞行次数除以总飞行次数，规划失败定义为撞到障碍物

已经在 .launch 文件中加入了自动录制包含以上几个信息的 topic 的 rosbag，当检测到碰撞时会自动 kill 掉所有 ros 节点
（也会结束 rosbag 录制）。测试结束后处理 rosbag 只需运行提供的 python 脚本，即可以实现飞行数据的快速导出与规划器性能一键分析

   data_process.py
   ```shell
   cd pred_occ_planner_plus_ws/src/bags
   ./data_process.py
   ```

分析示例如下（见 ./bags/output/test.bag_data.csv）：

![record](./images/record.png)

---

## License & Contact & Acknowledgements
参考 [pred-occ-planner](https://github.com/edmundwsy/pred-occ-planner)