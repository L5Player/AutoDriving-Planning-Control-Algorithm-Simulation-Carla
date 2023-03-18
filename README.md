# 欢迎仪式
<b><strong>欢迎大家来到自动驾驶Player(L5Player)的自动驾驶算法与仿真空间，在这个空间我们将一起完成这些事情：<strong><b>
1. 控制算法构建基础模块并仿真调试：PID、LQR、Stanley 、MPC、滑膜控制、模糊控制、横向控制、纵向控制
2. 运动规划算法构建基础模块并仿真调试：样条曲线、贝塞尔曲线、ASTAR、RRT、动态规划、二次规划、EM Planer、Lattice Planer
3. 基于以上基础模块构建L2～L4功能模块： AEB、ACC、LKA、TJA、ALC、高速NOP、城市NOP、AVP
4. 文章、算法、理论、书籍分享；
5. 日常交流，行业咨询分享；
<br> 

<b><strong>建立这个项目的目的，是希望从零开始，搭建完整的自动驾驶系统，并且与大家共同完成<strong><b>
<br>
github地址: https://github.com/L5Player/AutoDriving-Planning-Control-Algorithm-Simulation-Carla<br>
gitte地址：https://gitee.com/nannanbe/auto-driving-planning-control-algorithm-simulation-carla

<br> 

# 博客地址
<b><strong>想要一起学习的伙伴，请关注我的CSDN、b站、知乎、公众号：自动驾驶Player(L5Player)<strong><b>
<br>CSDN有系列专栏可以订阅<br>
CSDN地址: https://blog.csdn.net/nn243823163/category_11685852.html?spm=1001.2014.3001.5482<br>
知乎地址：https://www.zhihu.com/people/L5Player

<br> 
<br> 

# 首先启动Crla仿真器
1. 在Carla下运行：./CarlaUE4.sh 或 ./CarlaUE4.sh -prefernvidia
<br><br>
![carla](./figures/carla.png) 
<br><br>

# AEB function scenario design and control
0. 通过python api设计AEB场景并实现功能
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. python vehicle_gallery_aeb.py
4. ros2 run carla_l5player_aeb_with_python_script carla_l5player_aeb_with_python_script_node
<br><br>

# LQR PID Controller with Waypoint
0. 通过Waypoint Publisher发布轨迹并进行跟随
1. source source_env.sh
2. ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
3. ros2 launch carla_l5player_lqr_pid_controller_waypoint lqr_launch.py
4. ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
<br><br>

# NEW PID 模块启动流程
0. 在PID Controler基础上更换导航路径，增加launch启动脚本以及rviz显示全局路径以及历史轨迹
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. ros2 run carla_l5player_pid_new_controller carla_l5player_pid_new_controller_node
4. 启动节点以及rviz: ros2 launch carla_l5player_pid_new_controller new_pid_launch.py
<br><br>

# PID 模块启动流程
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. ros2 run carla_l5player_pid_controller carla_l5player_pid_controller_node
<br><br>

# Stanley 模块启动流程
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. ros2 run carla_l5player_stanley_pid_controller carla_l5player_stanley_pid_controller_node
<br><br>

# LQR 模块启动流程
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. ros2 launch carla_l5player_lqr_pid_controller lqr_launch.py
<br><br>

# MPC 模块启动流程
1. source source_env.sh
2. ros2 launch carla_l5player_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. ros2 launch carla_l5player_mpc_controller mpc_launch.py
<br><br>

# 场景仿真器启动
首先将carla仿真器通过软连接添加到本项目同级目录下，然后进入项目目录
1. source source_env.sh
2. ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py scenario_runner_path:=/home/bea20/l5player_premium/auto-driving-planning-control-algorithm-simulation-carla/scenario_runner-0.9.13
<br><br>

