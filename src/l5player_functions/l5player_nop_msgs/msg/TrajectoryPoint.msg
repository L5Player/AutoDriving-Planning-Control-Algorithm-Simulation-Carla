# 世界坐标系下的参数
float64 x     # x position
float64 y      # y position
float64 z      # z position
float64 theta  # yaw in rad
float64 kappa  # curvature曲率
float64 dkappa # curvature曲率导数
float64 v      # Tangential velocity
float64 a      # Tangential acceleration
# Frenet坐标系下的属性
float64 relative_time # relative_time
float64 absolute_time # 轨迹的绝对时"
float64 d             # lateral offset
float64 d_d           # lateral speed
float64 d_dd          # lateral acceleration
float64 s             # s position along spline
float64 s_d           # s speed
float64 s_dd          # s acceleration
float64 s_ddd
float64 d_ddd