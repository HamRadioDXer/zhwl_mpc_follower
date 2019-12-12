



<html>
  <embed src="//music.163.com/style/swf/widget.swf?sid=26203407&type=2&auto=1&width=320&height=66" width="340" height="86"  allowNetworking="all"></embed>
</html>


# MPC follower 说明
### 简介
基于模型预测控制（MPC）的航路点跟踪器，用于精确的路径跟踪。可以将它用作waypoint_follower，以及其他跟随节点的路径（例如pure_pursuit）。

---
与MPC关注者相关的节点有两个
- `/mpc_waypoint_converter`：转换`/final_waypoints`为`/mpc_waypoints`包含自身位置后方的航点。这是为了解决计划系统和mpc follower的暂时冲突，以便可以与pure_pursuit相同的方式使用mpc follower。
- `/mpc_follower`：生成要遵循的控制命令（`/twist_raw`或/和`/ctrl_raw`）`/mpc_waypoints`。
- 

---
## 节点的订阅和发布参考
- input
  - /mpc_waypoints : reference waypoints局部路点供mpc结算的那部分 (generated in mpc_waypoints_converter)
  - /current_pose : self pose（可以是定位模块的定位结果）
  - /vehicle_status : vehicle information 从测试车中读取速度转向角扭矩等等(as velocity and steering angle source)
 - output
    - /twist_raw : command for vehicle
    - /ctrl_raw : command for vehicle
    
### 节点组成
[![QrQiTg.md.png](https://s2.ax1x.com/2019/12/11/QrQiTg.md.png)](https://imgse.com/i/QrQiTg)

---
# 节点描述
> 默认速度为30km/h

## 节点参数

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|show_debug_info|bool|是否要显示调试信息|false|
|ctrl_period|double|control period （控制周期）[s]|0.03|
|traj_resample_dist|double|采样点间隔距离[m]|0.1|
|enable_path_smoothing|bool|path smoothing flag.当使用路径重采样以减少重采样噪声时，应该选中|true|
|enable_yaw_recalculation|bool|重新采样后重新计算偏航角。如果收到的航点中的偏航噪声很大，则设置为true|false|
|path_filter_moving_ave_num|int| 平滑路径的移动平均滤波器的数据点数|35|
|path_smoothing_times|int|应用路径平滑滤波器的次数|1|
|curvature_smoothing_num|double|`曲率`计算中使用的点的索引距离： `p(i-num), p(i), p(i+num)`。较大的数字会使噪声值较小|35|
|steering_lpf_cutoff_hz|double| 转向输出的低通滤波器截止频率 [hz]|3.0|
|admisible_position_error|double| 当跟随位置误差大于该值[m]时，停止车辆[m].|5.0|
|admisible_yaw_error_deg|double|当跟随偏航角误差大于该值[度]时，停止车辆[deg].|90.0|


## MPC 算法自身参数
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|qp_solver_type|string|QP解算器选项.|unconstraint_fast|
|qpoases_max_iter|int|（凸优化最大迭代次数）maximum iteration number for convex optimiaztion with qpoases.|500|
|vehicle_model_type|string|车辆运动学模型选项|kinematics|
|prediction_horizon|int|总的预测步骤|70|
|prediction_sampling_time|double|（论文词汇）prediction period for one step [s]|0.1|
|weight_lat_error|double|weight for lateral error|0.1|
|weight_heading_error|double|weight for heading error|0.0|
|weight_heading_error_squared_vel_coeff|double|weight for heading error * velocity|5.0|
|weight_steering_input|double|weight for steering error (steer command - reference steer)|1.0|
|weight_steering_input_squared_vel_coeff|double|（论文词汇）weight for steering error (steer command - reference steer) * velocity|0.25|
|weight_lat_jerk|double|weight for lateral jerk (steer(i) - steer(i-1)) * velocity|0.0|
|weight_terminal_lat_error|double|terminal cost weight for lateral error|1.0|
|weight_terminal_heading_error|double|terminal cost weight for heading error|0.1|
|zero_ff_steer_deg|double|前馈角阈值 [deg]. 小于此值的前馈角设置为零|2.0|

## 车辆物理参数

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|wheelbase|double|车辆模型的`wheel base`  [m] |2.9|
|steering_tau|double|转向动力学时间常数 (`1d approximation`) [s]|0.3|
|steer_lim_deg|double|steering angle limit for vehicle model [deg]. This is also used for QP constraint.|35.0|

## QP 求解器参数

- unconstraint : use least square method to solve unconstraint QP with eigen.
- unconstraint_fast : similar to unconstraint. This is faster, but lower accuracy for optimization.
- qpoases_hotstart : use QPOASES with hotstart for constrainted QP.

## 车辆模型参数

- kinematics（default） : (自相车模型)bicycle kinematics model with steering 1st-order delay
- kinematics_no_delay : （无转向延迟的自行车模型）bicycle kinematics model without steering delay
- dynamics : （动态建模，考虑侧滑偏移）bicycle dynamics model considering slip angle

# 调试
1. 设置适当的车辆运动性能的参数`wheelbase，steering_gear_ratio和steer_lim_deg`。还要检查`/vehicle_status`主题是否具有适当的值（速度：车辆后轮中心速度[km/h]，角度：转向（轮胎）角度[rad]）。这些值将车辆信息提供给控制器以进行路径跟踪。这些值中的错误会导致基本的跟踪错误。这些值是否正确，可以通过比较从模型（`/mpc_follower/debug/angvel_from_steer`）获得的角速度和实际角速度（例如`/estimate_twist/angular/z`）来确认。
2. 设置适当的车辆动力学参数steering_tau，该参数是从转向角命令到实际转向角的近似延迟。
3. 设置weight_steering_input= 1.0，weight_lat_error= 0.1，并将其他权重设置为0。如果在低速行驶时车辆振动，则将其`weight_lat_error`减小。
4. 其他权重的调整。调整的一种简单方法是增加`weight_lat_error`直到发生振荡。如果车辆不稳定且`weight_lat_error`跟小，则增加 `weight_terminal_lat_error``和weight_terminal_heading_error`提高跟踪稳定性。较大prediction_horizon和较小prediction_sampling_time对跟踪性能都是有效的，但是这是计算成本之间的折衷。

>其他参数可以如下调整。

- weight_lat_error：减少横向跟踪误差。这就像PID中的P增益一样。
- weight_heading_error：增加驱动器的动态响应能力。这就像PID中的D增益一样。
- weight_heading_error_squared_vel_coeff ：在高速范围内动态响应。
- weight_steering_input：减少跟踪中的振荡。
- weight_steering_input_squared_vel_coeff：减少高速范围内跟踪的振荡。
- weight_lat_jerk：减少侧向晃动。
- weight_terminal_lat_error：weight_lat_error为稳定起见，最好将其值设定为比正常侧向权重更大的值。
- weight_terminal_heading_error：weight_heading_error为了稳定起见，最好将其设置为比正常航向权值更大的值。



权zhi

 # 参考 

 [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics Institute, Carnegie Mellon University, February 2009.

