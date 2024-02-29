# DWA算法笔记

dwa算法基本思想：

1. 首先把机器人抽象为(v, w)二维平面二自由度(前后，旋转)的运动模型。
2. 然后根据机器人硬件限制和障碍物限制，计算当前位置和速度下的v 和 w 的范围。
3. 接着，设置一个分辨率采用v和w，计算每个采样的轨迹，并根据距离终点的状态和离障碍物的距离等因素对轨迹打分。
4. 最后选择分数最高的轨迹，作为控制命令。输出 v 和 w。

总结：思想是这么的朴实无华，代码实现也是如此的朴实无华，多么美丽的将连续世界转为了离散空间。

## 代码分析

首先，根据当前机器人状态（x，y，theta，v，w）确定速度和角速度搜索空间。

```python
# 根据当前状态，最大速度和角速度，最大加速度和角加速度，以及delta t，确定(v, w)搜索空间
def calc_dynamic_window(x, config: Config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw
```

接着，根据搜索空间分辨率，逐个搜索(v, w)空间，计算该（v，w）下的轨迹。

```python
# 计算轨迹，这里设定为3秒
def predict_trajectory(x_init, v, y, config: Config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory
```

然后，计算轨迹的代价，这里复现代码，使用了距离目标点的距离，当前速度，以及是否与障碍物相碰撞三个因素的加权和。最后选出代价最低的轨迹。

```python
for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * \
                calc_to_goal_cost(trajectory, goal) # 角度偏差越小越好
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - trajectory[-1, 3]) # 速度越大越好
            ob_cost = config.obstacle_cost_gain * \
                calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
```

## 学习总结

源码值得学习的内容有：判断矩形机器人是否与障碍物碰撞，这里使用了两个非常巧妙的思想。

1. 把障碍物转换到自车坐标系下面，只用判断障碍物的坐标是否在小车的AABB内，这样非常方便。（否则，需要使用向量叉乘的知识，判断平面内一个点是否在矩形内部）
2. 源码使用了矩阵的计算，一次性把所有轨迹点计算出来，提高了计算效率，值得学习。否则就需要手动for循环。cpp实现中依旧可以使用eigen库来实现。

实在是妙，妙不可言。

```python
if config.robot_type == RobotType.rectangle:
        # 这里就是计算所有轨迹是否与障碍物有碰撞，由于小车是矩形的，因此不能直接使用小车到障碍物的距离来判断。
        # 这里使用了很巧妙的办法，先把障碍物转换到小车局部坐标下面，然后判断障碍物是否在小车的范围内。
        # 备注：这里使用矩阵计算，一次性把所有的轨迹点都计算了，这是非常牛逼的地方。是我们需要学习的地方，否则
        # 按照已有的知识，我们只能遍历的计算每个轨迹点。这种矩阵计算的方式要高效很多。
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                       [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
```



