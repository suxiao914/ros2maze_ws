# maze_control —— TurtleBot3 迷宫左墙跟随

ROS 2 Humble 功能包。在自定义迷宫世界里让 TurtleBot3 Burger 贴左侧墙行进：
雷达分区取距离 + 里程计记录轨迹，四状态阈值控制输出 `/cmd_vel`。

## 1. 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（VMware 虚拟机，7 GB 内存 / 6 核，无 GPU 直通） |
| ROS 2 | Humble |
| Gazebo | Gazebo Classic 11.10 |
| TurtleBot3 | `turtlebot3_gazebo` / `turtlebot3_description` 2.3.x |
| Python 依赖 | `rclpy`、`numpy`（`scan_region` 节点用）、可选 `matplotlib`（航点节点出图） |

## 2. 三步跑起来

```bash
# ① 编译（首次或改动 launch/worlds/rviz 后需要）
cd ~/ros2maze_ws
colcon build --packages-select maze_control --symlink-install
source install/setup.bash

# ② 一键启动：Gazebo 世界 + Burger 模型 + robot_state_publisher + RViz2 + 算法节点
ros2 launch maze_control bringup.launch.py

# ③ 关闭：在该终端按一次 Ctrl+C，gzserver/gzclient/rviz2 会一起退出
```

只跑仿真、不跑算法：

```bash
ros2 launch maze_control bringup.launch.py follower:=false
```

无界面（远程或性能不足时）：

```bash
ros2 launch maze_control bringup.launch.py gzclient:=false rviz:=false
```

## 3. launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `world` | 包内 `worlds/maze_tb3.world` | 世界文件绝对路径 |
| `model` | `burger` | `burger` / `waffle` / `waffle_pi` |
| `x`、`y`、`yaw` | `2.4`、`2.5`、`3.14159` | 出生位姿（迷宫东北角走廊，车头朝 −x，左墙距 0.40 m） |
| `use_sim_time` | `true` | 所有节点跟随 `/clock` |
| `rviz` | `true` | 是否启动 RViz2 并加载 `rviz/tb3_maze.rviz` |
| `follower` | `true` | 是否在 8 s 后拉起 `tb3_wall_follower` |
| `gzclient` | `true` | `false` 时只跑 gzserver |

## 4. 节点

| 可执行名 | 订阅 | 发布 | 作用 |
| --- | --- | --- | --- |
| `tb3_point2point` | `/odom` | `/cmd_vel` | 里程计闭环点到点（双 P，目标 (1.0, 0.0)，阈值 0.05 m，20 Hz） |
| `tb3_waypoints` | `/odom` | `/cmd_vel`、`/trajectory` | 矩形四航点 (1,0)→(1,1)→(0,1)→(0,0)，出 `~/tb3_trajectory.csv` 与 `.png` |
| `scan_region` | `/scan` | 终端日志 | 分区取值：前方 [−15°,15°] 取最小，左右 [60°,90°] / [−90°,−60°] 取中值，带无效值掩码 |
| `tb3_avoid` | `/scan` | `/cmd_vel` | 反应式避障：前方 0.8 m 内有障碍就转向 |
| `tb3_wall_follower` | `/scan`、`/odom` | `/cmd_vel`、`/trajectory` | 左墙跟随主节点，同时写轨迹 CSV |

单独运行示例：

```bash
ros2 run maze_control tb3_wall_follower
ros2 run maze_control scan_region
```

## 5. 左墙跟随控制律

目标左墙距 0.40 m，线速度 0.15 m/s，转弯角速度 0.60 rad/s，控制周期 0.1 s：

| 状态 | 条件 | linear.x | angular.z |
| --- | --- | --- | --- |
| S3 前方遇墙 | `front < 0.35` | 0.00 | −0.60 |
| S1 离墙太远 | `left > 0.50` | 0.15 | +0.40 |
| S2 贴墙太近 | `left < 0.30` | 0.15 | −0.30 |
| S0 保持距离 | 0.30 ≤ `left` ≤ 0.50 | 0.15 | 0.00 |

分支顺序即优先级（S3 最高），输出只与当前帧有关，属于 Moore 型状态机。

## 6. 输出文件

- `~/tb3_trajectory.csv`：`t, x, y`，10 Hz 采样（墙跟随节点）
- `~/tb3_trajectory.png`：航点节点跑完四个航点后自动保存的轨迹图（需 matplotlib）

> 两个节点写的是同一个 CSV 路径，表头不同，请勿同时运行。

## 7. 目录结构

```
maze_control/
├── package.xml
├── setup.py                # 入口点与 launch/worlds/rviz 安装清单
├── setup.cfg
├── resource/maze_control   # ament 包标记
├── maze_control/           # 五个节点
├── launch/bringup.launch.py
├── worlds/maze_tb3.world   # 主迷宫，20 段合并墙体
├── worlds/my_obstacle.world# 避障测试场，5 个直径 1.0 m 圆柱
└── rviz/tb3_maze.rviz      # Fixed Frame=odom，5 个显示项
```

## 8. 已知限制

- 阈值按 1.2 m 走廊标定，进入净宽 2 m 以上的空间时 `left > 0.50` 会长期成立，车在宽走廊里绕圈；
- 没有到达判定，开出场地边界后仍按原规则运行，需要人工结束；
- 本场景出生点在东北角走廊，尚未把“从西侧入口一路走到出口”的完整路线跑通。

## 9. 分工

第 11 组：

- **组长 陈伟泷（24061719）**：迷宫场景与 `maze_tb3.world`、`tb3_wall_follower`、`tb3_point2point`、
  `tb3_waypoints`、`bringup.launch.py`、参数整定与联调、演示视频；
- **组员 孙怡飞（24061610）**：`scan_region`、`tb3_avoid`、`rviz/tb3_maze.rviz`、`/odom` 与轨迹数据统计、报告统稿。

## 10. AI 使用说明

报告的文字组织与版式由 AI 辅助完成，代码由组员编写与调试；AI 用于解释报错、讨论参数与检查文档中的事实性错误。

## 11. 开源

- 仓库地址：https://github.com/<填写你的用户名>/ros2maze_ws
- 仓库只包含 `src/`、`README.md` 与必要图示，不含 `build/`、`install/`、`log/`、`__pycache__`。
- 工作空间级的说明见仓库根目录的 `README.md`。
