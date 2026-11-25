# 第 11 组 · 走迷宫系统（TurtleBot3 迷宫左墙跟随）

《机器人感知与控制综合实践》课程代码包。
ROS 2 Humble + Gazebo Classic 11，在自定义迷宫世界里让 TurtleBot3 Burger 贴左侧墙行进：
激光雷达分区取距离 + 里程计记录轨迹，四状态阈值控制输出 `/cmd_vel`。

- 组长：陈伟泷（24061719）
- 组员：孙怡飞（24061610）
- 开源地址：https://github.com/<填写你的用户名>/ros2maze_ws

## 一、环境版本

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（VMware 虚拟机，7 GB 内存 / 6 核，无 GPU 直通） |
| ROS 2 | Humble |
| Gazebo | Gazebo Classic 11.10 |
| TurtleBot3 | `turtlebot3_gazebo` / `turtlebot3_description` 2.3.x |
| Python 依赖 | `rclpy`、`numpy`（`scan_region` 节点用）、可选 `matplotlib`（航点节点出图） |

依赖安装：

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
                 ros-humble-gazebo-ros-pkgs python3-numpy
```

## 二、编译

```bash
# ① 建工作空间并把 src/ 放进去
mkdir -p ~/ros2maze_ws && cd ~/ros2maze_ws
cp -r <本压缩包解压后的路径>/src .

# ② 编译（首次或改动 launch / worlds / rviz 后都要重编）
source /opt/ros/humble/setup.bash
colcon build --packages-select maze_control --symlink-install
source install/setup.bash
```

> `launch/`、`worlds/`、`rviz/` 是通过 `setup.py` 的 `data_files` 安装的，其中用到了
> `glob()`，只在编译时求值——新增或删除这三类文件必须重新 `colcon build`，
> 只改 Python 文件内容不用重编。

## 三、启动

一条命令拉起整套系统（Gazebo 世界 + Burger 模型 + robot_state_publisher + RViz2 + 算法节点）：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch maze_control bringup.launch.py
```

常用的几个变体：

```bash
# 只跑仿真，不跑算法
ros2 launch maze_control bringup.launch.py follower:=false
# 无界面（远程或性能不足）
ros2 launch maze_control bringup.launch.py gzclient:=false rviz:=false
# 换世界（避障测试场）
ros2 launch maze_control bringup.launch.py world:=$(ros2 pkg prefix maze_control)/share/maze_control/worlds/my_obstacle.world follower:=false
```

关闭：在启动终端按一次 `Ctrl+C`，`gzserver` / `gzclient` / `rviz2` / 算法节点会一起退出。

单节点运行（调试用）：

```bash
ros2 run maze_control tb3_wall_follower     # 左墙跟随主节点
ros2 run maze_control tb3_point2point       # 里程计闭环点到点
ros2 run maze_control tb3_waypoints         # 矩形四航点
ros2 run maze_control scan_region           # 雷达分区打印
ros2 run maze_control tb3_avoid             # 反应式避障
```

## 四、目录结构

```
ros2maze_ws/
├── README.md                      # 本文件
└── src/
    └── maze_control/
        ├── package.xml
        ├── setup.py               # 入口点与 launch/worlds/rviz 安装清单
        ├── setup.cfg
        ├── README.md              # 功能包说明（参数表、控制律、已知限制）
        ├── resource/maze_control  # ament 包标记文件
        ├── maze_control/          # 五个节点
        │   ├── tb3_wall_follower.py   # 左墙跟随（主算法）
        │   ├── tb3_point2point.py     # 点到点双 P
        │   ├── tb3_waypoints.py       # 矩形航点
        │   ├── scan_region_node.py    # 雷达分区取值
        │   └── tb3_avoid.py           # 反应式避障
        ├── launch/bringup.launch.py   # 一键启动
        ├── worlds/maze_tb3.world      # 主迷宫，20 段合并墙体
        ├── worlds/my_obstacle.world   # 避障测试场，5 个圆柱
        └── rviz/tb3_maze.rviz         # Fixed Frame=odom，5 个显示项
```

## 五、组员分工

| 成员 | 负责模块 | 具体工作 |
| --- | --- | --- |
| 陈伟泷（组长） | 迷宫场景建模、主算法与系统集成 | 搭建工作空间与功能包，设计并搭建 `maze_tb3.world`，编写 `tb3_wall_follower` 与点到点/航点节点，编写 `bringup.launch.py`，参数整定与联调，录制演示视频 |
| 孙怡飞（组员） | 雷达分区与避障模块、RViz2 配置、数据统计 | 编写并调试 `scan_region` 与 `tb3_avoid` 两个节点，配置 `rviz/tb3_maze.rviz`，用 `/odom` 与轨迹 CSV 统计数据，报告统稿 |

## 六、开源

- 仓库：https://github.com/<填写你的用户名>/ros2maze_ws
- 只包含 `src/`、`README.md` 与必要图示，不含 `build/`、`install/`、`log/`、`__pycache__`。
- 引用与参考见报告第 9 章。

## 七、已知限制

- 阈值按 1.2 m 走廊标定，进入净宽 2 m 以上的空间时 `left > 0.50` 会长期成立，车在宽走廊里绕圈；
- 没有“到达出口”的判定，开出场地边界后仍按原规则运行，需要人工结束；
- 出生点设在东北角走廊，尚未把“从西侧入口一路走到出口”的完整路线跑通。
