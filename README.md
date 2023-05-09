# espdrone_bridge

[espdrone](https://espressif-docs.readthedocs-hosted.com/projects/espressif-esp-drone/en/latest/gettingstarted.html) 是基于 Crazyflie 开发的 一款以 ESP32 为芯片的轻量化 quadrotor。由于无法搭载机载电脑，没有 mavros 的 setpoint 功能，仅凭官方给出的 [Python_lib](https://github.com/leeebo/crazyflie-lib-python/tree/esp-drone) 难以满足开发需求。

这项工作提供了一个 ros bridge，让 espdrone 的电机控制接口成为一个 ros 节点，基于此可以实现更为复杂的位置控制和轨迹追踪功能。其中，位置反馈由动捕系统实现。

**测试环境**: Ubuntu 20.04 + ROS Noetic

---

## Installation
首先要做一些硬件准备:

1. 一张无线网卡，我们需要让你的 Ubuntu 电脑作为地面站可以同时连接两个 wifi 网络，这样两张网卡可以分别负责**向无人机发送指令信息**和**从动捕电脑接受位置反馈**

2. 一台基于 Crazyflie 开发的微型无人机，当然它可以是 espdrone

run with your own computer
```bash
cd espdrone_ws/src
git clone https://github.com/bowenXuu/espdrone_bridge.git
cd ..
catkin build
source devel setup.zsh
```
---

## 控制模式
### Mode 1:  推力控制模式 

这种控制方式的控制器（基于 [lee controller](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_control) 开发的控制器）完全在地面站上实现，控制思想为由位置误差 `position_error` 和 速度误差 `velocity_error` 计算出期望加速度。再由期望加速度的大小和方向计算出期望升力，机身角速度和四元数，最终以 `<mavros_msgs::AttitudeTarget>` 的形式发布出来作为espdrone电机控制接口的输入。

**优点**是控制性能较好，可调参数较多

**缺点**是控制器和执行器之间通过网络连接，存在较高延迟：

lee_controller.launch
```XML
<!-- x方向位置误差的比例增益 -->
<arg name="ppx" default="1.0"/> 

<!-- y方向位置误差的比例增益 -->
<arg name="ppy" default="1.0"/>

<!-- z方向位置误差的比例增益 -->
<arg name="ppz" default="0.1"/>

<!-- x方向速度误差的比例增益 -->
<arg name="pvx" default="1.0"/>

<!-- y方向速度误差的比例增益 -->
<arg name="pvy" default="1.0"/>

<!-- z方向速度误差的比例增益 -->
<arg name="pvz" default="0.02"/>

<!-- 无人机的自身重量(kg) -->
<arg name="mass" default="0.044"/>

<!-- 无人机hover时输出推力占最大推力的比重 -->
<arg name="pth" default="0.65"/>

<!-- x方向的转向误差（四元数形式）的比例增益 -->
<arg name="angx" default="-4.0"/>

<!-- y方向的转向误差（四元数形式）的比例增益 -->
<arg name="angy" default="-4.0"/>

<!-- z方向的转向误差（四元数形式）的比例增益 -->
<arg name="angz" default="-4.0"/>
```
启动方式
```bash
sh ./src/espdrone-traj-tracking/scripts/esp_mocap_attitude.sh
```
### Mode 2:  位置控制模式
这种控制方式使用官方库中自带的set_position_setpoint() 函数，地面站只做轨迹点的按时间戳发布。

**优点**是可以在飞行时线上即时完成控制，基本可以做到无延迟

**缺点**是自带的位置控制器较为简单，控制效果较差。

启动方式

```bash
roslaunch espdrone-traj-tracking esp_position.launch
```

---

## 测试方法
1. 首先完成 espdrone 开机自检，确保四个电机连接正常

2. 将 attitude_ctrl_esp_drone.py 中的 upd 地址改为你的无人机 udp 地址

    ```python
    le1 = ESPPlanner('udp://192.168.43.42')
    ```
    确保两张网卡分别连接到 espdrone 的网络和动捕电脑所在网络
3. 发布你想要让无人机飞行的轨迹（可以是 rosbag 的形式），消息类型应该是`<trajectory_msgs::MultiDOFJointTrajectory>`，可以看到 espdrone 追踪给定轨迹的现象

## Contact
- Bowen Xu {[200320109@stu.hit.edu.cn]()}