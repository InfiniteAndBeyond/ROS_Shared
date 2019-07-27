

```
mkdir -p ros_ws/src
cd ros_ws/
catkin_make
```

安装turtlebot3仿真机器人

```bash
 roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
 roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch 
```

启动gazebo仿真环境，首次加载地图十分耗费时间

```bash
export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```

WanderBot：

3秒改变一次运动状态，初始0.5m/s的速度沿x直行，若正前方有障碍物，则原地旋转。若前面无障碍物，则速度提升到1m/s



执行方法：

```bash
cd wanderbot
python red_light_green_light.py
python  wander.py
```

机器人就开始运动了