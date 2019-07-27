# Ubuntu16.04 + ROS kinetic

## 安装kinetic

1、安装来自中国的源：

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

2、设置key

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

3、更新

```bash
sudo apt-get update
```

安装，这里介绍Desktop-Full安装: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

```bash
sudo apt-get install ros-kinetic-desktop-full
```

4、解决依赖

```bash
sudo rosdep init
rosdep update
```

5、环境设置

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6、安装rosinstall,便利的工具

```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## **卸载Kinetic**

- 用apt-get方式安装Kinetic的，卸载使用如下命令：

```
$ sudo apt-get remove ros-kinetic-*
```

- 卸载成功的效果：就是/opt下的ROS文件夹Kinetic被删除

## **catkin方式创建ROS工作空间**

- 这些操作方法只适用于ROS Groovy及后期版本，对于ROS Fuerte及早期版本请选择rosbuild。
  下面我们开始创建一个catkin 工作空间：

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
```

- 即使这个工作空间是空的（在'src'目录中没有任何软件包，只有一个`CMakeLists.txt`链接文件），依然可以编译它：

```
$ cd ~/catkin_ws/
$ catkin_make
```

- catkin_make命令在catkin 工作空间中是一个非常方便的工具。如果你查看一下当前目录应该能看到'build'和'devel'这两个文件夹。在'devel'文件夹里面你可以看到几个setup.*sh文件。source这些文件中的任何一个都可以将当前工作空间设置在ROS工作环境的最顶层，想了解更多请参考catkin文档。接下来首先source一下新生成的setup.*sh文件：

```
$ source devel/setup.bash
```

- 要想保证工作空间已配置正确需确保`ROS_PACKAGE_PATH`环境变量包含你的工作空间目录，采用以下命令查看：

```
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks
```