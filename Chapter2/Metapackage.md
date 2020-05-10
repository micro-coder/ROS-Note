# Metapackage

在一些ROS的教学资料和博客里，你可能还会看到一个Stack（功能包集）的概念，它指的是将多个功能接近、甚至相互依赖的软件包的放到一个集合中去。但Stack这个概念在Hydro之后就取消了，取而代之的就是Metapackage。尽管换了个马甲，但它的作用没变，都是把一些相近的功能模块、软件包放到一起。

ROS里常见的Metapacakge有：

Metapacakge名称|	描述|	链接
:---:|:---:|:---:
navigation|	导航相关的功能包集|	https://github.com/ros-planning/navigation
moveit|	运动规划相关的（主要是机械臂）功能包集|	https://github.com/ros-planning/moveit
image_pipeline|	图像获取、处理相关的功能包集|	https://github.com/ros-perception/image_common
vision_opencv|	ROS与OpenCV交互的功能包集|	https://github.com/ros-perception/vision_opencv
turtlebot	|Turtlebot机器人相关的功能包集	|https://github.com/turtlebot/turtlebot
pr2_robot	|pr2机器人驱动功能包集|	https://github.com/PR2/pr2_robot
...|	...	|...
以上列举了一些常见的功能包集，例如navigation、turtlebot，他们都是用于某一方面的功能，以navigation metapackage（官方介绍里仍然沿用stack的叫法）为例，它包括了以下软件包：

包名|	功能
:---:|:---:
navigation|	Metapacakge，依赖以下所有pacakge
amcl|	定位
fake_localization	|定位
map_server|	提供地图
move_base|	路径规划节点
nav_core|	路径规划的接口类
base_local_planner|	局部规划
dwa_local_planner|	局部规划
...	|...
这里只看一个软件包navigation。这个navigation就是一个简单的pacakge，里面只有几个文件，但由于它依赖了其他所有的软件包。Catkin编译系统会明白，这些软件包都属于navigation metapacakge。

这个道理并不难理解，比如我们在安装ROS时，用到了sudo apt-get install ros-kinetic-desktop-full命令，由于它依赖了ROS所有的核心组件，我们在安装时也就能够安装整个ROS。

## Metapackage写法

我们以ROS-Academy-for-beginners为例介绍meteapckage的写法。在教学包内，有一个ros-academy-for-beginners软件包，该包即为一个metapacakge，其中有且仅有两个文件：CMakeLists.txt和pacakge.xml。(这两个文件在github下载页中)
CMakeLists.txt写法如下：
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(ros_academy_for_beginners)
find_package(catkin REQUIRED)
catkin_metapackage()
```
package.xml写法如下:
```xml
<package>
    <name>ros_academy_for_beginners</name>
    <version>18.2.10</version>
    <description>
        --------------------------------------------------------------------------
        A ROS tutorial for beginner level learners. This metapacakge includes some
        demos of topic, service, parameter server, tf, urdf, navigation, SLAM...
        It tries to explain the basic concepts and usages of ROS.
        --------------------------------------------------------------------------
    </description>
    <maintainer email="chaichangkun@163.com">Chai Changkun</maintainer>
    <author>Chai Changkun</author>
    <license>BSD</license>
    <url>http://rosacademy.cn</url>

    <buildtool_depend>catkin</buildtool_depend>

    <run_depend>navigation_sim_demo</run_depend>
    <run_depend>param_demo</run_depend>
    <run_depend>robot_sim_demo</run_depend>
    <run_depend>service_demo</run_depend>
    <run_depend>slam_sim_demo</run_depend>
    <run_depend>tf_demo</run_depend>
    <run_depend>topic_demo</run_depend>

    <export>
        <metapackage/>
    </export>
</package>
```

metapacakge中的以上两个文件和普通pacakge不同点是：
 * CMakeLists.txt:加入了catkin_metapackage()宏，指定本软件包为一个metapacakge。
 * package.xml:标签将所有软件包列为依赖项，标签中添加标签声明。

metapacakge在我们实际开发一个大工程时可能有用