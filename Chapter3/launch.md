# launch文件

机器人是一个系统工程，通常一个机器人运行操作时要开启很多个node，对于一个复杂的机器人的启动操作应该怎么做呢？当然，我们并不需要每个节点依次进行rosrun，ROS为我们提供了一个命令：  能一次性启动master和多个node。该命令是：
```
roslaunch pkg_name file_name.launch
```
roslaunch命令首先会自动检测系统的roscore有没有运行，也即是确认节点管理器master是否处于运行状态，如果master没有启动，那么roslaunch就会首先启动master，然后再按照launch文件的规则执行。launch文件里已经配置好了启动规则。 所以roslaunch就像是一个启动工具，能够一次性把多个节点按照我们预先的配置启动起来，减少我们在终端中一条条输入指令的麻烦。

## 一、lanunch文件写法与与格式

launch文件同样也遵循着xml格式规范，是一种标签文本，它的格式包括以下标签：
```xml
<launch>    <!--根标签-->
    <node>    <!--需要启动的node及其参数-->
    <include>    <!--包含其他launch-->
    <machine>    <!--指定运行的机器-->
    <env-loader>    <!--设置环境变量-->
    <param>    <!--定义参数到参数服务器-->
    <rosparam>    <!--启动yaml文件参数到参数服务器-->
    <arg>    <!--定义变量-->
    <remap>    <!--设定参数映射-->
    <group>    <!--设定命名空间-->
</launch>    <!--根标签-->
```
参考链接:http://wiki.ros.org/roslaunch/XML

## 二、launch示例

launch文件的写法和格式看起来内容比较复杂，我们先来介绍一个最简单的例子如下：
```xml
<launch>
    <node name="talker" pkg="rospy_tutorials" type="talker" />
</launch>
```
这是官网给出的一个最小的例子。文本中的信息是，它启动了一个单独的节点talker,该节点是包rospy_tutorials软件包中的节点。
然而实际中的launch文件要复杂很多，我们以Ros-Academy-for-Beginners中的robot_sim_demo为例：
```xml
<launch>
<!--arg是launch标签中的变量声明，arg的name为变量名，default或者value为值-->
<arg name="robot" default="xbot2"/>
<arg name="debug" default="false"/>
<arg name="gui" default="true"/>
<arg name="headless" default="false"/>

<!-- Start Gazebo with a blank world -->
<include file="$(find gazebo_ros)/launch/empty_world.launch"> <!--include用来嵌套仿真场景的launch文件-->
<arg name="world_name" value="$(find robot_sim_demo)/worlds/ROS-Academy.world"/>
<arg name="debug" value="$(arg debug)" />
<arg name="gui" value="$(arg gui)" />
<arg name="paused" value="false"/>
<arg name="use_sim_time" value="true"/>
<arg name="headless" value="$(arg headless)"/>
</include>

<!-- Oh, you wanted a robot? --> <!--嵌套了机器人的launch文件-->
<include file="$(find robot_sim_demo)/launch/include/$(arg robot).launch.xml" />

<!--如果你想连同RViz一起启动，可以按照以下方式加入RViz这个node-->
<!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_sim_demo)/urdf_gazebo.rviz" /-->
</launch>
```
这个launch文件相比上一个简单的例子来说，内容稍微有些复杂，它的作用是：启动gazebo模拟器，导入参数内容，加入机器人模型。