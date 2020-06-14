# Client library和roscpp

## Client library

ROS为机器人开发者们提供了不同语言的编程接口，比如C++接口叫做roscpp，Python接口叫做rospy，Java接口叫做rosjava。尽管语言不通，但这些接口都可以用来创建topic、service、param，实现ROS的通信功能。Clinet Lirary有点类似开发中的Helper Class，把一些常用的基本功能做了封装。

目前ROS支持的Clinet Library包括：

Client Library	|介绍
:---:|:---:
roscpp	|ROS的C++库，是目前最广泛应用的ROS客户端库，执行效率高
rospy	|ROS的Python库，开发效率高，通常用在对运行时间没有太大要求的场合，例如配置、初始化等操作
roslisp|	ROS的LISP库
roscs	|Mono/.NET.库，可用任何Mono/.NET语言，包括C#，Iron Python， Iron Ruby等
rosgo	|ROS Go语言库
rosjava|	ROS Java语言库
rosnodejs|	Javascript客户端库
...	|...

目前最常用的只有roscpp和rospy，而其余的语言版本基本都还是测试版。

从开发客户端库的角度看，一个客户端库，至少需要能够包括master注册、名称管理、消息收发等功能。这样才能给开发者提供对ROS通信架构进行配置的方法。  
整个ROS包括的packages如下，你可以看到roscpp、rospy处于什么位置。
![ros_pkgs.png](picture/ros_pkgs.png)

## roscpp

roscpp is a C++ implementation of ROS. It provides a client library that enables C++ programmers to quickly interface with ROS Topics, Services, and Parameters. roscpp is the most widely used ROS client library and is designed to be the high-performance library for ROS.

roscpp位于/opt/ros/kinetic之下，用C++实现的ROS功能库。在ROS中，C++的代码是通过catkin这个编译系统（扩展的CMake）来进行编译构建的。所以简单地理解，你也可以把roscpp就当作为一个C++的库，我们创建一个CMake工程，在其中include了roscpp等ROS的libraries，这样就可以在工程中使用ROS提供的函数了。  

通常我们要调用ROS的C++接口，首先就需要#include <ros/ros.h>。

* roscpp的主要部分包括：
* ros::init() : 解析传入的ROS参数，创建node第一步需要用到的函数
* ros::NodeHandle : 和topic、service、param等交互的公共接口
* ros::master : 包含从master查询信息的函数
* ros::this_node：包含查询这个进程(node)的函数
* ros::service：包含查询服务的函数
* ros::param：包含查询参数服务器的函数，而不需要用到NodeHandle
* ros::names：包含处理ROS图资源名称的函数

具体可见：http://docs.ros.org/api/roscpp/html/index.html

以上功能可以分为以下几类：

* Initialization and Shutdown 初始与关闭
* Topics 话题
* Services 服务
* Parameter Server 参数服务器
* Timers 定时器
* NodeHandles 节点句柄
* Callbacks and Spinning 回调和自旋（或者翻译叫轮询）
* Logging 日志
* Names and Node Information 名称管理
* Time 时钟
* Exception 异常

看到这么多接口，千万别觉得复杂，我们日常开发并不会用到所有的功能，你只需对经常使用的有一些印象，掌握几个比较常见和重要的用法就足够了。