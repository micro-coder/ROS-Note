# node & master

## 一、 node

在ROS的世界里，最小的进程单元就是节点（node）。一个软件包里可以有多个可执行文件，可执行文件在运行之后就成了一个进程(process)，<font color="red">**这个进程在ROS中就叫做节点**</font>。 从程序角度来说，node就是一个可执行文件（通常为C++编译生成的可执行文件、Python脚本）被执行，加载到了内存之中；从功能角度来说，<font color="red">**通常一个node负责者机器人的某一个单独的功能**</font>。由于机器人的功能模块非常复杂，我们往往不会把所有功能都集中到一个node上，而会采用分布式的方式，把鸡蛋放到不同的篮子里。例如有一个node来控制底盘轮子的运动，有一个node驱动摄像头获取图像，有一个node驱动激光雷达，有一个node根据传感器信息进行路径规划……这样做可以降低程序发生崩溃的可能性，试想一下如果把所有功能都写到一个程序中，模块间的通信、异常处理将会很麻烦。

我们在<font color="green">**第二篇 ROS配置**</font>打开了小海龟的运动程序和键盘控制程序，在1.5节同样启动了键盘运动程序，这每一个程序便是一个node。ROS系统中不同功能模块之间的通信，也就是节点间的通信。我们可以把键盘控制替换为其他控制方式，而小海龟运动程序、机器人仿真程序则不用变化。这样就是一种模块化分工的思想。

## 二、master

由于机器人的元器件很多，功能庞大，因此实际运行时往往会运行众多的node，负责感知世界、控制运动、决策和计算等功能。那么如何合理的进行调配、管理这些node呢？这就要利用ROS提供给我们的节点管理器master, master在整个网络通信架构里相当于管理中心，管理着各个node。<font color="red">**node首先在master处进行注册，之后master会将该node纳入整个ROS程序中。node之间的通信也是先由master进行“牵线”，才能两两的进行点对点通信。**</font>当ROS程序启动时，第一步首先启动master，由节点管理器处理依次启动node。

## 三、启动master和node

当我们要启动ROS时，首先输入命令:
```
roscore
```
此时ROS master启动，同时启动的还有rosout和parameter server。其中rosout是负责日志输出的一个节点，其作用是告知用户当前系统的状态，包括输出系统的error、warning等等，并且将log记录于日志文件中。parameter server即是参数服务器，它并不是一个node，而是存储参数配置的一个服务器，后文我们会单独介绍。每一次我们运行ROS的节点前，都需要把master启动起来，这样才能够让节点node启动和注册。

masterq启动之后，节点管理器master就开始按照系统的安排协调进行启动具体的节点。节点就是一个进程，只不过在ROS中它被赋予了专用的名字里——node。在第二章我们介绍了ROS的文件系统，我们知道一个package中存放着可执行文件，可执行文件是静态的，当系统执行这些可执行文件，将这些文件加载到内存中，它就成为了动态的node。具体启动node的语句是：
```
rosrun pkg_name node_name
```
通常我们运行ROS，就是按照这样的顺序启动。有时候节点太多，我们会选择用launch文件来启动。Master、Node之间以及Node之间的关系如下图所示：
![master-node](picture/masterandnode.png)

## rosrun和rosnode命令

rosrun命令的详细用法如下：
```
rosrun [--prefix cmd] [--debug] pkg_name node_name [ARGS]
```
rosrun将会寻找package下的名为executable的可执行程序，将可选参数ARGS传入。 例如在gdb下运行ros程序：
```
rosrun --prefix 'gdb -ex run --args' pkg_name node_name
```
rosnode命令的详细作用列表如下：

rosnode命令	|作用
:---:|:---:
rosnode help | rosnode使用帮助信息
rosnode list	|列出当前运行的node信息
rosnode info node_name	|显示出node的详细信息
rosnode kill node_name|	结束某个node
rosnode ping	|测试连接节点
rosnode machine	|列出在特定机器或列表机器上运行的节点
rosnode cleanup	|清除不可到达节点的注册信息

以上命令中常用的为前三个，在开发调试时经常会需要查看当前node以及node信息，所以请记住这些常用命令。如果你想不起来，也可以通过rosnode help来查看rosnode命令的用法。