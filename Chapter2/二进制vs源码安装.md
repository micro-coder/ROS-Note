# 二进制安装 vs 源代码安装

## 1. 二进制包与源代码包  
我们通过apt方式安装了ROS系统以及相关的软件包，我们也通过下载源码的方式编译安装了一个ROS教学代码包。这是两种常见的软件包安装方式，通常我们的软件包(Package)就可以分为二进制和源代码。
 * 二进制包里面包括了已经编译完成，可以直接运行的程序。通过 **`sudo apt-get install`** 来进行下载和解包（安装），执行完该指令后就可以马上使用了。因此这种方式简单快捷，适合比较固定、无需改动的程序。
 * 而源代码包里是程序的原始代码，下载到你的计算机上必须经过编译，生成可执行的二进制文件之后，方可运行。一些个人开发的程序、第三方修改或者你希望修改的程序都应当通过源代码包的来编译安装。

区别|二进制包|源代码包
:---:|:---:|:---:
下载方式 |	apt-get install/直接下载deb	|git clone/直接下载源代码
ROS包存放位置|	/opt/ros/kinetic/|	通常~/tutorial_ws/src
编译方式|	无需编译	|通过make/cmake/caktin_make
来源|	官方apt软件源|	开源项目、第三方开发者
扩展性|	无法修改|	通过源代码修改
可读性|	无法查看源代码|	方便阅读源代码
优点|	下载简单，安装方便|	源码可修改，便于定制功能
缺点|	无法修改|	编译工具、软件包依赖、版本和参数
应用场景|	基础软件|	需要查看、开发和修改的程序

我们用apt-get安装了ROS及其组件，因此我们不需要编译就可以运行turtlesim程序。对于这些程序，除非我们做操作系统的设计开发才会去下载源码，否则直接用官方提供的ROS软件包；  
而ROS-Academy-for-Beginners以源码呈现，你可以看到每个demo下面的C++源代码。对于这些源文件我们必须caktin_make编译，然后才能运行。
## 2. ROS二进制包的安装
在ROS中，我们可能经常会遇到缺少相关的ROS依赖的问题。有些时候你编译或者运行一些ROS程序，系统会提示找不到XXX功能包，如图所示。  
遇到这样的问题，请先注意阅读错误原因，看看是否有解决方法，也可以Google一下。如果是缺少ROS的依赖，通常可以用以下命令来安装：
```bash
sudo apt-get install ros-kinetic-PACAKGE
```
 将PACKAGE替换为系统提示缺少的软件包，例如
```bash
sudo apt-get install ros-kinetic-slam-gmapping   #GMapping-SLAM算法包
sudo apt-get install ros-kinetic-turtlebot-description  #Turtlebot机器人模型包
```

*<font color="red">**注：**所有APT官方中的ROS功能包都是按照 `ros-` 的形式来命名的。</font>*