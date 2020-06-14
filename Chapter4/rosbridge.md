# Rosbridge

Rosbridge是一个用在ROS系统和其他系统之间的一个功能包,就像是它的名字一样,起到一个"桥梁"的作用,使得ros系统和其他系统能够进行交互.Rosbridge为非ROS程序提供了一个JSON API,有许多与Rosbridge进行交互的前端，包括一个用于Web浏览器交互的WebSocket服务器。Rosbridge_suite是一个包含Rosbridge的元程序包，用于Rosbridge的各种前端程序包（如WebSocket程序包）和帮助程序包。

## 协议和实现
Rosbridge主要包含两部分内容:协议(Potocol)和实现(Implementation)
### 协议
Ｒosbridge Protocol提供了非ROS程序与ROS通信的具体的格式规范，规范基于JSON格式,包括订阅topic，发布message,调用server，设置参数，压缩消息等等．例如订阅topic的格式规范如下：
```
{ "op": "subscribe",
    "topic": "/cmd_vel",
    "type": "geometry_msgs/Twist"
}
```

此规范与所用的编程语言和传输方式无关，任何可以发送JSON格式的语音和传输方式都可以Rosbridge protocol进行交流，并且与ROS进行交互．
### 实现
Rosbridge_suite元程序包是实现Ｒosbridge　Protocol并提供WebSocket传输层的包的集合。
这些软件包包括：
* Rosbridge_library : 核心rosbridge软件包。Rosbridge_library负责获取JSON字符串并将命令发送到ROS，反过来接收处理ROS发过来的信息，将之转换为JSON字符串，并将结果转交给非ROS程序。
* rosapi : 通过服务调用来访问某些ROS操作，这些服务通常为ROS客户端库保留的服务．这些操作包括获取和设置参数，获取主题列表等等。
* rosbridge_server : 虽然Rosbridge_library提供JSON到ROS转换，但它将传输层留给其他人。Rosbridge_server提供了一个WebSocket连接，所以浏览器可以与ROS“交谈”。Roslibjs是一个浏览器的JavaScript库，可以通过rosbridge_server与ROS进行交流。
源码

## 安装与使用

### 安装
Rosbridge是基于ROS的，首先要确保自己正确的安装完成了ROS之后可以启动终端执行命令：
```
sudo apt-get install ros- <rosdistro> -rosbridge-server
```
中间的rosdistro为自己的ROS版本，依照自己的版本进行安装．
### 使用
关于更深入的使用，可以参考本课程的视频课程，简单的入门使用可以参考链接如下：
[链接](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)