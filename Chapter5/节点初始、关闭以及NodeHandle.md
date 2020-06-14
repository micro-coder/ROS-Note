# 节点初始、关闭以及NodeHandle

当执行一个ROS程序，程序被加载到内存中，就成为了一个进程。在ROS里叫做节点。每一个ROS的节点尽管功能不同，但都有必不可少的一些步骤，比如初始化、销毁，需要通行的场景通常都还需要节点的句柄。 这一篇我们来学习Node(节点)最基本的一些操作。

## 初始化节点

对于一个C++写的ROS程序，之所以它区别于普通C++程序，是因为代码中做了两层工作：  
 1. 调用了ros::init()函数，从而初始化节点的名称和其他信息，一般我们ROS程序一开始都会以这种方式开始。
 2. 创建ros::NodeHandle对象，也就是节点的句柄，它可以用来创建Publisher、Subscriber以及做其他事情。

句柄(Handle)这个概念可以理解为一个“把手”。你握住了门把手，就可以很容易把整扇门拉开，而不必关心门是什么样子。NodeHandle就是对节点资源的描述，有了它你就可以操作这个节点的各种信息和动作，比如为程序提供服务、监听某个topic上的消息、访问和修改param等等。

## 关闭节点

通常我们要关闭一个节点可以直接在终端上按Ctrl+C，系统会自动触发SIGINT句柄来关闭这个进程。  
你也可以通过调用ros::shutdown()来手动关闭节点，但通常我们很少这样做。
以下是一个节点初始化、关闭的例子:
```cpp
#include<ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "your_node_name"); 
    ros::NodeHandle nh;
    //....节点功能
    //....
    ros::spin();//用于触发topic、service的响应队列
    return 0;
}
```
这段代码是最常见的一个ROS程序的执行步骤。通常要先启动节点，获取句柄。而关闭的工作系统自动帮我们完成，如果有特殊需要你也可以自定义。你可能很关心句柄可以用来做些什么，接下来我们来看看NodeHandle常用的成员函数。

## NodeHandle常用成员函数
NodeHandle是Node的句柄，用来对当前节点进行各种操作。  
在ROS中，NodeHandle是roscpp已经定义好的一个类，通过引用 `include <ros/ros.h>` ，我们可以创建这个类，以及使用它的成员函数。  
NodeHandle常用成员函数包括：
```cpp
//创建话题的publisher 
ros::Publisher advertise(const string &topic, uint32_t queue_size, bool latch=false); 
//第一个参数为发布话题的名称
//第二个是消息队列的最大长度，如果发布的消息超过这个长度而没有被接收，那么旧的消息就会出队。(先进先出原则)通常设为一个较小的数即可。
//第三个参数是是否锁存。某些话题并不是会以某个频率发布，比如/map这个topic，只有在初次订阅或者地图更新这两种情况下，/map才会发布消息。这里就用到了锁存。

//创建话题的subscriber
ros::Subscriber subscribe(const string &topic, uint32_t queue_size, void(*)(M));
//第一个参数是订阅话题的名称
//第二个参数是订阅队列的长度，如果受到的消息都没来得及处理，那么新消息入队，旧消息就会出队(先进先出原则)
//第三个参数是回调函数指针，指向回调函数入口地址，以此来处理接收到的消息

//创建服务的server，提供服务
ros::ServiceServer advertiseService(const string &service, bool(*srv_func)(Mreq &, Mres &)); 
//第一个参数是service名称
//第二个参数是服务函数的指针，指向服务函数。指向的函数应该有两个参数，分别接受请求和响应。

//创建服务的client
ros::ServiceClient serviceClient(const string &service_name, bool persistent=false); 
//第一个参数是service名称
//第二个参数用于设置服务的连接是否持续，如果为true，client将会保持与远程主机server的连接，这样后续的请求会快一些。通常我们设为flase

//查询某个参数的值
bool getParam(const string &key, std::string &s); 
bool getParam (const std::string &key, double &d) const；
bool getParam (const std::string &key, int &i) const；
//从参数服务器上获取key对应的值，已重载了多个类型

//给参数赋值
void setParam (const std::string &key, const std::string &s) const；
void setParam (const std::string &key, const char *s) const;
void setParam (const std::string &key, int i) const;
//给key对应的val赋值，重载了多个类型的val
```

可以看出，NodeHandle对象在ROS C++程序里非常重要。各种类型的通信都需要用NodeHandle来创建完成。下面我们具体来看看topic、service和param这三种基本通信方式的写法。