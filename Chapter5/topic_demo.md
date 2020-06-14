# topic_demo

Topic是ROS里一种异步通信的模型。一般是节点间分工明确，有的节点只负责发送，有的节点只负责接收处理。对于绝大多数的机器人应用场景，比如传感器数据收发，速度控制指令的收发，Topic模型是最适合的通信方式。

为了讲明白topic通信的编程思路，我们首先来看topic_demo中的代码,这个程序是一个消息收发的例子：自定义一个类型为gps的消息（包括位置x，y和工作状态state信息），一个node以一定频率发布模拟的gps消息，另一个node接收并处理，算出到原点的距离。

## 第一步 创建gps消息

在代码中，我们会用到自定义类型的gps消息，因此就需要来自定义gps消息，在msg路径下创建gps.msg： 见topic_demo/msg/gps.msg
```
  string state   #工作状态
  float32 x      #x坐标
  float32 y      #y坐标
```
以上就定义了一个gps类型的消息，你可以把它理解成一个C语言中的结构体，类似于
```cpp
  struct gps
  {
      string state;
      float32 x;
      float32 y;
  }
```
注意到了没，gps.msg文件的文件名称就是结构体的名称。
在程序中对一个gps消息进行创建修改的方法和对结构体的操作一样。  
接下来，还有关键的一步：我们要确保msg文件被转换成为C++，Python和其他语言的源代码：  
当你创建完了msg文件，记得修改CMakeLists.txt和package.xml，从而让系统能够编译自定义消息。  
  1. 在package.xml文件中需要改动  
  确保它包含以下两条语句:
  ```xml
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
  ```
  如果没有，添加进去。 注意，在构建的时候，我们只需要"message_generation"。然而，在运行的时候，我们需要"message_runtime"。

  2. 在 CMakeLists.txt文件中需要改动  
  利用find_packag函数，增加对**message_generation**的依赖，这样就可以生成消息了。你可以直接在COMPONENTS的列表里增加message_generation，就像这样：
  ```cmake
    find_package(catkin REQUIRED COMPONENTS
        roscpp 
        rospy 
        std_msgs
        message_generation
    )
  ```
  <font color="red">注：</font>有时候你会发现，即使你没有调用find_package,你也可以编译通过。这是因为catkin把你所有的package都整合在一起。因此，如果其他的package调用了find_package，你的package的依赖就会是同样的配置。但是，在你单独编译时，忘记调用find_package会很容易出错。

  同样，你需要确保你设置了运行时依赖：
  ```cmake
    catkin_package(
      ...
      CATKIN_DEPENDS message_runtime ...
      ...)
  ```
  接着，找到如下代码块:
  ```cmake
    # add_message_files(
    #   FILES
    #   Message1.msg
    #   Message2.msg
    # )
  ```
  去掉注释符号#，用你的.msg文件替代Message*.msg，就像下边这样：
  ```cmake
    add_message_files(
      FILES
      gps.msg
    )
  ```
  接下来，找到如下部分:
  ```cmake
    # generate_messages(
    #   DEPENDENCIES
    # #  std_msgs  # Or other packages containing msgs
    # )
  ```
  去掉注释符号#，并附加上所有你消息文件所依赖的那些含有.msg文件的package（这个例子非常简单，只依赖std_msgs,不用添加roscpp,rospy)，结果如下:
  ```cmake
    generate_messages(
      DEPENDENCIES
      std_msgs
    )
  ```

### 修改后的CMakeLists.txt文件

```cmake
  find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation   #需要添加的地方
  )
  ## Generate messages in the 'msg' folder
  add_message_files(
    FILES 
    gps.msg
  )  
  #catkin在cmake之上新增的命令，指定从哪个消息文件生成
  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
  ) 
  #catkin新增的命令，用于生成消息
  #DEPENDENCIES后面指定生成msg需要依赖其他什么消息，由于gps.msg用到了flaot32这种ROS标准消息类型，因此需要再把std_msgs作为依赖
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES publish_subscribe_demo
    CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
  #  DEPENDS system_lib
  )
```

### 修改后的package.xml文件

`<exec_depend> </exec_depend>`与`<run_depend> </run_depend>`作用相同
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>message_runtime</run_depend>
```

由于增加了新的消息，所以我们需要重新编译我们的package：
```
  # In your catkin workspace
  catkin_make
```
编译完成之后会在工作空间的devel路径下生成gps.msg对应的头文件gps.h。头文件是按照C++的语法规则定义了topic_demo::gps类型的数据。

## 第二步 消息发布节点代码 talker.cpp

<font color="red">注：</font>要在代码中使用自定义消息类型，只要`#include <topic_demo/gps.h>`就行了。

定义完了消息，就可以开始写ROS代码了。通常我们会把消息收发的两端分成两个节点来写，一个节点就是一个完整的C++程序。  
**talker.cpp**
```cpp
  //ROS头文件
  #include <ros/ros.h>
  //自定义msg产生的头文件
  #include <topic_demo/gps.h>

  int main(int argc, char **argv)
  {
    //用于解析ROS参数，第三个参数为本节点名
    ros::init(argc, argv, "talker");

    //实例化句柄，初始化node
    ros::NodeHandle nh;

    //自定义gps msg
    topic_demo::gps msg;
    msg.x = 1.0;
    msg.y = 1.0;
    msg.state = "working";

    //创建publisher
    ros::Publisher pub = nh.advertise<topic_demo::gps>("gps_info", 1);

    //定义消息发布的频率 
    ros::Rate loop_rate(1.0);
    //循环发布msg
    while (ros::ok())
    {
      //以指数增长，每隔1秒更新一次
      msg.x = 1.03 * msg.x ;
      msg.y = 1.01 * msg.y;
      ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
      //以1Hz的频率发布msg
      pub.publish(msg);
      //根据前面定义的频率, sleep 1s
      loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
    }

    return 0;
  } 
```

机器人上几乎所有的传感器，几乎都是按照固定频率发布消息这种方式来传输数据，只是发布频率和数据类型的区别。

## 第三步 消息接受节点代码 listener.cpp

```cpp
  //ROS头文件
  #include <ros/ros.h>
  //包含自定义msg产生的头文件
  #include <topic_demo/gps.h>
  //ROS标准msg头文件
  #include <std_msgs/Float32.h>

  //回调函数
  void gpsCallback(const topic_demo::gps::ConstPtr &msg)
  {  
      //计算离原点(0,0)的距离
      std_msgs::Float32 distance;
      distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
      //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
      ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("gps_info", 1, gpsCallback);
    //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
    ros::spin(); 
    return 0;
  }
```

在topic接收方，有一个比较重要的概念，就是回调(CallBack)。在本例中，回调就是预先给gps_info话题传来的消息准备一个回调函数。事先定义好回调函数的操作，本例中是计算到原点的距离。只有当有消息来时，回调函数才会被触发执行。具体去触发的命令就是ros::spin()，它会反复的查看有没有消息来，如果有就会让回调函数去处理。

<font color="red">注：因此千万不要认为，只要指定了回调函数就会自动触发。你必须使用ros::spin()或者ros::spinOnce()才能真正使回调函数生效</font>.

### 修改CMakeLists.txt文件

在CMakeLists.txt添加以下内容，生成可执行文件
```cmake
  add_executable(talker src/talker.cpp) #生成可执行文件talker
  add_dependencies(talker topic_demo_generate_messages_cpp)
  #表明在编译talker前，必须先编译完成自定义消息
  #必须添加add_dependencies，否则找不到自定义的msg产生的头文件
  target_link_libraries(talker ${catkin_LIBRARIES}) #链接

  add_executable(listener src/listener.cpp ) #生成可执行文件listener
  add_dependencies(listener topic_demo_generate_messages_cpp)
  target_link_libraries(listener ${catkin_LIBRARIES})#链接
```
以上cmake语句告诉catkin编译系统如何去编译生成我们的程序。这些命令都是标准的cmake命令，如果不理解，请查阅cmake教程。
之后经过catkin_make，一个自定义消息+发布接收的基本模型就完成了。

## 第四步 运行方法

打开两个终端:  
一个运行:
```
 rosrun  topic_demo talker 
```
一个运行:
```
rosrun  topic_demo listener 
```