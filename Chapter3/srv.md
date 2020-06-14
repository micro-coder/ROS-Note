# srv文件

<font color="red">一个srv文件描述一项服务。它包含两个部分：请求和响应</font>
   
msg文件存放在package的msg目录下，srv文件则存放在srv目录下。

类似msg文件，srv文件是用来描述服务(service)数据类型的，service通信的数据格式就定义在*.srv格式的文件中。它声明了一个服务类型，包括请求(request)和响应（reply）两部分。其格式声明如下，举例：

msgs_demo/srv/DetectHuman.srv
```
bool start_detect     #client请求服务格式
---                   #分界线
my_pkg/HumanPose[] pose_data    #server返回数据格式
```
msgs_demo/msg/HumanPose.msg
```
std_msgs/Header header
string uuid
int32 number_of_joints
my_pkg/JointPose[] joint_data
```
msgs_demo/msg/JointPose.msg
```
string joint_name
geometry_msgs/Pose pose
float32 confidence
```

以DetectHUman.srv文件为例，该服务例子取自OpenNI的人体检测ROS软件包。它是用来查询当前深度摄像头中的人体姿态和关节数的。srv文件格式很固定，第一行是请求的格式，中间用---隔开，第三行是应答的格式。在本例中，请求为是否开始检测，应答为一个数组，数组的每个元素为某个人的姿态（HumanPose）。而对于人的姿态，其实是一个msg。<font color="red">所以msg可以嵌套msg。srv也可以嵌套msg，但不能嵌套srv。</font>

## 操作命令

具体的操作指令如下表：

rossrv 命令|	作用
:---:|:---:
rossrv show	|显示服务描述
rossrv list	|列出所有服务
rossrv md5|	显示服务md5sum
rossrv package|	列出包中的服务
rossrv packages	|列出包含服务的包

## 生成头文件

定义好了msg、srv文件之后，我们写的C++程序还不能直接使用`.msg`或者`.srv`文件内容。所以接下来，我们还需要做一些步骤，使`.msg`或者`.srv`文件变成`.h`头文件。

  1. 修改package.xml文件，添加依赖信息
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
  2. 修改CMakeLists.txt,声明`.msg`或者`.srv`文件信息，也添加依赖
  ```cmake
  find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  )

  add_message_files(
  FILES
  DetectHuman.srv   
  HumanPose.msg 
  JointPos.msg
  )

  catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_subscribe_demo
  CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
  ```
  3. 最后一步，执行编译命令，`.msg`或者`.srv`文件编译后生成文件就是相同文件名的`.h`头文件了
  ```bash
  catkin_make
  ```
  <font color="red">谨记：执行`catkin_make`必须在工作空间的最顶层目录执行。
  编译生成的头文件在 `devel/include/` 的文件里</font>

## 常用srv类型

介绍常见的srv类型及其定义， srv类型相当于两个msg通道，一个发送，一个接收。

1. **AddTwoInts.srv**

  文件位置：自定义srv文件
  ```
  #对两个整数求和，虚线之上是输入量，之下是返回量
  int32 a
  int32 b
  ---
  int32 sum
  ```

2. **Empty.srv**

  文件位置：std_srvs/Empty.srv
  ```
  #代表一个空的srv类型
  ---
  ```

3. **GetMap.srv **

  文件位置:nav_msgs/GetMap.srv
  ```
  #获取地图，注意请求部分为空
  ---
  nav_msgs/OccupancyGrid map
  ```

4. **GetPlan.srv**

  文件位置:nav_msgs/GetPlan.srv
  ```
  #得到一条从当前位置到目标点的路径
  geometry_msgs/PoseStamped start        #起始点
  geometry_msgs/PoseStamped goal        #目标点
  float32 tolerance    #到达目标点的x，y方向的容错距离
  ---
  nav_msgs/Path plan
  ```

5. **SetBool.srv**

  文件位置：std_srvs/SetBools.srv
  ```
  bool data # 启动或者关闭硬件
  ---
  bool success   # 标示硬件是否成功运行
  string message # 运行信息
  ```
6. **SetCameraInfo.srv**

  文件位置:sensor_msgs/SetCameraInfo.srv
  ```
  #通过给定的CameraInfo相机信息，来对相机进行标定
  sensor_msgs/CameraInfo camera_info        #相机信息
  ---
  bool success            #如果调用成功，则返回true
  string status_message    #给出调用成功的细节
  ```

7. **SetMap.srv**

  文件位置：nav_msgs/SetMap.srv
  ```
  #以初始位置为基准，设定新的地图
  nav_msgs/OccupancyGrid map
  geometry_msgs/PoseWithCovarianceStamped initial_pose
  ---
  bool success
  TalkerListener.srv
  #文件位置: 自定义srv文件
  ---
  bool success   # 标示srv是否成功运行
  string message # 信息，如错误信息等
  ```

8. **Trigger.srv**

  文件位置:std_srvs/Trigger.srv
  ```
  ---
  bool success   # 标示srv是否成功运行
  string message # 信息，如错误信息等
  ```