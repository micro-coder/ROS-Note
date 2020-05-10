# srv文件

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

以DetectHUman.srv文件为例，该服务例子取自OpenNI的人体检测ROS软件包。它是用来查询当前深度摄像头中的人体姿态和关节数的。srv文件格式很固定，第一行是请求的格式，中间用---隔开，第三行是应答的格式。在本例中，请求为是否开始检测，应答为一个数组，数组的每个元素为某个人的姿态（HumanPose）。而对于人的姿态，其实是一个msg，所以srv可以嵌套msg在其中，但它不能嵌套srv。

## 操作命令

具体的操作指令如下表：

rossrv 命令|	作用
:---:|:---:
rossrv show	|显示服务描述
rossrv list	|列出所有服务
rossrv md5|	显示服务md5sum
rossrv package|	列出包中的服务
rossrv packages	|列出包含服务的包
