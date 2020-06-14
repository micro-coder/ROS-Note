# moveit!

[官方网站](https://moveit.ros.org/)

2012年，一款叫做moveit!的移动操作软件诞生了，moveit！最初在Willow Garage由Sachin Chitta，Ioan Sucan，Gil E. Jones，Acorn Pooley，Suat Gedikli，Dave Hershberger开发，它融合了研究者在运动规划、操纵、3D感知、运动学、控制和导航方面的最新进展，为操作者提供了一个易于使用的平台，使用它可以开发先进的机器人应用，也被广泛应用于工业，商业，研发和其他领域。由于以上特性，moveit！一跃成为在机器人上最广泛使用的开源操作软件，截止2017年，已经被用于超过65台机器人。
moveit!视频链接
## 使用
moveit!的使用通过为用户提供接口来调用它，包括C++、Python、GUI三种接口。ROS中的
move_group节点充当整合器，整合多个独立组件，提供ROS风格的Action和service。move_group通过ROS topic和action与机器人通讯，获取机器人的位置、节点等状态，获取数据再传递给机器人的控制器。

move_group节点获取到节点状态信息或者机器人变换信息时候，会通过控制器的接口去处理这些信息，比如进行坐标转换、规划场景、3D感知。另外，move_group的结构比较容易扩展，不仅具有独立的能力如抓放，运动规划，也可扩展自公共类，但实际作为独立的插件运行。moveit!系统结构图如下：
![moveIT.png](picture/moveIT.png)