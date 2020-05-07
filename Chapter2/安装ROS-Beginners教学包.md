#  安装ROS-Academy-for-Beginners教学包
这是我找到的算是比较系统的学习ROS入门的资料，目前关于ROS基础的学习资料比较少，所以网上资料也比较零散。  
我参考的这份学习资料对于我这个还没入门的人来说，很珍贵。所以我基本上参考这份资料进行学习，并记录自己在学习过程中遇到的问题和解决方法。  
1. 下载git  
因为我们需要从github上下载源代码，所以需要先安装git工具。ubuntu系统一般会自带git。
```bash
sudo apt install git
```
2. 然后，创建一个文件夹,作为ROS的工作空间workspace，名称随意，我这里命名为**`tutorial_ws`**  
 ```bash
 mkdir  tutorial_ws
 ```
并且在`tutorial_ws`文件夹下再创建一个子文件夹，这个子文件夹的名称不是随意的了，必须为**`src`**
 ```bash
 cd tutorial_ws
 mkdir src
 ```
3. 接下来克隆github上的教学包代码到本地的**`src`**文件夹里
```bash
cd tutorial_ws/src
git clone https://github.com/DroidAITech/ROS-Academy-for-Beginners.git
```

4. 教学代码包还需要一些依赖文件，所以接下来就安装依赖文件
```bash
cd tutorial_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
*<font color="red">**注：**</font>以上命令很重要，缺少依赖将导致教学包无法正常编译和运行*

5. 在开始编译教学代码之前，还需要确保安装的[gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)版本在7.0及7.0以上
使用指令查看一下
```
gazebo -v
```
如果你的gazebo版本低于7.0，则需要升级
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
```
*<font color="red">**注：**</font>最后一行命令最好还是下载 gazebo7，更高版本可能会有其他问题*

6. 准备工作已经完成了，现在可以编译教学代码了
```bash
cd ~/tutorial_ws
catkin_make
source ~/tutorial_ws/devel/setup.bash
```
*<font color="red">**注：**</font>每一次使用**`catkin_make`**编译完成后，必须使用 **`source`** 命令刷新一下工作空间的环境，否则可能找不到工作空间。*
 * 许多时候我们为了打开终端就能够运行工作空间中编译好的ROS程序，我们习惯把**`source ~/tutorial_ws/devel/setup.bash`**命令追加到**`~/.bashrc`**文件中(**`tutorial_ws`**替换为你自己的工作空间名称)。这样每次打开终端，系统就会自动刷新工作空间环境了。你可以通过
 ```bash
 echo "source ~/tutorial_ws/devel/setup.bash" >> ~/.bashrc
 ```
 命令来添加。

7. 编译成功之后，就可以运行本教学配套的仿真程序了
  1. 输入命令：
  ```bash
  roslaunch robot_sim_demo robot_spawn.launch
  ```
  你会看到仿真画面启动，仿真界面中包括了软件博物馆和Xbot机器人模型。  
  *<font color="red">**注：**</font>第一次启动gazebo仿真软件，会比较慢，需要耐心等待一下。*  
  2. 另外打开一个终端，输入命令：
  ```bash
  rosrun robot_sim_demo robot_keyboard_teleop.py
  ```
  将会打开键盘控制程序，可以控制机器人移动。
  聚焦键盘控制终端窗口，按下I，J,L等按键，这样就通过键盘控制机器人移动了。
8. 当完成了上面7个步骤之后，不知不觉中，我们就已经完成了ROS中最常见的 *源码下载->安装依赖->编译->运行* 的流程了。  
在ROS社区有许许多多这样的软件代码包，基本都按照这样的流程来运行。相信你一定可以举一反三。

