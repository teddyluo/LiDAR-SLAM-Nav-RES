Turtlebot 2e 导航之`move_base` 参数详解
====

[toc]

ROS的`move_base`正如其名，是用于基座移动的功能包，用于实现基座的移动。为把握`move_base`对于`costmap2D`，`global planner`，`local planner`的调用关系。 
这里采用`turtlebot_navigation`的package 为例进行说明。

## `move_base`的启动代码

启动`move_base`的launch文件内容通常为（以Turtlebot 2e为例）：
- 在`turtlebot_apps/turtlebot_navigation/launch/amcl_demo.launch`的启动代码为：

```xml
  <!-- Move base -->
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/$(arg 3d_sensor)_costmap_params.yaml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml">
    <arg name="custom_param_file" value="$(arg custom_param_file)"/>
  </include>
```

- 在`turtlebot_apps/turtlebot_navigation/launch/gmapping_demo.launch`中的启动代码为：

```xml
  <!-- Move base -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml"/>
```

可见建图的过程中采用的是默认参数，而导航过程考虑了3D sensor的传感器特征。以下看看各种传感器的`costmap_param.yaml`定义：
```bash
$ ls -al                               
total 48
lrwxrwxrwx 1 teddyluo teddyluo   10 5月  15 20:43 astra_costmap_params.yaml -> dummy.yaml
lrwxrwxrwx 1 teddyluo teddyluo   10 5月  15 20:43 asus_xtion_pro_costmap_params.yaml -> dummy.yaml
lrwxrwxrwx 1 teddyluo teddyluo   10 5月  15 20:43 asus_xtion_pro_offset_costmap_params.yaml -> dummy.yaml
-rw-rw-r-- 1 teddyluo teddyluo 1621 5月  15 20:43 costmap_common_params.yaml
-rw-rw-r-- 1 teddyluo teddyluo   57 5月  15 20:43 dummy.yaml
-rw-rw-r-- 1 teddyluo teddyluo 2262 5月  15 20:43 dwa_local_planner_params.yaml
-rw-rw-r-- 1 teddyluo teddyluo  405 5月  15 20:43 global_costmap_params.yaml
-rw-rw-r-- 1 teddyluo teddyluo 1730 5月  15 20:43 global_planner_params.yaml
lrwxrwxrwx 1 teddyluo teddyluo   10 5月  15 20:43 kinect_costmap_params.yaml -> dummy.yaml
-rw-rw-r-- 1 teddyluo teddyluo  394 5月  15 20:43 local_costmap_params.yaml
-rw-rw-r-- 1 teddyluo teddyluo  695 5月  15 20:43 lsxxx_costmap_params.yaml
-rw-rw-r-- 1 teddyluo teddyluo 1764 5月  15 20:43 move_base_params.yaml
```
其中 `dummy.yaml`是个空文件。

可以看到，最常用的传感器的`$(arg 3d_sensor)_costmap_params.yaml`并没有定义。可以认为这两条指令是等价的。

## `move_base.launch.xml`的内容
```xml
<!-- 
    ROS navigation stack with velocity smoother and safety (reactive) controller
-->
<launch>
  <include file="$(find turtlebot_navigation)/launch/includes/velocity_smoother.launch.xml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/safety_controller.launch.xml"/>
  
  <arg name="odom_frame_id"   default="odom"/>
  <arg name="base_frame_id"   default="base_footprint"/>
  <arg name="global_frame_id" default="map"/>
  <arg name="odom_topic" default="odom" />
  <arg name="laser_topic" default="scan" />
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/dummy.yaml"/>

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />   
    <rosparam file="$(find turtlebot_navigation)/param/local_costmap_params.yaml" command="load" />   
    <rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot_navigation)/param/dwa_local_planner_params.yaml" command="load" />
    <rosparam file="$(find turtlebot_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find turtlebot_navigation)/param/global_planner_params.yaml" command="load" />
    <rosparam file="$(find turtlebot_navigation)/param/navfn_global_planner_params.yaml" command="load" />
    <!-- external params file that could be loaded into the move_base namespace -->
    <rosparam file="$(arg custom_param_file)" command="load" />
    
    <!-- reset frame_id parameters using user input data -->
    <param name="global_costmap/global_frame" value="$(arg global_frame_id)"/>
    <param name="global_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="local_costmap/global_frame" value="$(arg odom_frame_id)"/>
    <param name="local_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="DWAPlannerROS/global_frame_id" value="$(arg odom_frame_id)"/>

    <remap from="cmd_vel" to="navigation_velocity_smoother/raw_cmd_vel"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <remap from="scan" to="$(arg laser_topic)"/>
  </node>
</launch>
```

## 速度平滑功能包`velocity_smoother.launch.xml`
`velocity_smoother.launch.xml`为速度平滑包，是一个速度平滑控制器，用来防止robot navigation的速度/转速过快，加速度/快减速过大。smoother主要针对navigation或者其他一些例子。robot有一个命令选择node cmd_vel_mux，它可以防止robot被多个ros app下发的运动命令控制从而出现运动问题。见下图：

![p-cart-eq1.png](./figs/m-velocity-smooth.gif)

`velocity_smoother.launch.xml`的内容为：
```xml
<!-- 
         Velocity smoother
-->
<launch>
  <node pkg="nodelet" type="nodelet" name="navigation_velocity_smoother" args="load yocs_velocity_smoother/VelocitySmootherNodelet mobile_base_nodelet_manager">
    <rosparam file="$(find turtlebot_bringup)/param/defaults/smoother.yaml" command="load"/>
    <remap from="navigation_velocity_smoother/smooth_cmd_vel" to="cmd_vel_mux/input/navi"/>

    <!-- Robot velocity feedbacks; use the default base configuration -->
    <remap from="navigation_velocity_smoother/odometry" to="odom"/>
    <remap from="navigation_velocity_smoother/robot_cmd_vel" to="mobile_base/commands/velocity"/>
  </node>
</launch>
```
虽然在`move_base`脚本中启动，但smoother是一个`nodelete`，被`moibile_base_nodelete_manager`加载。其中`smoothrt.yaml`参数配置文件更是在`turtlebot_bringup` package下面。最后将节点中的发布的topic “`navigation_velocity_smoother/smooth_cmd_vel`” 重新映射成 “`cmd_vel_mux/input/navi`”，这样`cmd_vel_mux`就可以订阅到`smoother`发布的主题。

### 订阅的主题
```bash

~raw_cmd_vel (geometry_msgs/Twist)：输入的速度命令
~odometry (nav_msgs/Odometry):接收里程计信息，确保下发的运动命令没有大的跳跃。根据feedback参数配置
~robot_cmd_vel (geometry_msgs/Twist):接受robot 命令信息，确保下发的运动命令没有大的跳跃。根据feedback参数配置
```

### 发布的主题

```sh
~smooth_cmd_vel (geometry_msgs/Twist):发出来的命令，通常会将其remap到cmd_vel_mux订阅的运动命令topic上
```

### 参数
```bsh

~accel_lim_v (double):强制必须设置，线性加速度最大值
~accel_lim_w (double):强制必须设置，角加速度最大值
~speed_lim_v (double):强制必须设置，线速度最大值
~speed_lim_w (double):强制必须设置，角速度最大值
~decel_factor (double, default: 1.0):加减速比，对于惯性大的机器人
~frequency (double, default: 20.0):输出速度频率。不论输入命令的频率。必要时插值
~robot_feedback (int, default: 0):速度反馈(0 - none, 1 - odometry, 2 - end robot commands). 
```
### 参数配置文件`smoother.yaml`

```xml
# Default parameters used by the yocs_velocity_smoother module.
# This isn't used by minimal.launch per se, rather by everything
# which runs on top.
 
# Mandatory parameters
speed_lim_v: 0.8
speed_lim_w: 5.4
 
accel_lim_v: 1.0 # maximum is actually 2.0, but we push it down to be smooth
accel_lim_w: 2.0
 
# Optional parameters
frequency: 20.0
decel_factor: 1.5
 
# Robot velocity feedback type:
#  0 - none (default)
#  1 - odometry
#  2 - end robot commands
robot_feedback: 2
```

### 其他

1. 除了frequency其它参数都是动态可配置的

2. 如果有一个恒定的旋转半径，线/角速度会更加平滑

3. 输入的最后一个topic信息不为0时(命令下发node 崩溃/不活跃)，为了保证robot不出现意外，会在最后加一个0速度。

4. 使用yocs_cmd_vel_mux进行机器人速度控制切换，可参考以下博文：
https://www.cnblogs.com/21207-iHome/p/8228356.html

5. 关于velocity smooth算法，有篇比较好的综述文件：

Ravankar A, Ravankar A, Kobayashi Y, et al. Path smoothing techniques in robot navigation: State-of-the-art, current and future challenges[J]. Sensors, 2018, 18(9): 3170.

## 安全驾驶`safety_controller.launch.xml`

```xml
<!-- 
    Safety controller
-->
<launch>
  <node pkg="nodelet" type="nodelet" name="kobuki_safety_controller" args="load kobuki_safety_controller/SafetyControllerNodelet mobile_base_nodelet_manager">
    <remap from="kobuki_safety_controller/cmd_vel" to="cmd_vel_mux/input/safety_controller"/>
    <remap from="kobuki_safety_controller/events/bumper" to="mobile_base/events/bumper"/>
    <remap from="kobuki_safety_controller/events/cliff" to="mobile_base/events/cliff"/>
    <remap from="kobuki_safety_controller/events/wheel_drop" to="mobile_base/events/wheel_drop"/>
  </node>
</launch>
```
类似于`velocity_smoother`, `safety_controller`亦是一个notelet, 通过`mobile_base_nodelet_manager`加载。

ROS Wiki：http://wiki.ros.org/kobuki_safety_controller

`kobuki_safety_controller`可确保Kobuki的安全运行。 SafetyController可跟踪保险杠碰撞、悬崖和车轮跌落事件。 如果是前两个事件，则底卒Kobuki会被命令往回回。 若是后一种事件，Kobuki将停止。 `kobuki_safety_controller`控制器可以通过选项开启/禁用。 也可以重置安全状态（按下保险杠等）。 对操作的警告：很危险！ 所以一般情况下保持默认值即可。

`kobuki_safety_controller`控制器通常与Kobuki的最小应用配置(minimal.launch)或上层应用程序一起使用，例如：Kobuki随机行走、TurtleBot导航等。

## 坐标系的定义
ROS最常用的坐标系为`map`，`odom`，`base_link`，`base_laser`坐标系。这些坐标系无论是在AMCL或是Gmapping中都要用到的。

- `map`:地图坐标系，顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。

- `base_link`:机器人本体坐标系，与机器人中心重合，坐标系原点一般为机器人的旋转中心。

- `base_footprint`坐标系原点为`base_link`原点在地面的投影，有些许区别（z值不同）。

- `odom`：里程计坐标系，这里要区分开odom topic，这是两个概念。一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。但是两者也有关系，odom topic 转化的位姿矩阵是odom-->base_link的tf关系。
   
   这时可有会有疑问，odom和map坐标系是不是重合的？可以很肯定的告诉你，机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.

- `base_laser`:激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的。

参考：http://www.ros.org/reps/rep-0105.html

### 如何理解`map`、`odom`、`base_link`三者关系
- 世界坐标(map)

该map坐标系是一个世界固定坐标系，其Z轴指向上方。相对于map坐标系的移动平台的姿态，不应该随时间显著移动。map坐标是不连续的，这意味着在map坐标系中移动平台的姿态可以随时发生离散的跳变。

典型的设置中，定位模块基于传感器的监测，不断的重新计算世界坐标中机器人的位姿，从而消除偏差，但是当新的传感器信息到达时可能会跳变。

map坐标系作为长期的全局参考是很有用的，但是跳变使得对于本地传感和执行器来说，其实是一个不好的参考坐标。

- 里程计坐标系(odom)

odom 坐标系是一个世界固定坐标系。在odom 坐标系中移动平台的位姿可以任意移动，没有任何界限。这种移动使得odom 坐标系不能作为长期的全局参考。然而，在odom 坐标系中的机器人的姿态能够保证是连续的，这意味着在odom 坐标系中的移动平台的姿态总是平滑变化，没有跳变。

在一个典型设置中，odom 坐标系是基于测距源来计算的，如车轮里程计，视觉里程计或惯性测量单元。

odom 坐标系作为一种精确，作为短期的本地参考是很有用的，但偏移使得它不能作为长期参考

- 基座标(base_link)

该base_link坐标刚性地连接到移动机器人基座。base_link可以安装在基座中的任意方位；对于每个硬件平台，在基座上的不同地方都会提供一个明显的参考点。

坐标之间的关系： 在机器人系统中，我们使用一棵树来来关联所有坐标系，因此每个坐标系都有一个父坐标系和任意子坐标系，如下：

map --> odom --> base_link
世界坐标系是odom坐标系的父，odom坐标系是base_link的父。虽然直观来说，map和odom应连接到base_link，这是不允许的，因为每坐标系只能有一个父类。

坐标系权限
odom到base_link的转换是由里程计源计算和发布的。然而，定位模块不发布map到base_link的转换(transform)。相反，定位模块先接收odom到base_link的 transform，并使用这个信息发布map到odom的transform。

通俗理解
odom和map坐标系在机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。那map-->odom的tf就是在一些校正传感器合作校正的package比如gmapping会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map–->odom的tf就是

![p-cart-eq1.png](./figs/m-tf-relationship.png)

>The map frame is the parent of odom, and odom is the parent of base_link. Although intuition would say that both map and odom should be attached to base_link, this is not allowed because each frame can only have one parent.**

## `move_base`节点定义
 
 ```xml
 <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" /> #通用的costmap参数定义(全局空间)
    <rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" /> #通用的costmap参数定义(局部空间)
    <rosparam file="$(find turtlebot_navigation)/param/local_costmap_params.yaml" command="load" /> #本地的costmap参数定义 
    <rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" /> #全局costmap参数定义
    <rosparam file="$(find turtlebot_navigation)/param/dwa_local_planner_params.yaml" command="load" /> #dwa本地规划参数定义
    <rosparam file="$(find turtlebot_navigation)/param/move_base_params.yaml" command="load" /> #底盘移动启动文件
    <rosparam file="$(find turtlebot_navigation)/param/global_planner_params.yaml" command="load" /> #全局规划参数定义
    <rosparam file="$(find turtlebot_navigation)/param/navfn_global_planner_params.yaml" command="load" /> #navfn全局规划参数定义
    <!-- external params file that could be loaded into the move_base namespace -->
    <rosparam file="$(arg custom_param_file)" command="load" />
    
    <!-- reset frame_id parameters using user input data -->
    <param name="global_costmap/global_frame" value="$(arg global_frame_id)"/>
    <param name="global_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="local_costmap/global_frame" value="$(arg odom_frame_id)"/>
    <param name="local_costmap/robot_base_frame" value="$(arg base_frame_id)"/>
    <param name="DWAPlannerROS/global_frame_id" value="$(arg odom_frame_id)"/>

    <remap from="cmd_vel" to="navigation_velocity_smoother/raw_cmd_vel"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <remap from="scan" to="$(arg laser_topic)"/>
  </node>
```
**注：`#`后表示注释。**

首先，代价地图的定义，包含全局代价地图和局部代价地图：
```xml
/param/costmap_common_params.yaml" command="load" ns="global_costmap"   
/param/costmap_common_params.yaml" command="load" ns="local_costmap" 
/param/local_costmap_params.yaml" command="load" 
/param/global_costmap_params.yaml" command="load"  #全局costmap参数定义
```
其次，加载了规划器的参数，
- DWA局部规划器的参数`dwa_local_planner_params.yaml`
- 全局规划器`global_planner`的参数`global_planner_params.yaml` `navfn_global_planner_params.yaml`
- 自定义参数`$(arg custom_param_file)`，由前面可知是一个空文件

最后，定义了坐标系：
- `global_costmap/global_frame` --> `map` 全局代价地图中全局坐标系为map
- `global_costmap/robot_base_frame` --> `base_footprint` 全局代价地图中机器人本体坐标系为`base_footprint`
- `local_costmap/global_frame` --> `odom` 局部代价的全局坐标系为odom
- `local_costmap/robot_base_frame` --> `base_footprint`  局部代价机器人本体坐标系为`base_footprint`
- `DWAPlannerROS/global_frame_id` --> `odom` DWA局部规划器中全局坐标系为`odom`

三个topic的remap:
- `cmd_vel` --> `navigation_velocity_smoother/raw_cmd_vel` 将速度话题`cmd_vel`remap为`navigation_velocity_smoother/raw_cmd_vel`
- `odom` --> `odom` `odom` remap为 `odom` (写得能通用，因为可以在前面改变topic的name)
- `scan` --> `scan` `scan` remap为 `scan`

**注：变量名称**
```xml
  <arg name="odom_frame_id"   default="odom"/>
  <arg name="base_frame_id"   default="base_footprint"/>
  <arg name="global_frame_id" default="map"/>
  <arg name="odom_topic" default="odom" />
  <arg name="laser_topic" default="scan" />
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/dummy.yaml"/>
```
 理解为
 - 参数名`odom_frame_id`的默认值为`odom` (`odom`坐标系)
 - 参数名`base_frame_id`的默认值为`base_footprint` (`base_footprint`坐标系)
 - 参数名`global_frame_id`的默认值为`map` (`map`坐标系)
 - 参数名`odom_topic`的默认值为`odom` (`odom` topic)
 - 参数名`laser_topic`的默认值为`scan` (`scan` topic)
 - 参数名`custom_param_file`的默认值为`$(find turtlebot_navigation)/param/dummy.yaml`


### `turtlebot_ws/src/turtlebot_apps/turtlebot_navigation`目录的文件树结构

``` bash
├── CHANGELOG.rst
├── CMakeLists.txt
├── env-hooks                                                          #钩子
│   └── 25.turtlebot-navigation.sh.em                                  #设置一些稳定的默认值，目前是导出TURTLEBOT_MAP_FILE，使用默认的地图。
├── laser                                                              #雷达相关设置
│   ├── costmap_common_params.yaml                                     #costmap的通用参数
│   ├── laser_amcl_demo.launch                                         #激光amcl启动文件
│   ├── laser_gmapping_demo.launch                                     #激光gmapping启动文件
│   └── move_base_laser.launch                                         #带激光底盘移动启动文件
├── launch                                                             #启动目录
│   ├── amcl_demo.launch                                               #amcl启动文件     
│   ├── gmapping_demo.launch                                           #gmapping启动文件
│   ├── graveyard
│   │   └── graveyard_bump_navi_demo.launch                            #
│   └── includes                                                       #启动文件的子模块
│       ├── amcl                                                       #即时定位
│       │   ├── amcl.launch.xml                                        #即时定位核心启动文件
│       │   ├── astra_amcl.launch.xml -> amcl.launch.xml               #使用astra进行定位
│       │   ├── asus_xtion_pro_amcl.launch.xml -> amcl.launch.xml      #使用asus_xtion_pro进行定位
│       │   ├── asus_xtion_pro_offset_amcl.launch.xml -> amcl.launch.xml #使用asus_xtion_pro_live进行定位
│       │   ├── kinect_amcl.launch.xml -> amcl.launch.xml                #使用kinect进行定位
│       │   └── r200_amcl.launch.xml                                     #使用r200进行定位
│       ├── gmapping                                                             #实时建图
│       │   ├── astra_gmapping.launch.xml -> gmapping.launch.xml                 #使用astra建图 
│       │   ├── asus_xtion_pro_gmapping.launch.xml -> gmapping.launch.xml        #使用asus_xtion_pro建图
│       │   ├── asus_xtion_pro_offset_gmapping.launch.xml -> gmapping.launch.xml #使用asus_xtion_pro_live建图
│       │   ├── gmapping.launch.xml                                              #实际建图核心启动文件
│       │   ├── kinect_gmapping.launch.xml -> gmapping.launch.xml                #使用kinect建图 
│       │   └── r200_gmapping.launch.xml                                         #使用r200建图
│       ├── move_base.launch.xml                                                 #底盘移动启动文件
│       ├── safety_controller.launch.xml                                         #安全控制启动文件
│       └── velocity_smoother.launch.xml                                         #速度平滑启动文件
├── maps                                                                         #地图目录
│   ├── willow-2010-02-18-0.10.pgm
│   └── willow-2010-02-18-0.10.yaml
├── package.xml
├── param                                                                        #参数目录
│   ├── astra_costmap_params.yaml -> dummy.yaml                                  #astra的costmap参数定义,指向空文件
│   ├── asus_xtion_pro_costmap_params.yaml -> dummy.yaml                         #asus_xtion_pro的costmap参数定义,指向空文件
│   ├── asus_xtion_pro_offset_costmap_params.yaml -> dummy.yaml                  #asus_xtion_pro_live的costmap参数定义,指向空文件
│   ├── costmap_common_params.yaml                                               #通用的costmap参数定义
│   ├── dummy.yaml                                                               #空文件,没有自定义的参数设置
│   ├── dwa_local_planner_params.yaml                                            #dwa本地规划参数定义
│   ├── global_costmap_params.yaml                                               #全局costmap参数定义
│   ├── global_planner_params.yaml                                               #全局规划参数定义
│   ├── kinect_costmap_params.yaml -> dummy.yaml                                 #kinect的costmap参数定义,指向空文件
│   ├── local_costmap_params.yaml                                                #本地的costmap参数定义
│   ├── move_base_params.yaml                                                    #底盘移动参数定义
│   ├── navfn_global_planner_params.yaml                                         #navfn全局规划参数定义
│   └── r200_costmap_params.yaml                                                 #r200的costmap参数定义
└── src                                                                          #源码目录
    └── laser_footprint_filter.cpp                                               #激光过滤footprint源码, 订阅/scan话题,过滤后,重发布/scan_filtered话题
```

下面我们先看看环境代价地图的定义，再理解规划器的设置，最后看看move_base启动本身。

## 全局代价地图的设置
全局代价地图的设置主要有两个文件：
```xml
<rosparam file="$(find turtlebot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
<rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" />
```
其中，`costmap_common.yaml`为costmap的通用参数设置,定义为
```yaml
max_obstacle_height: 0.60  # assume something like an arm is mounted on top of the robot

# Obstacle Cost Shaping (http://wiki.ros.org/costmap_2d/hydro/inflation)
robot_radius: 0.20  # distance a circular robot should be clear of the obstacle (kobuki: 0.18)
# footprint: [[x0, y0], [x1, y1], ... [xn, yn]]  # if the robot is not circular

map_type: voxel

obstacle_layer:
  enabled:              true
  max_obstacle_height:  0.6
  origin_z:             0.0
  z_resolution:         0.2
  z_voxels:             2
  unknown_threshold:    15
  mark_threshold:       0
  combination_method:   1
  track_unknown_space:  true    #true needed for disabling global path planning through unknown space
  obstacle_range: 2.5
  raytrace_range: 3.0
  origin_z: 0.0
  z_resolution: 0.2
  z_voxels: 2
  publish_voxel_map: false
  observation_sources:  scan bump
  scan:
    data_type: LaserScan
    topic: scan
    marking: true
    clearing: true
    min_obstacle_height: 0.25
    max_obstacle_height: 0.35
  bump:
    data_type: PointCloud2
    topic: mobile_base/sensors/bumper_pointcloud
    marking: true
    clearing: false
    min_obstacle_height: 0.0
    max_obstacle_height: 0.15
  # for debugging only, let's you see the entire voxel grid

#cost_scaling_factor and inflation_radius were now moved to the inflation_layer ns
inflation_layer:
  enabled:              true
  cost_scaling_factor:  5.0  # exponential rate at which the obstacle cost drops off (default: 10)
  inflation_radius:     0.5  # max. distance from an obstacle at which costs are incurred for planning paths.

static_layer:
  enabled:              true
  

```