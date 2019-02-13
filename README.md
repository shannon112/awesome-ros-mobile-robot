## awesome-ros-mobile-robot
This repository provides some useful resources and informations about **mobile robot(AGV,AMR)** research based on **ROS**. It would not contain high level application, but focus on basic function of mobile robot(and more at **navigation**). (including both **Chinese** and **English** materials)  
content:  
* 0.Robotics
* 1.ROS
* 2.Robot platform
* 3.Robot sensor
* 4.SLAM
* 5.Map
* 6.Localization
* 7.Sensor fusion
* 8.Navigation (move_base)
* 9.Navigation (advanced)
* 10.Others (Non-tech)
  * (1)Famous robotics company  
  * (2)Famous robotics conference&journal  
  * (3)Famous robotics competition in Taiwan  
  * (4)Famous ros organizations & activities  
  * (5)Famous ros relative people  

# 0.Robotics
Books:
```
"Multiple View Geometry in Computer Vision", Richard Hartley, Andrew Zisserman
"Probabilistic Robotics", Sebastian Thrun 
"視覺 SLAM 十四講：從理論到實踐", 高翔
```
Courses:
```
"Robot Mapping", Cyrill Stachniss: http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/
"機器人學一 (Robotics (1))", 林沛群: https://www.coursera.org/learn/robotics1
```
Papers:
```
ieee xplore
arxiv
google
```

# 1.ROS
ROS blogs:  
```
半閒居士：https://www.cnblogs.com/gaoxiang12/
MR.POJENLAI: https://pojenlai.wordpress.com/
```
Books:
```
"ROS by Example", python, Patrick Goebel
"Mastering ROS for Robotics Programming", C++, Lentin Joseph
"Learning ROS for Robotics Programming", C++, Enrique Fernandez
"Programming Robots with ROS: A Practical Introduction to...", Morgan Quigley 
```
more:
```
https://hackmd.io/s/B1fv26WBz#
```

# 2.Robot platform
holomic vs. non-holomic
```
Comparison: https://www.evernote.com/l/ATuaHlX8moZHApQrFpCNVYR4SlRPo8Tz53Y
Caster wheel: https://en.wikipedia.org/wiki/Caster
Mecanum wheel: https://en.wikipedia.org/wiki/Mecanum_wheel
Omni wheel: https://en.wikipedia.org/wiki/Omni_wheel
```
race car project  
```
MIT: https://mit-racecar.github.io
Penn: http://f1tenth.org/ [without slam, NAV]
UCB: http://www.barc-project.com/projects/ [without laser] 
Georgia Tech: https://github.com/AutoRally [for outdoor]
Taiwan Hypharos: https://github.com/Hypha-ROS/hypharos_racecar
```
ros robots  
```
https://robots.ros.org/
```
processing unit:  
```
Raspberry Pi
NVIDIA Jetson TX1
NVIDIA Jetson TX2
```
motor & controller:  
```
Faulhaber
Maxon
```

# 3.Robot sensor
rgbd-camera:  
lidar(laser scanner):  
imu:  

# 4.SLAM

# 5.Map

# 6.Localization

# 7.Sensor fusion (for odom-to-base_link or map-to-base_link)

# 8.Navigation (move_base compatible, nav_core supported)
local_planner:  
global_planner:  
RecoveryBehavior:  

# 9.Navigation (social aware, novel research)


# 10.Others (Non-tech)
## (1) Famous robotics company
Research center:  
```Toyota Research Institute(TRI), Microsoft Research, Google AI```  
Manipulator:  
```ABB, FANUC, KUKA, YASKAWA, TECHMAN_ROBOT, HIWIN, Universal_robots  ```  
Service robot:  
```willow_garage, softbank_robotics  ```  
Humanoid:  
```boston_dynamics, softbank_robotics  ```  
AGV:  
```omron_robotics, clearpath_robotics&OTTO_Motors, amazon robotics(Kiva System), Yujin Robotics, ROBOTIS, fetch_robotics```  
Drone:  
```Dji, Tello```  
ROS2.0:  
```ADLINK ```   

## (2) Famous robotics conferences & journals
Top conference: ```ieee_IROS, ieee_ICRA```  
Top journal: ```ieee_Transactions_on_Robotics_and_Automation```  
Minor conference:```ieee_IECON, ieee_SII, ieee_ISIE, ieee_ICIT, ieee_ICPS```  

## (3) Famous robotics competition
Global:
```
DARPA Robotics Challenge: https://en.wikipedia.org/wiki/DARPA_Robotics_Challenge
```
In Taiwan:
```
SKS
PMC
HIWIN
```

## (4) Famous ros organizations & activities
organizations
```
OSRF
OSRC
ROS.Taiwan
ROS.Taipei
```
activities: 
```
ROScon
ROS summer school
```

## (5) Famous ros relative people & robotics labs
robotics labs
```
宾夕法尼亚大学 GRASP 实验室
坦福大学人工智能实验室(SAIL)
斯坦福大学人工智能实验室吴恩 达教授
斯坦福大学机器人实验室，由基南·威罗拜克和埃里克·博格开发的PR1机 器人。
MIT 人工智能实验室
MIT 媒体实验室
南加州大 学机器人实验室
```
people
```
斯科特·哈森(Scott Hassan) - 创立了 eGroup 公司，stanford研究生中途退学，Willow garage founder，後來創立beam Suitable Robotics
史蒂夫·库辛斯(Steve Cousins) - Willow garage CEO，後來創立Savioke Robotics
埃里克·博格(Eric Berger) - PR1 , stanford , STAIR參與者, PR2之父
和基南·威罗拜克(Keenan Wyrobek) - PR1, stanford, PR2之父
蕾拉·高山(Leila Takayama) - PR2外型設計，非工程專業
肯尼斯·萨里斯伯里(Kenneth Salisbury) - 兩個做PR1學生的指導老師
吳恩達(Andrew Yan-Tak Ng) - stanford教授 人工智能實驗室 SAIL
摩根·奎格利(Morgan Quigley) - Andrew底下的博士學生，STAIR架構的開發，ROS之父，OSRF首席架構師 SAIL，rosbuild
布莱恩·格基 (Brian Gerkey，OSRF CEO 創始人) USC畢業 SAIL，rosbuild，catkin
罗伯特·弗里德曼(Roberta Friedman， OSRF CFO)
凯特·考尼(Nate Koenig， OSRF CTO)
思特菲·派姬(Steffi Paepcke) OSRF co-founder 人機互動方面專業，turtlesim小烏龜設計者
乔许·埃林森(Josh Ellingson) 设计了几乎所有的 ROS 吉祥物，PR2 宣传形象，还有ROSCon的海报。
埃坦•马德-爱泼斯坦(Eitan Marder- Eppstein) navigation stack的作者，創立hiDOF，被google收購後進入Tango組
印度大叔萨钦·启德(Sachin Chitta) MoveIt!的创始人，創立Kinema 机器人公司
戴夫·赫什博格(Dave Hershberger) rviz製作者之一，加入Kinema
大卫·高索(David Gossow) rviz製作者之一，加入hiDOF，加入tango，創立Lucid
乔西·浮士德(Josh Faust) rviz製作者之一，加入suitable，加入magic leap
乔纳森·波仁(Jonathan Bohren) SMACH的開發者，加入honeybee robotics
凯仁·萧(Kaijen Hsiao) manipulation 软件包開發，加入bosch，創立Kuri
马泰·乔卡列(Matei Ciocarlie) manipulation 软件包開發，EigenGrasp開發
文森特·莱保德(Vincent Rabaud) 負責對kinect的驅動
图利·弗特(Tully Foote) 負責對kinect的驅動
莱度·茹苏(Radu B. Rusu) PCL的開發者
盖里·布拉德斯基(Gary Bradski) OpenCV 是他在 Intel 工作期间开发的一个开源计算机视觉库，於車庫早期加入， 創立Industrial Perception, Inc.(IPI)
帕特里克(Patrick Mihelich) ROS 软件包 vision_opencv 的开发者
詹姆斯 (James Bowman) ROS 软件包 vision_opencv 的开发者
鲁本·斯密斯(Ruben Smits) 在车库实习，将 OROCOS 于 ROS 集成，KDL開發者，創立Intermodalics，開發Pick-it3D
艾文·艾尔特比列恩(Erwin Aertbelien) KDL開發者
丹尼尔·斯托尼(Daniel Stonier) win_ros 是由韩国柳真机器人公司领导开发的。領導團隊開發Koboki
安德鲁·霍华德(Andrew Howard) Gazebo 在南加州大学，开发的一个开源的机器人仿真环境，教授
凯特·考尼(Nate Koenig) Gazebo 在南加州大学，开发的一个开源的机器人仿真环境，博士生，連同前面創立了OSRF
特洛伊·斯特拉斯海姆(Troy Straszheim) rosbuild，創立Industrial Perception, Inc.(IPI) catkin
莫腾·谢尔高(Morten Kjaergaard) catkin
德克·托马斯(Dirk Thomas) catkin
图利·弗特(Tully Foote) turtlebot from irobot and kinect 搞很多無人車，现在是ＯＳＲＦ ROS 开发部门的负责人。
麦罗尼·威瑟(Melonee Wise) turtlebot from irobot and kinect ，platformBot 車庫第二號員工 創立Unbounded Robotics，創立Fetch Robotics
```
