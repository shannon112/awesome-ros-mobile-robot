## awesome-ros-mobile-robot
Autonomous Mobile Robots aka. AMR  
Autonomous Mobile Manipulators aka. AMM  
This repository provides some useful resources and informations about **mobile robots (AMR&AMM)** research based on **ROS**. It would not contain high level application, but focus on basic function of mobile robots(and more at **navigation**). (including both **Chinese** and **English** materials)  
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
"Control of Mobile Robots", Magnus Egerstedt: https://www.coursera.org/learn/mobile-robot"
"Modern Robotics: Mechanics, Planning, and Control", Kevin Lynch: https://www.coursera.org/specializations/modernrobotics
```
Papers:
```
"IEEE Xplore Digital Library": https://ieeexplore.ieee.org/Xplore/home.jsp
"arXiv.org e-Print archive": https://arxiv.org/
"Google Scholar": https://scholar.google.com/
```

# 1.ROS
ROS blogs&channel:  
```
"半閒居士"： https://www.cnblogs.com/gaoxiang12/
"MR.POJENLAI": https://pojenlai.wordpress.com/
"The construct": https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q
```
Books:
```
"ROS by Example", python, Patrick Goebel
"Mastering ROS for Robotics Programming", C++, Lentin Joseph
"Learning ROS for Robotics Programming", C++, Enrique Fernandez
"Programming Robots with ROS: A Practical Introduction to...", Morgan Quigley 
```
more:```https://hackmd.io/s/B1fv26WBz#```  

# 2.Robot platform
ros robots:```https://robots.ros.org/```  
holomic vs. non-holomic
```
"Comparison": https://www.evernote.com/l/ATuaHlX8moZHApQrFpCNVYR4SlRPo8Tz53Y
"Caster wheel": https://en.wikipedia.org/wiki/Caster
"Mecanum wheel": https://en.wikipedia.org/wiki/Mecanum_wheel
"Omni wheel": https://en.wikipedia.org/wiki/Omni_wheel
```
race car project  
```
"MIT": https://mit-racecar.github.io
"Penn": http://f1tenth.org/ [without slam, NAV]
"UCB": http://www.barc-project.com/projects/ [without laser] 
"Georgia Tech": https://github.com/AutoRally [for outdoor]
"Taiwan Hypharos": https://github.com/Hypha-ROS/hypharos_racecar
```
processing unit: ```Raspberry Pi, NVIDIA Jetson TX1, NVIDIA Jetson TX2```  
motor & controller:  ```Faulhaber, Maxon```  

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
```
MIT AerospaceControlsLab
"Decentralized non-communicating multiagent collision avoidance with deep reinforcement learning,"
 Y. F. Chen, M. Liu, M. Everett and J. P. How
2017 IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017, pp. 285-292. 
https://www.youtube.com/watch?v=PS2UoyCTrSw
https://www.youtube.com/watch?v=BryJ9jeBkbU
```
```
MIT AerospaceControlsLab
"Socially aware motion planning with deep reinforcement learning,"
Y. F. Chen, M. Everett, M. Liu and J. P. How, 
2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, 2017, pp. 1343-1350.
https://www.youtube.com/watch?v=CK1szio7PyA&t=2s
```
```
MIT AerospaceControlsLab
"Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning,"
M. Everett, Y. F. Chen and J. P. How, 
2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 3052-3059.
https://www.youtube.com/watch?v=XHoXkWLhwYQ
```
```
Google AI Research
"PRM-RL: Long-range Robotic Navigation Tasks by Combining Reinforcement Learning and Sampling-Based Planning,"
A. Faust et al.,  
2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, 2018, pp. 5113-5120.
https://ai.googleblog.com/2019/02/long-range-robotic-navigation-via.html
```

# 10.Others (Non-tech)
### (1) Famous robotics company
Research center: ```Toyota_Research_Institute(TRI), Microsoft_Research, Google_AI```  
Manipulator: ```ABB, FANUC, KUKA, YASKAWA, TECHMAN_ROBOT, HIWIN, Universal_robots  ```  
Service robot: ```willow_garage, softbank_robotics, fetch_robotics```  
Humanoid: ```boston_dynamics, softbank_robotics```  
AGV: ```omron_robotics, clearpath_robotics&OTTO_Motors, amazon_robotics(Kiva_System), Yujin_Robotics, ROBOTIS, fetch_robotics, GreenTrans, KUKA```  
Leg: ```boston_dynamics, unitree_robotics, MIT_Cheetah```  
Drone: ```Dji, Tello```  
ROS2.0: ```ADLINK ```   

### (2) Famous robotics conferences & journals
Top conference: ```ieee_IROS, ieee_ICRA```  
Top journal: ```ieee_Transactions_on_Robotics_and_Automation```  
Minor conference:```ieee_IECON, ieee_SII, ieee_ISIE, ieee_ICIT, ieee_ICPS```  
Minor journal: ```ieee_Access```  

### (3) Famous robotics competition
Global:
```
"DARPA Robotics Challenge": https://en.wikipedia.org/wiki/DARPA_Robotics_Challenge
"RoboCup": https://en.wikipedia.org/wiki/RoboCup
"Amazon Robotics/Picking Challenge": http://amazonpickingchallenge.org/
```
In Taiwan:
```
SKS 新光保全智慧型保全機器人競賽
PMC 智慧機器人競賽 Robot competition
HIWIN 上銀智慧機械手實作競賽
```

### (4) Famous ros organizations & activities
organizations
```
"Open Source Robotics Foundation (OSRF)": https://www.openrobotics.org/
"Open Source Robotics Corporation (OSRC)": https://www.openrobotics.org/
"ROS.Taiwan": https://www.facebook.com/groups/ros.taiwan/
"ROS.Taipei": https://www.facebook.com/groups/ros.taipei/
```
activities: 
```
"ROScon": https://roscon.ros.org/
"ROS summer school(CN)": http://www.roseducation.org/
```
