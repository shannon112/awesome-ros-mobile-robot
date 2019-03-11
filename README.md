# awesome-ros-mobile-robot  [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/shannon112/awesome-ros-mobile-robot)  
Autonomous Mobile Robots aka. AMR  
Autonomous Mobile Manipulators aka. AMM  
This repository provides some useful resources and informations about **mobile robots (AMR&AMM)** research based on **ROS**. It would not contain high level application, but focus on basic function of mobile robots(and more at **navigation**). (including both **Chinese** and **English** materials)  

## contents:  
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

## 0.Robotics
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
"Linear algebra", Hung-yi Lee: http://speech.ee.ntu.edu.tw/~tlkagk/courses_LA18.html
```
Papers:
```
"IEEE Xplore Digital Library": https://ieeexplore.ieee.org/Xplore/home.jsp
"arXiv.org e-Print archive": https://arxiv.org/
"Google Scholar": https://scholar.google.com/
```

## 1.ROS
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
more: https://hackmd.io/s/B1fv26WBz# 

## 2.Robot platform
ROS robots: https://robots.ros.org/  
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
ROS mobile robot
```
"turtlebot": https://github.com/turtlebot
"clearpath husky": https://github.com/husky
"clearpath jackel": https://github.com/jackal
```
ROS mobile manipulator
```
"kuka youbot": https://github.com/youbot
"clearpath husky+UR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyManip.html
"clearpath husky+dualUR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html
```
processing unit: ```Raspberry Pi, NVIDIA Jetson TX1, NVIDIA Jetson TX2```  
motor & controller:  ```Faulhaber, Maxon```  

## 3.Robot sensor
rgbd-camera:  
```
microsoft kinectv1 with openni: https://github.com/ros-drivers/openni_camera
microsoft kinectv1 with freenect: https://github.com/ros-drivers/freenect_stack
asus xtion with openni2: https://github.com/ros-drivers/openni2_camera
intel realsense d435: https://github.com/intel-ros/realsense
```
lidar(laser scanner):  
```
hokuyo_urg: http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
hokuyo_utm: http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
rplidar: http://wiki.ros.org/rplidar
```
imu:  
```
SparkFun 9DOF Razor IMUM0: http://wiki.ros.org/razor_imu_9dof
```

## 4.SLAM
```
The classical SLAM theorem
“Simultaneous localisation and map- ping (SLAM): Part II”
T. Bailey and H. F. Durrant-Whyte
IEEE Robot. Auton. Syst., vol. 13, no. 3, pp. 108–117, 2006. 
```
```
The classical SLAM theorem
“Simultaneous localisation and map- ping (SLAM): Part I”
H. F. Durrant-Whyte and T. Bailey
IEEE Robot. Autom. Mag., vol. 13, no. 2, pp. 99–110, Jun. 2006
```
```
Survey paper
“Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age”
Cesar Cadena ; Luca Carlone ; Henry Carrillo ; Yasir Latif ; Davide Scaramuzza ; José Neira ; Ian Reid ; John J. Leonard
IEEE Transactions on RoboticsYear: 2016, Volume: 32, Issue: 6Pages: 1309 - 1332
```
Cartographer from Google: https://google-cartographer-ros.readthedocs.io/en/latest/
```
“Real-time loop closure in 2D LIDAR SLAM ”
Wolfgang Hess ; Damon Kohler ; Holger Rapp ; Daniel Andor 
2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016, pp. 1271-1278.
```
RTABmap: http://introlab.github.io/rtabmap/
```
“RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation” 
M. Labbé and F. Michaud
Journal of Field Robotics, accepted, 2018
Universit ́e de Sherbrooke
```
gmapping: http://wiki.ros.org/gmapping
```
"Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters,"
G. Grisetti, C. Stachniss and W. Burgard,  in 
IEEE Transactions on Robotics, vol. 23, no. 1, pp. 34-46, Feb. 2007.
```

## 5.Map
OctoMap - 3D occupancy mapping: https://octomap.github.io/
```
OctoMap: An efficient probabilistic 3D mapping framework based on octrees. Autonomous Robots.
Hornung, Armin & Wurm, Kai & Bennewitz, Maren & Stachniss, Cyrill & Burgard, Wolfram. (2013).  
Autonomous Robots Journal. 34. 10.1007/s10514-012-9321-0. 
```

## 6.Localization
amcl Adaptive (or KLD-sampling) Monte Carlo localization: http://wiki.ros.org/amcl  
mrpt_localization: http://wiki.ros.org/mrpt_localization  

## 7.Sensor fusion (for odom-to-base_link or map-to-base_link)
odometry
```
Wheel encoder and actuator | "ros_control": http://wiki.ros.org/ros_control
Laser odometry(old) | "laser_scan_matcher": http://wiki.ros.org/laser_scan_matcher
Laser odometry | "rf2o": https://github.com/MAPIRlab/rf2o_laser_odometry
Visual odometry: 
Visual-Inertial odometry: 
Inertial odometry: 
```
sensor fusion  
```
ekf | "robot_pose_ekf": http://wiki.ros.org/robot_pose_ekf
ekf&ukf | "robot_localization": http://docs.ros.org/melodic/api/robot_localization/html/index.html
```

## 8.Navigation (move_base compatible, nav_core supported)
dwa_local_planner, base_local_planner http://wiki.ros.org/dwa_local_planner
```
"The dynamic window approach to collision avoidance,"
D. Fox, W. Burgard and S. Thrun,  
in IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23-33, March 1997.
```
teb_local_planner http://wiki.ros.org/teb_local_planner
```
C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.
Source code: https://github.com/rst-tu-dortmund/teb_local_planner
```
navigation_stack: http://wiki.ros.org/navigation  
local_planner:  
```base_local_planner, dwa_local_planner, eband_local_planner, teb_local_planner, robotino_local_planner, asr_ftc_local_planner, simple_local_planner```  
global_planner:  
```carrot_planner, navfn, global_planner, sbpl_lattice_planner, srl_global_planner, voronoi_planner```  
RecoveryBehavior: ```rotate_recovery, move_slow_and_clear, stepback_and_steerturn_recovery```  

## 9.Navigation (social aware, novel research)
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

## 10.Others (Non-tech)
### (1) Famous robotics related company
Research center: ```Toyota_Research_Institute(TRI), Microsoft_Research, Google_AI```  
Manipulator: ```ABB, FANUC, KUKA, YASKAWA, TECHMAN_ROBOT, HIWIN, Universal_robots  ```  
Mobile Robot(AGV, base only): ```omron_robotics, clearpath_robotics&OTTO_Motors, amazon_robotics(Kiva_System), Yujin_Robotics, ROBOTIS, fetch_robotics, GreenTrans, KUKA, iRobot```  
Service robot(with torso): ```willow_garage, softbank_robotics, fetch_robotics```  
Humanoid: ```boston_dynamics, softbank_robotics, PAL_Robotics```  
Four-Leg: ```boston_dynamics, unitree_robotics, MIT_Cheetah```  
Drone: ```Dji, Tello```  
ROS2.0: ```ADLINK ```
Cleaning: ```iRobot```

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
"SKS 新光保全智慧型保全機器人競賽": https://www.facebook.com/sksrobot/
"PMC 智慧機器人競賽 Robot competition": http://www.pmccontest.com/
"HIWIN 上銀智慧機械手實作競賽": http://www.hiwin.org.tw/Awards/HIWIN_ROBOT/Original.aspx
```

### (4) Famous ros organizations & activities
organizations:
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
