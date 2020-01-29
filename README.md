# awesome-ros-mobile-robot  [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/shannon112/awesome-ros-mobile-robot)  
This repository provides some useful resources and informations about **autonomous mobile robots (AMR)** research based on **ROS**. It would mainly focus on basic function of mobile robots(like **odometry**, **SLAM** and **navigation**).  
(including both **Chinese** and **English** materials)   

<img src="https://wiki.ros.org/boxturtle?action=AttachFile&do=get&target=Box_Turtle.320.png" height="100"> <img src="https://wiki.ros.org/custom/images/wiki/cturtle.jpg" height="100"> <img src="https://wiki.ros.org/custom/images/wiki/diamondback_posterLo-240w.jpg" height="100"> <img src="https://www.ros.org/news/resources/2011/electric_640w.png" height="100"> <img src="https://wiki.ros.org/custom/images/fuerte-320w.jpg" height="100"> <img src="https://wiki.ros.org/custom/images/groovygalapagos-320w.jpg" height="100"> <img src="http://i.imgur.com/xvfZPAo.png" height="100"> <img src="http://i.imgur.com/YBCUixi.png" height="100"> <img src="http://i.imgur.com/99oTyT5.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/kinetic.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/lunar_with_bg.png" height="100"> 
<img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/melodic_with_bg.png" height="100">

## contents:  
* [0.Robotics](README.md#0_robotics)
* [1.ROS](README.md#1_ros)
* [2.Robot platform](README.md#2_robot_platform)
* [3.Robot sensor](README.md#3_robot_sensor)
* [4.Calibration](README.md#4_calibration)
* [5.Odometry](README.md#5_odometry)
* [6.SLAM](README.md#6_slam)
* [7.RGB-D SLAM](README.md#7_RGBD_slam)
* [8.Localization](README.md#8_localization)
* [9.MAP](README.md#9_map)
* [10-.Navigation](README.md#10_navigation)
* [11.Others (Non-tech)](README.md#11_others_non_tech_part)
  * (1)Famous robotics company  
  * (2)Famous robotics conference&journal  
  * (3)Famous robotics competition in Taiwan  
  * (4)Famous ros organizations & activities  

## 0_Robotics
Books:
```js
"Introduction to Algorithms", Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, Clifford Stein
"Multiple View Geometry in Computer Vision", Richard Hartley, Andrew Zisserman
"Probabilistic Robotics", Sebastian Thrun 
"Introduction to Linear Algebra", Five Edition, Gilbert Strang
"視覺 SLAM 十四講：從理論到實踐", 高翔
```
Courses:
```js
"Robot Mapping" {Universität of Freiburg} Cyrill Stachniss: http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/
"機器人學一 (Robotics (1))" {NTU} 林沛群: https://www.coursera.org/learn/robotics1, http://peichunlin.me.ntu.edu.tw/Homepage/Intro2Robotics.htm
"Control of Mobile Robots" {Georgia Tech} Magnus Egerstedt: https://www.coursera.org/learn/mobile-robot"
"Modern Robotics: Mechanics, Planning, and Control" {Northwestern University} Kevin Lynch: https://www.coursera.org/specializations/modernrobotics
"Robotics" {UPenn} https://zh-tw.coursera.org/specializations/robotics
"Linear algebra" {NTU} Hung-yi Lee: http://speech.ee.ntu.edu.tw/~tlkagk/courses_LA18.html
"Linear algebra" {MIT} Gilbert Strang: https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/
"Machine Learning" {NTU} Hung-yi Lee: http://speech.ee.ntu.edu.tw/~tlkagk/courses_ML19.html
"Machine Learning" {STANFORD} Andrew Ng: https://www.coursera.org/learn/machine-learning
"Probabilistic Systems Analysis and Applied Probability" {MIT} John Tsitsiklis https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-041-probabilistic-systems-analysis-and-applied-probability-fall-2010/"
"Deep Reinforcement Learning" {UCB} Sergey Levine: http://rail.eecs.berkeley.edu/deeprlcourse/
"Vision Algorithms for Mobile Robotics" {ETHZ} 	D. Scaramuzza: http://rpg.ifi.uzh.ch/teaching.html
"Self-Driving Cars" {TORONTO} https://www.coursera.org/specializations/self-driving-cars
```
Paper libraries:
```js
"IEEE Xplore Digital Library": https://ieeexplore.ieee.org/Xplore/home.jsp
"arXiv.org e-Print archive": https://arxiv.org/
"Google Scholar": https://scholar.google.com/
```

## 1_ROS
ROS blogs&channel:  
```js
"半閒居士"： https://www.cnblogs.com/gaoxiang12/
"MR.POJENLAI": https://pojenlai.wordpress.com/
"The construct": https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q
"泡泡機器人": https://space.bilibili.com/38737757/
"泡泡機器人論壇": http://paopaorobot.org/bbs/
```
Books:
```js
"C++ Primer", Stanley B. Lippman, Josée Lajoie, Barbara E. Moo
"ROS by Example", python, Patrick Goebel
"Mastering ROS for Robotics Programming", C++, Lentin Joseph
"Learning ROS for Robotics Programming", C++, Enrique Fernandez
"Programming Robots with ROS: A Practical Introduction to...", Morgan Quigley 
"机器人操作系统（ROS）浅析", Jason M. O'Kane著, 肖军浩译
"ROS 机器人操作系统ROS史话36篇", 张新宇, http://www.roseducation.org/docs/ROS_history.pdf
```

## 2_Robot_platform
**ROS robots:** https://robots.ros.org/  
holomic vs. non-holomic
```js
"Comparison": https://www.evernote.com/l/ATuaHlX8moZHApQrFpCNVYR4SlRPo8Tz53Y
"Caster wheel": https://en.wikipedia.org/wiki/Caster
"Mecanum wheel": https://en.wikipedia.org/wiki/Mecanum_wheel
"Omni wheel": https://en.wikipedia.org/wiki/Omni_wheel
```
race car project  
```js
"MIT": https://mit-racecar.github.io
"Penn": http://f1tenth.org/ [without slam, NAV]
"UCB": http://www.barc-project.com/projects/ [without laser] 
"Georgia Tech": https://github.com/AutoRally [for outdoor]
"Taiwan Hypharos": https://github.com/Hypha-ROS/hypharos_racecar
```
ROS mobile robot
```js
"turtlebot": https://github.com/turtlebot
"turtlebot3": https://github.com/ROBOTIS-GIT/turtlebot3
"clearpath husky": https://github.com/husky
"clearpath jackel": https://github.com/jackal
```
ROS mobile manipulator
```js
"Personal Robot 2 (PR2)": https://github.com/PR2
"kuka youbot": https://github.com/youbot
"fetch robotics": https://github.com/fetchrobotics
"clearpath husky+UR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyManip.html
"clearpath husky+dualUR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html
```
ROS manipulator
```js
"Franka Emika panda": https://github.com/frankaemika/franka_ros | https://github.com/ros-planning/panda_moveit_config
"Universal Robot 3/5/10/e": https://github.com/ros-industrial/universal_robot
"Techman Robot": https://github.com/kentsai0319/techman_robot
```
processing unit: 
```
Raspberry Pi 3(RPi3), BeagleBone Black(BBB)
Odroid XU4, Odroid N2, Asus tinker board
NVIDIA Jetson TX1, NVIDIA Jetson TX2, NVIDIA Jetson NANO, NVIDIA Jetson Xavier
```  
motor & controller:  
```
Elmo Motion Control Ltd,
Dr. Fritz Faulhaber GmbH & Co. KG,
Maxon group motors & drivers, 
Dexmart motors & drivers (Trumman Technology Corp)
```  

## 3_Robot_sensor
RGB camera:  
```js
"usb camera": http://wiki.ros.org/usb_cam
"gstream-based camera": http://wiki.ros.org/gscam
"opencv camera": http://wiki.ros.org/cv_camera
```
RGB-D camera:  
```js
"microsoft kinectv1 with openni": https://github.com/ros-drivers/openni_camera
"microsoft kinectv1 with freenect": https://github.com/ros-drivers/freenect_stack
"microsoft azure-kinect-dk": https://azure.microsoft.com/zh-tw/services/kinect-dk/
"asus xtion with openni2": https://github.com/ros-drivers/openni2_camera
"intel realsense d435": https://github.com/intel-ros/realsense
```
Stereo camera:
```js
"Stereolabs ZED": http://wiki.ros.org/zed-ros-wrapper
"Carnegie Robotics MultiSense™ S7": http://docs.carnegierobotics.com/S7/
"e-Con Systems Tara Stereo Camera": https://github.com/dilipkumar25/see3cam
"nerian SP1": http://wiki.ros.org/nerian_sp1
```
Laser rangefinder [laser scanners] [scanning rangefinder]  
– often represent 2D laser scanning  
```js
"hokuyo_urg": http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
"hokuyo_utm": http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
"ydlidar": https://github.com/YDLIDAR/ydlidar_ros
"rplidar": http://wiki.ros.org/rplidar
"sick": http://wiki.ros.org/sick_scan
```
LIDAR [light detection and ranging] [light imaging, detection, and ranging] [3D laser scanning ]   
– often represent 3D laser scanning
```js
"velodyne": http://wiki.ros.org/velodyne
```
IMU [inertial measurement unit]:  
```js
"SparkFun 9DOF Razor IMUM0": http://wiki.ros.org/razor_imu_9dof
"MicroStrain 3DM-GX5-35": http://wiki.ros.org/microstrain_3dm_gx5_45
```
Odometry & 3D scanning environment
```js
"Kaarta": https://www.kaarta.com/
"matterport": https://matterport.com/
```
Microphone array
```js
"microsoft azure-kinect-dk": https://azure.microsoft.com/zh-tw/services/kinect-dk/
"ReSpeaker Mic Array v2.0": http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/
```
Matrix barcode (Fiducial Marker Systems)  
```js
"ARTag": http://wiki.ros.org/ar_track_alvar
"AprilTag": http://wiki.ros.org/apriltag_ros
"CALTag": http://www.cs.ubc.ca/labs/imager/tr/2010/Atcheson_VMV2010_CALTag/
"comparison": Sagitov, Artur, et al. "ARTag, AprilTag and CALTag Fiducial Marker Systems: Comparison in a Presence of Partial Marker Occlusion and Rotation." ICINCO (2). 2017.
```
## 4_calibration

camera calibration
```
https://www.mathworks.com/help/vision/ug/camera-calibration.html
https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#bouguetmct
https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
http://wiki.ros.org/image_pipeline/CameraInfo
Bouguet, Jean-Yves. “Camera calibration toolbox for matlab.” (2001).
Z. Zhang, "A flexible new technique for camera calibration," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 22, no. 11, pp. 1330-1334, Nov. 2000.
```
Camera calibration tool
```
http://wiki.ros.org/camera_calibration
http://wiki.ros.org/camera_calibration_parsers
http://wiki.ros.org/camera_calibration/Tutorials
```
Camera hand-eye calibration on manipulator
```sh
#Papers
M. Shah, R. D. Eastman, T. Hong, An Overview of Robot-Sensor Calibration Methods for Evaluation of Perception Systems, Performance Metrics for Intelligent Systems, (2012). 
[1] Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE Transactions on 5.3 (1989): 345-358.
Domae, Yukiyasu, et al. "Fast graspability evaluation on single depth maps for bin picking with general grippers." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.
Mano, Kousuke, et al. "Fast and Precise Detection of Object Grasping Positions with Eigenvalue Templates." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.
#Tutorials
https://blog.csdn.net/u011089570/article/details/47945733
https://www.twblogs.net/a/5bb026aa2b7177781a0fc79a
http://math.loyola.edu/~mili/Calibration/index.html
#Tools
http://wiki.ros.org/rc_visard/Tutorials/HandEyeCalibration
https://github.com/IFL-CAMP/easy_handeye
```
IMU(9dof-razor-imu-m0) calibration
```
https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial
https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/all
http://wiki.ros.org/razor_imu_9dof
```

## 5_odometry

#1#2 LOAM, V-LOAM, DEMO - lidar 
```
201905KITTI#1#2 - Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar.
J Zhang, S Singh, "LOAM: Lidar Odometry and Mapping in Real-time", Robotics: Science and Systems Conference (RSS 2014)
J Zhang, S Singh, "Visual-lidar Odometry and Mapping: Low-drift, Robust, and Fast", IEEE International Conference on Robotics and Automation (ICRA)
J. Zhang, M. Kaess and S. Singh: Real-time Depth Enhanced Monocular Odometry. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2014.
https://github.com/laboshinl/loam_velodyne
https://github.com/cuitaixiang/LOAM_NOTED
```
#3#5 IMLS-SLAM, IMLS-SLAM++ - lidar
```
201905KITTI#3#5
Jean-Emmanuel Deschaud, "IMLS-SLAM: scan-to-model matching based on 3D data", arXiv:1802.08633 [cs.RO]
```
#4#17 SOFT, SOFT2 - Stereo 
```
201905KITTI#4,17 - Stereo Odometry based on careful Feature selection and Tracking. 
Cvišic, Igor, et al. "Soft-slam: Computationally efficient stereo visual slam for autonomous uavs." Journal of field robotics (2017).
Cvišić, Igor, and Ivan Petrović. "Stereo odometry based on careful feature selection and tracking." 2015 European Conference on Mobile Robots (ECMR). IEEE, 2015.
https://github.com/Mayankm96/Stereo-Odometry-SOFT
```
#10#19 RotRocc+, RotRocc, ROCC, MonoROCC - stereo?
```
201905KITTI#10#19
M. Buczko and V. Willert: Flow-Decoupled Normalized Reprojection Error for Visual Odometry. 19th IEEE Intelligent Transportation Systems Conference (ITSC) 2016.
M. Buczko, V. Willert, J. Schwehr and J. Adamy: Self-Validation for Automotive Visual Odometry. IEEE Intelligent Vehicles Symposium (IV) 2018.
M. Buczko: Automotive Visual Odometry. 2018.
M. Buczko and V. Willert: Monocular Outlier Detection for Visual Odometry. IEEE Intelligent Vehicles Symposium (IV) 2017.
M. Buczko and V. Willert: How to Distinguish Inliers from Outliers in Visual Odometry for High-speed Automotive Applications. IEEE Intelligent Vehicles Symposium (IV) 2016.
```
#11,13,23 LIMO_GP, LIMO2, LIMO - mono + lidar
```
201905KITTI#11,13,23 - Lidar-Monocular Visual Odometry
https://github.com/johannes-graeter/limo
Graeter, Johannes, Alexander Wilczynski, and Martin Lauer. "Limo: Lidar-monocular visual odometry." 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.
```
#34 VINS - Stereo, mono, RGBD + inertial
```
201905KITTI#34 - An optimization-based multi-sensor state estimator
Online Temporal Calibration for Monocular Visual-Inertial Systems, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), best student paper award pdf
VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Roboticspdf
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
https://github.com/HKUST-Aerial-Robotics/VINS-Mono
```
#40 ORB-SLAM2 - Stereo, mono, RGBD
```
201905KITTI#40 - Real-Time SLAM for Monocular, Stereo and RGB-D Cameras, with Loop Detection and Relocalization Capabilities
[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics Best Paper Award). PDF.
[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics, vol. 33, no. 5, pp. 1255-1262, 2017. PDF.
[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188-1197, 2012. PDF
https://github.com/appliedAI-Initiative/orb_slam_2_ros
https://github.com/ethz-asl/orb_slam_2_ros
https://github.com/raulmur/ORB_SLAM2
https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map
```
#48 VoBa - IMU, visual-aided
```
201905KITTI#48
J. Tardif, M. George, M. Laverne, A. Kelly and A. Stentz: A new approach to vision-aided inertial navigation. 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems, October 18-22, 2010, Taipei, Taiwan 2010.
```
#54 RTAB-Map - RGB-D, Stereo and Lidar
```
A RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. 
M. Labbé and F. Michaud, “RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation,” in Journal of Field Robotics, vol. 36, no. 2, pp. 416–446, 2019. (Wiley) Universit ́e de Sherbrooke
http://introlab.github.io/rtabmap/
https://github.com/introlab/rtabmap_ros
```
#109 VISO2 - Stereo or Mono
```
Geiger, Andreas, Julius Ziegler, and Christoph Stiller. "Stereoscan: Dense 3d reconstruction in real-time." 2011 IEEE Intelligent Vehicles Symposium (IV). Ieee, 2011.
http://wiki.ros.org/viso2_ros
http://www.cvlibs.net/software/libviso/
```
DeepVO - learning_based: RGB
```
S. Wang, R. Clark, H. Wen and N. Trigoni, "DeepVO: Towards end-to-end visual odometry with deep Recurrent Convolutional Neural Networks," 2017 IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017, pp. 2043-2050.
https://github.com/ChiWeiHsiao/DeepVO-pytorch
https://github.com/ildoonet/deepvo
https://github.com/krrish94/DeepVO
https://github.com/linjian93/pytorch-deepvo
```
VINET - learning_based: RGB + IMU
```
Clark, Ronald, et al. "VINet: Visual-Inertial Odometry as a Sequence-to-Sequence Learning Problem." AAAI. 2017.
https://github.com/HTLife/VINet
```
Wheel encoder odometry
```
Wheel encoder and actuator | "ros_control": http://wiki.ros.org/ros_control
```
Odometry fusion ros pkg 
```
ekf | "robot_pose_ekf": http://wiki.ros.org/robot_pose_ekf
ekf&ukf | "robot_localization": http://docs.ros.org/melodic/api/robot_localization/html/index.html
```
rf2o - 2D laser
```
M. Jaimez, J. Monroy, J. Gonzalez-Jimenez, Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach, IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, pp. 4479-4485, 2016. 
Laser odometry(old) | "laser_scan_matcher": http://wiki.ros.org/laser_scan_matcher
Laser odometry | "rf2o": https://github.com/MAPIRlab/rf2o_laser_odometry
```


## 6_SLAM
Related work keyword
```
Graph-Based optimization / Particle filter / Kalman filter series / learning based
Direct / Indirect Visual Processing
Tightly / Loosely -coupled Sensor Fusion
Dense / Semi-Dense / Sparse map
2D occupancy map / 3D OctoMap / 3D feature map / 3D pointcloud map / TSDF / Surfel
```
SLAM benchmark (dataset)
```
(KITTI) Geiger, Andreas, Philip Lenz, and Raquel Urtasun. "Are we ready for autonomous driving? the kitti vision benchmark suite." 2012 IEEE Conference on Computer Vision and Pattern Recognition. IEEE, 2012.
(MIT Stata Center) Fallon, Maurice, et al. "The mit stata center dataset." The International Journal of Robotics Research 32.14 (2013): 1695-1699.
(Radish) A.Howard and N.Roy, "The robotics data set repository." 2003. [Online]. Available: http://radish.sourceforge.net/ , http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
(measurement) R.K ̈ummerle,  B.Steder,  C.Dornhege,  M.Ruhnke,  G.Grisetti, C.Stachniss, and A.Kleiner, "On measuring the accuracy of SLAMalgorithms," Autonomous Robots, vol. 27, no. 4, pp. 387–407, 2009.
```
Classical SLAM theorem
```
T. Bailey and H. F. Durrant-Whyte, “Simultaneous localisation and map- ping (SLAM): Part II”, IEEE Robot. Auton. Syst., vol. 13, no. 3, pp. 108–117, 2006. 
H. F. Durrant-Whyte and T. Bailey, “Simultaneous localisation and map- ping (SLAM): Part I”, IEEE Robot. Autom. Mag., vol. 13, no. 2, pp. 99–110, Jun. 2006
```
SLAM tutorial
```
Strasdat, Hauke, José MM Montiel, and Andrew J. Davison. "Visual SLAM: why filter?." Image and Vision Computing 30.2 (2012): 65-77. (comparison between filter and graph)
Grisetti, Giorgio, et al. "A tutorial on graph-based SLAM." IEEE Intelligent Transportation Systems Magazine 2.4 (2010): 31-43.
```
SLAM survey paper
```
Cesar Cadena ; Luca Carlone ; Henry Carrillo ; Yasir Latif ; Davide Scaramuzza ; José Neira ; Ian Reid ; John J. Leonard, “Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age”, IEEE Transactions on RoboticsYear: 2016, Volume: 32, Issue: 6Pages: 1309 - 1332
```
VINS survey paper
```
G. Huang, "Visual-Inertial Navigation: A Concise Review," 2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 9572-9582.
```
cartographer - rangeSensor, odom, imu
```
Wolfgang Hess ; Damon Kohler ; Holger Rapp ; Daniel Andor, “Real-time loop closure in 2D LIDAR SLAM ”, 2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016, pp. 1271-1278.
https://github.com/googlecartographer/cartographer
https://github.com/googlecartographer/cartographer_ros
```
gmapping - laser, odom
```
G. Grisetti, C. Stachniss and W. Burgard, "Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters," IEEE Transactions on Robotics, vol. 23, no. 1, pp. 34-46, Feb. 2007.
http://wiki.ros.org/gmapping
```
hector_slam - laser, imu
```
S. Kohlbrecher, O. von Stryk, J. Meyer and U. Klingauf, "A flexible and scalable SLAM system with full 3D motion estimation," 2011 IEEE International Symposium on Safety, Security, and Rescue Robotics, Kyoto, 2011, pp. 155-160.
http://wiki.ros.org/hector_slam
```
karto_slam - laser, odom
```
karto SLAM, ROS package. accessed Nov, 2016. [online], wiki.ros.org/slam_karto
```
ViTa-SLAM - cognitive related SLAM
```
ViTa-SLAM: A Bio-inspired Visuo-Tactile SLAM for Navigation whileInteracting with Aliased Environments
https://arxiv.org/pdf/1906.06422.pdf
```
Kimera - semantic mappping
```
A. Rosinol, M. Abate, Y. Chang, L. Carlone. Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping. arXiv preprint arXiv:1910.02490.
```

## 7_RGBD_SLAM
RGB-D SLAM benchmark (dataset)
```
(KITTI) Geiger, Andreas, Philip Lenz, and Raquel Urtasun. "Are we ready for autonomous driving? the kitti vision benchmark suite." 2012 IEEE Conference on Computer Vision and Pattern Recognition. IEEE, 2012.
(TUM rgbd) Sturm, Jürgen, et al. "A benchmark for the evaluation of RGB-D SLAM systems." 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
(ICL-NUIM rgbd) A. Handa, T. Whelan, J. McDonald, and A. J. Davison, “A bench-mark for rgb-d visual odometry, 3d reconstruction and slam,” inRobotics and automation (ICRA), 2014 IEEE international conferenceon. IEEE, 2014, pp. 1524–1531.
(EuRoC MAV) Burri, Michael, et al. "The EuRoC micro aerial vehicle datasets." The International Journal of Robotics Research 35.10 (2016): 1157-1163.
(survey) Cai, Ziyun, et al. "RGB-D datasets using microsoft kinect or similar sensors: a survey." Multimedia Tools and Applications 76.3 (2017): 4313-4355.
```
RGB-D SLAM survey
```
Jamiruddin, Redhwan, et al. "Rgb-depth slam review." arXiv preprint arXiv:1805.07696 (2018).
Zollhöfer, Michael, et al. "State of the Art on 3D Reconstruction with RGB‐D Cameras." Computer graphics forum. Vol. 37. No. 2. 2018.
```
ORB-SLAM2 - Stereo, mono, RGBD
```
201905KITTI#40 - Real-Time SLAM for Monocular, Stereo and RGB-D Cameras, with Loop Detection and Relocalization Capabilities
[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics Best Paper Award). PDF.
[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics, vol. 33, no. 5, pp. 1255-1262, 2017. PDF.
[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188-1197, 2012. PDF
https://github.com/appliedAI-Initiative/orb_slam_2_ros
https://github.com/ethz-asl/orb_slam_2_ros
https://github.com/raulmur/ORB_SLAM2
https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map
```
DVO & DVO-SLAM
```
Dense Visual SLAM for RGB-D Cameras (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
Robust Odometry Estimation for RGB-D Cameras (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
Real-Time Visual Odometry from Dense RGB-D Images (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.
https://vision.in.tum.de/data/software/dvo
https://github.com/tum-vision/dvo_slam
```
RGBDv2 SLAM with ROS
```
"3D Mapping with an RGB-D Camera", F. Endres, J. Hess, J. Sturm, D. Cremers, W. Burgard, IEEE Transactions on Robotics, 2014.
https://github.com/felixendres/rgbdslam_v2
```
RTAB-Map - RGB-D, Stereo and Lidar
```
A RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. 
M. Labbé and F. Michaud, “RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation,” in Journal of Field Robotics, vol. 36, no. 2, pp. 416–446, 2019. (Wiley) Universit ́e de Sherbrooke
http://introlab.github.io/rtabmap/
https://github.com/introlab/rtabmap_ros
```
KinectFusion: the first one and the famous one
```
Izadi, Shahram, et al. "KinectFusion: real-time 3D reconstruction and interaction using a moving depth camera." Proceedings of the 24th annual ACM symposium on User interface software and technology. ACM, 2011.
Newcombe, Richard A., et al. "Kinectfusion: Real-time dense surface mapping and tracking." ISMAR. Vol. 11. No. 2011. 2011.
```
ElasticFusion: root of many dense slam
```
Whelan, Thomas, et al. "ElasticFusion: Dense SLAM without a pose graph." Robotics: Science and Systems, 2015.
Whelan, Thomas, et al. "ElasticFusion: Real-time dense SLAM and light source estimation." The International Journal of Robotics Research 35.14 (2016): 1697-1716.
Dyson Robotics Laboratory at Imperial College
https://github.com/mp3guy/ElasticFusion
https://www.youtube.com/watch?v=-dz_VauPjEU
https://www.youtube.com/watch?v=XySrhZpODYs
```
BundleFusion: state of art dense slam
```
Dai, Angela, et al. "Bundlefusion: Real-time globally consistent 3d reconstruction using on-the-fly surface reintegration." ACM Transactions on Graphics (ToG) 36.3 (2017): 24.
http://graphics.stanford.edu/projects/bundlefusion/
```
Kimera mono/stereo/IMU (C++ library)
```
A. Rosinol, M. Abate, Y. Chang, L. Carlone. Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping. arXiv preprint arXiv:1910.02490.
```
Dense RGBDi with gpu
```
Laidlow, Tristan, et al. "Dense RGB-D-inertial SLAM with map deformations." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.
Dyson Robotics Laboratory at Imperial College
https://www.youtube.com/watch?v=-gUdQ0cxDh0
```
Dense RGBDi with cpu
```
Hsiao, Ming, Eric Westman, and Michael Kaess. "Dense planar-inertial slam with structural constraints." 2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2018.
https://www.youtube.com/watch?v=kLsyDEX_U0g
```
Dense RGBD-odometry (KO-Fusion)
```
Houseago, Charlie, Michael Bloesch, and Stefan Leutenegger. "KO-Fusion: Dense Visual SLAM with Tightly-Coupled Kinematic and Odometric Tracking." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.
Dyson Robotics Laboratory at Imperial College
https://www.youtube.com/watch?v=yigoIYoY7Wg
```
Desnse RGBD-odometry (arm-slam)
```
M. Klingensmith, S. S. Sirinivasa and M. Kaess, "Articulated Robot Motion for Simultaneous Localization and Mapping (ARM-SLAM)," in IEEE Robotics and Automation Letters, vol. 1, no. 2, pp. 1156-1163, July 2016.
https://www.youtube.com/watch?v=QrFyaxFUs9w
```

## 8_Localization
amcl | Adaptive (or KLD-sampling) Monte Carlo localization: http://wiki.ros.org/amcl  
mrpt_localization: http://wiki.ros.org/mrpt_localization  
SLAM algorithms support pure localization: ```google_cartographer, ORB_SLAM2, RTAB-Map, etc.```

## 9_Map
2D occupancy map / 3D OctoMap / 3D feature map / 3D pointcloud map / TSDF / Surfel  
OctoMap - 3D occupancy mapping: https://octomap.github.io/
```
Hornung, Armin & Wurm, Kai & Bennewitz, Maren & Stachniss, Cyrill & Burgard, Wolfram, "OctoMap: An efficient probabilistic 3D mapping framework based on octrees. Autonomous Robots.", Autonomous Robots Journal (2013). 34. 10.1007/s10514-012-9321-0. 
```

## 10_Navigation
 > move_base compatible, nav_core supported  

navigation_stack: http://wiki.ros.org/navigation  
```js
"local_planner": base_local_planner, dwa_local_planner, eband_local_planner, teb_local_planner, robotino_local_planner, asr_ftc_local_planner, simple_local_planner  
"global_planner": carrot_planner, navfn, global_planner, sbpl_lattice_planner, srl_global_planner, voronoi_planner
"RecoveryBehavior": rotate_recovery, move_slow_and_clear, stepback_and_steerturn_recovery
```
dwa_local_planner, base_local_planner http://wiki.ros.org/dwa_local_planner
```
D. Fox, W. Burgard and S. Thrun, "The dynamic window approach to collision avoidance," in IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23-33, March 1997.
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
condition keywords: ```in crowded spaces, in cluttered environments, socially aware```
```
MIT AerospaceControlsLab
Y. F. Chen, M. Liu, M. Everett and J. P. How "Decentralized non-communicating multiagent collision avoidance with deep reinforcement learning," 2017 IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017, pp. 285-292. 
https://www.youtube.com/watch?v=PS2UoyCTrSw
https://www.youtube.com/watch?v=BryJ9jeBkbU
```
```
MIT AerospaceControlsLab
Y. F. Chen, M. Everett, M. Liu and J. P. How, "Socially aware motion planning with deep reinforcement learning," 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, 2017, pp. 1343-1350.
https://www.youtube.com/watch?v=CK1szio7PyA&t=2s
```
```
MIT AerospaceControlsLab
M. Everett, Y. F. Chen and J. P. How, "Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 3052-3059.
https://www.youtube.com/watch?v=XHoXkWLhwYQ
```
```
Google AI Research
A. Faust et al., "PRM-RL: Long-range Robotic Navigation Tasks by Combining Reinforcement Learning and Sampling-Based Planning," 2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, 2018, pp. 5113-5120.
Francis, Anthony & Faust, Aleksandra & Chiang, Hao-Tien Lewis & Hsu, Jasmine & Chase Kew, J & Fiser, Marek & Edward Lee, Tsang-Wei. (2019). Long-Range Indoor Navigation with PRM-RL. 
H. L. Chiang, A. Faust, M. Fiser and A. Francis, "Learning Navigation Behaviors End-to-End With AutoRL," in IEEE Robotics and Automation Letters, vol. 4, no. 2, pp. 2007-2014, April 2019.
https://ai.googleblog.com/2019/02/long-range-robotic-navigation-via.html
```
```
ETHz Autonomous System Lab
M. Pfeiffer, U. Schwesinger, H. Sommer, E. Galceran and R. Siegwart, "Predicting actions to act predictably: Cooperative partial motion planning with maximum entropy models," 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp. 2096-2101.
https://www.youtube.com/watch?v=GPp5mnybm8g
```
coverage navigation servey
```
Galceran, Enric, and Marc Carreras. "A survey on coverage path planning for robotics." Robotics and Autonomous systems 61.12 (2013): 1258-1276.
```

## 11_Others_Non_tech_part
### (1) Famous robotics related company
| categories | companies |
| --------   | -------- |
| Research center | Toyota_Research_Institute(TRI), Microsoft_Research, Google_AI |
| Manipulator | ABB, FANUC, KUKA, YASKAWA, TECHMAN_ROBOT, HIWIN, Universal_robots, Innfos |
| Mobile Robot(AGV, base only) | Omron_robotics, Clearpath_robotics&OTTO_Motors, Amazon_robotics(Kiva_System), Yujin_Robotics, ROBOTIS, Fetch_robotics, GreenTrans, KUKA, iRobot, Pal_robotics, Robotnik_Automation | 
| Service robot(with torso) | Willow_garage, Softbank_robotics, Fetch_robotics, Pal_robotics, Robotnik_automation, Innfos |  
| Humanoid | Boston_dynamics, Softbank_robotics, Pal_robotics |
| Quadruped | Boston_dynamics, Unitree_robotics, MIT_Cheetah, ANYrobotics(ANYmal), Standford＿Doggo, Innfos |
| Educational Rotbot | Willow_garage(Pr2), Facebook(pyrobot), ROBOTIS(turtlebot3), Fetch_robotics |
| Drone | Dji, Tello |
| ROS2.0 | ADLINK |  
| Cleaning | iRobot |  
| Gripper | ROBOTIQ |  
| Self-Driving Cars | Alphabet Waymo, Uber, Apple Project Titan |

### (2) Famous robotics conferences & journals

| Tile | Website |
| -------- | -------- |
| IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS) | https://ieeexplore.ieee.org/xpl/conhome/1000393/all-proceedings  | 
| IEEE International Conference on Robotics and Automation(ICRA)  |  https://ieeexplore.ieee.org/xpl/conhome/1000639/all-proceedings  |
| IEEE_Transactions_on_Robotics_and_Automation(old) | https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8856  |
| IEEE Transactions on Automation Science and Engineering | https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8856  |
| IEEE_Transactions_on_Robotics | https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8860  |

IEEE Robotics and Automation Society: https://www.ieee-ras.org/conferences-workshops  
IEEE Industrial Electronics Society: http://www.ieee-ies.org/conferences  
Google scholar H5-index rank: https://scholar.google.com/citations?view_op=top_venues&hl=en&vq=eng_robotics  

### (3) Famous robotics competition
Global:
```js
"DARPA Robotics Challenge": https://en.wikipedia.org/wiki/DARPA_Robotics_Challenge
"RoboCup": https://en.wikipedia.org/wiki/RoboCup
"Amazon Robotics/Picking Challenge": http://amazonpickingchallenge.org/
"ICRA Robot Competitions: including lots of competitions would be different every years"
"IROS Robot Competitions: including lots of competitions would be different every years"
```
In Taiwan:
```js
"SKS 新光保全智慧型保全機器人競賽": https://www.facebook.com/sksrobot/
"PMC 智慧機器人競賽 Robot competition": http://www.pmccontest.com/
"HIWIN 上銀智慧機械手實作競賽": http://www.hiwin.org.tw/Awards/HIWIN_ROBOT/Original.aspx
"SiliconAwards 旺宏金矽獎"http://www.mxeduc.org.tw/SiliconAwards/
```

### (4) Famous ros organizations & activities
ROS related work:
```js
"ROS-industrial": https://rosindustrial.org/
"ROS2.0": https://design.ros2.org/
"ROS-H": https://acutronicrobotics.com/technology/H-ROS/"
```
organizations/communities:
```js
"Open Source Robotics Foundation (OSRF)": https://www.openrobotics.org/
"Open Source Robotics Corporation (OSRC)": https://www.openrobotics.org/
"ROS.Taiwan": https://www.facebook.com/groups/ros.taiwan/
"ROS.Taipei": https://www.facebook.com/groups/ros.taipei/
```
activities: 
```js
"ROScon": https://roscon.ros.org/
"ROSDevCon": http://www.theconstructsim.com/ros-developers-online-conference-2019-rdc-worldwide/
"ROS summer school(CN)": http://www.roseducation.org/
```
