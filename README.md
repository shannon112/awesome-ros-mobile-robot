# awesome-ros-mobile-robot  [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)  
This repository provides some useful resources and informations about **autonomous mobile robots (AMR)** research based on **ROS**. It would mainly focus on basic function of mobile robots(like **odometry**, **SLAM**, **navigation** and **manipulation**).  
(including both **Chinese** and **English** materials)  

<img src="https://wiki.ros.org/boxturtle?action=AttachFile&do=get&target=Box_Turtle.320.png" height="100"> <img src="https://wiki.ros.org/custom/images/wiki/cturtle.jpg" height="100"> <img src="https://wiki.ros.org/custom/images/wiki/diamondback_posterLo-240w.jpg" height="100"> <img src="https://www.ros.org/news/resources/2011/electric_640w.png" height="100"> <img src="https://wiki.ros.org/custom/images/fuerte-320w.jpg" height="100"> <img src="https://wiki.ros.org/custom/images/groovygalapagos-320w.jpg" height="100"> <img src="http://i.imgur.com/xvfZPAo.png" height="100"> <img src="http://i.imgur.com/YBCUixi.png" height="100"> <img src="http://i.imgur.com/99oTyT5.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/kinetic.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/lunar_with_bg.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/melodic_with_bg.png" height="100"> <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/noetic.png" height="100">  
[<img src="https://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png" align="right" width="86">](https://www.ros.org/)
http://wiki.ros.org/Distributions

# Index:  
* [0.Robotics](README.md#0_robotics)
* [1.Robot-Operating-System(ROS)](README.md#1_robot_operating_system)
* [2.Robotic-Platform](README.md#2_robotic_platform)
* [3.Robotic-Sensing](README.md#3_robotic_sensing)
* [4.Calibration](README.md#4_calibration)
* [5.Odometry](README.md#5_odometry)
* [6.SLAM](README.md#6_slam)
* [7.Localization](README.md#7_localization)
* [8.Mapping](README.md#8_mapping)
* [9.Navigation](README.md#9_navigation)
* [10.Manipulation](README.md#10_manipulation)
* [11.Others (Non-tech)](README.md#11_others_non_tech_part)
  * 11-1. Famous robotics company  
  * 11-2. Famous robotics conference&journal  
  * 11-3. Famous robotics competition in Taiwan  
  * 11-4. Famous ros organizations & activities  

# 0_Robotics
📚 Books:
 * "Introduction to Algorithms", Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, Clifford Stein
 * "Multiple View Geometry in Computer Vision", Richard Hartley, Andrew Zisserman
 * "Probabilistic Robotics", Sebastian Thrun 
 * "Introduction to Linear Algebra", Five Edition, Gilbert Strang
 * "Pattern Recognition and Machine Learning", Christopher M. Bishop
 * "Introduction to autonomous mobile robots" Siegwart, Roland, Illah Reza Nourbakhsh, and Davide Scaramuzza
 * "視覺 SLAM 十四講：從理論到實踐", 高翔

📖 Courses:
 * "Matlab Lecture" {Matlab}
   * https://www.youtube.com/user/MATLAB/playlists
 * "Control System Lecture" {Brian Douglas} Brian Douglas
   * https://www.youtube.com/user/ControlLectures/playlists
 * "Robotics Sensing Related Lecture" {Cyrill Stachniss} Cyrill Stachniss
   * https://www.youtube.com/c/CyrillStachniss/playlists
* "Robot Mapping" {Universität of Freiburg} Cyrill Stachniss
   * http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/
 * "Introduction to Mobile Robotics" {Universität of Freiburg}  Wolfram Burgard, et al.
   * http://ais.informatik.uni-freiburg.de/teaching/ss13/robotics/
 * "Robotics (1)" {NTU} Pei Chun Lin
   * https://www.coursera.org/learn/robotics1, http://peichunlin.me.ntu.edu.tw/Homepage/Intro2Robotics.htm
 * "Control of Mobile Robots" {Georgia Tech} Magnus Egerstedt
   * https://www.coursera.org/learn/mobile-robot"
 * "Modern Robotics: Mechanics, Planning, and Control" {Northwestern University} Kevin Lynch
   * https://www.coursera.org/specializations/modernrobotics
 * "Robotics" {UPenn} Vijay Kumar, et al. 
   * https://zh-tw.coursera.org/specializations/robotics
 * "Linear algebra" {NTU} Hung-yi Lee
   * http://speech.ee.ntu.edu.tw/~tlkagk/courses_LA18.html
 * "Linear algebra" {MIT} Gilbert Strang
   * https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/
 * "Machine Learning" {NTU} Hung-yi Lee
   * http://speech.ee.ntu.edu.tw/~tlkagk/courses_ML19.html
 * "Machine Learning" {STANFORD} Andrew Ng
   * https://www.coursera.org/learn/machine-learning
 * "Probabilistic Systems Analysis and Applied Probability" {MIT} John Tsitsiklis
   * https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-041-probabilistic-systems-analysis-and-applied-probability-fall-2010/"
 * "Deep Reinforcement Learning" {UCB} Sergey Levine
   * http://rail.eecs.berkeley.edu/deeprlcourse/
 * "Vision Algorithms for Mobile Robotics" {ETHZ} 	D. Scaramuzza
   * http://rpg.ifi.uzh.ch/teaching.html
 * "Self-Driving Cars" {TORONTO}
   * https://www.coursera.org/specializations/self-driving-cars

📜 Paper Libraries:
 * "IEEE Xplore Digital Library": https://ieeexplore.ieee.org/Xplore/home.jsp
 * "arXiv.org e-Print archive": https://arxiv.org/
 * "Google Scholar": https://scholar.google.com/
 * "Sci-Hub": https://sci-hub.tw/

# 1_Robot_Operating_System
👾 Resources:
 * The ROS status: https://status.ros.org/
 * The ROS wiki: https://wiki.ros.org
 * The ROS website: https://www.ros.org
 * The ROS build farm: http://build.ros.org
 * The ROS package repository http://packages.ros.org
 * The ROS documentation site: http://docs.ros.org

🗣 ROS Blogs & Channels & Forums:  
 * "ROS Answer": https://answers.ros.org/questions/
 * "The construct": https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q
 * "半閒居士"： https://www.cnblogs.com/gaoxiang12/
 * "泡泡機器人": https://space.bilibili.com/38737757/
 * "泡泡機器人論壇": http://paopaorobot.org/bbs/

📚 Books:
 * "C++ Primer", Stanley B. Lippman, Josée Lajoie, Barbara E. Moo
 * "ROS by Example", python, Patrick Goebel
 * "Mastering ROS for Robotics Programming", C++, Lentin Joseph
 * "Learning ROS for Robotics Programming", C++, Enrique Fernandez
 * "Programming Robots with ROS: A Practical Introduction to...", Morgan Quigley 
 * "機器人作業系統ROS 淺析", Jason M. O'Kane著, 肖軍浩譯
 * "機器人作業系統ROS 史话36篇", 張新宇, http://www.roseducation.org/docs/ROS_history.pdf

# 2_Robotic_Platform
🤖 ROS Robot Overview
 * https://robots.ros.org/  

🚘 Wheel Robot Configurations  
> (ref: Siegwart, Roland, Illah Reza Nourbakhsh, and Davide Scaramuzza. Introduction to autonomous mobile robots. MIT press, 2011, Table 2.1, p.34~36)  

<img src="https://i.imgur.com/STBL1pF.png" height="300"><img src="https://i.imgur.com/ubb5N8B.png" height="300"><img src="https://i.imgur.com/8GtsLOD.png" height="300">

🚗 Race Car Projects  
 * "MIT": https://mit-racecar.github.io
 * "Penn": http://f1tenth.org/ [without slam, NAV]
 * "UCB": http://www.barc-project.com/projects/ [without laser] 
 * "Georgia Tech": https://github.com/AutoRally [for outdoor]
 * "Taiwan Hypharos": https://github.com/Hypha-ROS/hypharos_racecar

🤖 ROS Mobile Robot Github  
 * "turtlebot": https://github.com/turtlebot
 * "turtlebot3": https://github.com/ROBOTIS-GIT/turtlebot3
 * "clearpath husky": https://github.com/husky
 * "clearpath jackel": https://github.com/jackal
 * "Robotnik XL-GEN": https://github.com/RobotnikAutomation/summit_xl_sim or summit_xl_common
 * "Robotnik RB-KAIROS": https://github.com/RobotnikAutomation/rbkairos_sim or rbkairos_common

🤖 ROS Mobile Manipulator Github  
 * "Personal Robot 2 (PR2)": https://github.com/PR2
 * "kuka youbot": https://github.com/youbot
 * "fetch robotics": https://github.com/fetchrobotics
 * "clearpath husky+UR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyManip.html
 * "clearpath husky+dualUR5": http://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html
 * "Robotnik RB-1": https://github.com/RobotnikAutomation/rb1_sim or rb1_common

🤖 ROS Manipulator Github
 * "Franka Emika panda": https://github.com/frankaemika/franka_ros | https://github.com/ros-planning/panda_moveit_config
 * "Universal Robot 3/5/10/e": https://github.com/ros-industrial/universal_robot
 * "Techman Robot": https://github.com/kentsai0319/techman_robot

💻 Processing Unit  
 * Raspberry Pi 3(RPi3), BeagleBone Black(BBB)
 * Odroid XU4, Odroid N2, Asus tinker board
 * NVIDIA Jetson TX1, NVIDIA Jetson TX2, NVIDIA Jetson NANO, NVIDIA Jetson Xavier

🕹 Motor & Controller & Encoder:  
 * Elmo Motion Control Ltd,
 * Dr. Fritz Faulhaber GmbH & Co. KG,
 * Maxon group motors & drivers, 
 * Dexmart motors & drivers (Trumman Technology Corp)
 * RLS d.o.o. (Rotary and Linear Motion Sensors)

# 3_Robotic_Sensing
📷 RGB Camera  
 * "usb camera": http://wiki.ros.org/usb_cam
 * "gstream-based camera": http://wiki.ros.org/gscam
 * "opencv camera": http://wiki.ros.org/cv_camera

📸 RGB-D Camera  
 * "microsoft kinectv1 with openni": https://github.com/ros-drivers/openni_camera
 * "microsoft kinectv1 with freenect": https://github.com/ros-drivers/freenect_stack
 * "microsoft kinect one/v2": https://github.com/code-iai/iai_kinect2
 * "asus xtion with openni2": https://github.com/ros-drivers/openni2_camera
 * "intel realsense d435": https://github.com/intel-ros/realsense

🎥 Stereo Camera
 * "Stereolabs ZED": http://wiki.ros.org/zed-ros-wrapper
 * "Carnegie Robotics MultiSense™ S7": http://docs.carnegierobotics.com/S7/
 * "e-Con Systems Tara Stereo Camera": https://github.com/dilipkumar25/see3cam
 * "nerian SP1": http://wiki.ros.org/nerian_sp1

🔦 Laser Rangefinder [laser scanners] [scanning rangefinder]  
– often represent 2D laser scanning  
 * "hokuyo_urg": http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
 * "hokuyo_utm": http://wiki.ros.org/urg_node (old: http://wiki.ros.org/hokuyo_node
 * "ydlidar": https://github.com/YDLIDAR/ydlidar_ros
 * "rplidar": http://wiki.ros.org/rplidar
 * "sick": http://wiki.ros.org/sick_scan

💡 LIDAR [light detection and ranging] [light imaging, detection, and ranging] [3D laser scanning ]   
– often represent 3D laser scanning  
 * "velodyne": http://wiki.ros.org/velodyne

🍎 IMU [inertial measurement unit]:  
 * "Xsense": http://wiki.ros.org/xsens_driver
 * "MicroStrain 3DM-GX2": http://wiki.ros.org/microstrain_3dmgx2_imu
 * "SparkFun 9DOF Razor IMUM0": http://wiki.ros.org/razor_imu_9dof

🚨 3D Scanning & Novel Sensing Device
 * "Kaarta": https://www.kaarta.com/
 * "matterport": https://matterport.com/
 * "Intel LiDAR Camera L515": https://www.intelrealsense.com/lidar-camera-l515/
 * "microsoft azure-kinect-dk": https://azure.microsoft.com/zh-tw/services/kinect-dk/

🎙 Microphone Array
 * "microsoft azure-kinect-dk": https://azure.microsoft.com/zh-tw/services/kinect-dk/
 * "ReSpeaker Mic Array v2.0": http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/
 
🔊 Text To Speech (TTS)  
 * "gTTS": https://pypi.org/project/gTTS/
 * "sound_play": http://wiki.ros.org/sound_play

🗣 Speech Recognition
 * "SpeechRecognition": https://pypi.org/project/SpeechRecognition/

🚀 Vocal Assistant
 * "Amazon Alexa": https://www.amazon.com/Amazon-Echo-And-Alexa-Devices/b?ie=UTF8&node=9818047011
 * "Google Nest": https://store.google.com/product/google_nest_mini
 * "Apple Homepod": https://www.apple.com/tw/shop/buy-homepod/homepod/
 * "Mi AI Speaker": https://www.mi.com/aispeaker
 * "ASUS Smart Speaker": https://www.asus.com/tw/ASUS-Smart-Speaker/ASUS-Smart-Speaker-Xiao-Bu/
 * "PyAIML -- The Python AIML Interpreter": https://github.com/cdwfs/pyaiml

👾 Matrix Barcode (Fiducial Marker Systems, or ARTag, or Auxiliary marker)  
 * "ARTag": http://wiki.ros.org/ar_track_alvar
 * "AprilTag": http://wiki.ros.org/apriltag_ros
 * "CALTag": http://www.cs.ubc.ca/labs/imager/tr/2010/Atcheson_VMV2010_CALTag/
 * "comparison": Sagitov, Artur, et al. "ARTag, AprilTag and CALTag Fiducial Marker Systems: Comparison in a Presence of Partial Marker Occlusion and Rotation." ICINCO (2). 2017.

🔅 Learning-Based Feature Extractor  
 * ```Alexnet, VGG, ResNet, InceptionV3, DenseNet, GoogleNet, MobileNet, SqueezeNet, etc.```
 * Pytorch implementation: https://pytorch.org/docs/stable/torchvision/models.html

🔅 Learning-Based Object Detection
 * "Faster R-CNN"
   > Ren, Shaoqing, et al. "Faster r-cnn: Towards real-time object detection with region proposal networks." Advances in neural information processing systems. 2015.
 * "SSD" 
   > Liu, Wei, et al. "Ssd: Single shot multibox detector." European conference on computer vision. Springer, Cham, 2016.
 * "YOLOv3" https://github.com/leggedrobotics/darknet_ros
   > (v4) Bochkovskiy, Alexey, Chien-Yao Wang, and Hong-Yuan Mark Liao. "YOLOv4: Optimal Speed and Accuracy of Object Detection." arXiv preprint arXiv:2004.10934 (2020).  
   > (v3) Redmon, Joseph, and Ali Farhadi. "Yolov3: An incremental improvement." arXiv preprint arXiv:1804.02767 (2018).  
   > (v2) Redmon, Joseph, and Ali Farhadi. "YOLO9000: better, faster, stronger." Proceedings of the IEEE conference on computer vision and pattern recognition. 2017.  
   > (v1) Redmon, Joseph, et al. "You only look once: Unified, real-time object detection." Proceedings of the IEEE conference on computer vision and pattern recognition. 2016.  
   
🔅 Learning-Based Human Pose Estimation
 * "OpenPose": https://github.com/CMU-Perceptual-Computing-Lab/openpose
 * "OpenPose-plugin": https://github.com/ildoonet/tf-pose-estimation

# 4_Calibration
📷 Camera Calibration (Intrinsic and Extrinsic parameters)
 * "Tool": http://wiki.ros.org/camera_calibration
 * "Converter": http://wiki.ros.org/camera_calibration_parsers

👁 Hand-Eye Calibration
 * "Tool": https://github.com/IFL-CAMP/easy_handeye

🍎 IMU(9dof-razor-imu-m0) calibration
 * "Github Wiki": https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial
 * "Sparkfun": https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/all
 * "ROS Wiki": http://wiki.ros.org/razor_imu_9dof
 * "Calibration Guide": https://github.com/shannon112/imu_calibration/blob/master/README.md

# 5_Odometry
☠︎ Visual Based Ego-Motion Backbone
* Components
  * Feature Keypoint & Desciptor - ```SURF, SIFT, ORB``` 
  * Feature Matching - ```Brute-Force, FLANN```
    * https://docs.opencv.org/3.4/db/d27/tutorial_py_table_of_contents_feature2d.html
  * Optical Flow - ```Lucas-Kanade (LK)```
  * Motion Estimation:
    * 2D-2D: Epipolar Geometry & Triangulation
    * 2D-3D: Perspective-n-Point (PnP) - ```P3P, DLT, EPnP, UPnP, BA```
    * 3D-3D: Iterative Closest Point (ICP) - ```ICP(SVD), GICP, NDT, IPDA, Non-LinearOpt```, ```point2point, point2plane```
  * Direct Method - ```Dense, Semi-Dense, Sparse```
* Solutions
  * Extract Feature Keypoint -> Desciptor -> Matching -> Motion Estimation
  * Extract Feature Keypoint -> Optical Flow -> Motion Estimation
  * Extract Feature Keypoint -> Sparse Direct Method
  * Semi-Dense/Dense Direct Method

📚 Odometry Survey Paper
* Delmerico, Jeffrey, and Davide Scaramuzza. "A benchmark comparison of monocular visual-inertial odometry algorithms for flying robots." 2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2018.
* G. Huang, "Visual-Inertial Navigation: A Concise Review," 2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 9572-9582.

🏆 Odometry Algorithm Ranking
* KITTI: http://www.cvlibs.net/datasets/kitti/eval_odometry.php

🚖 Wheel Odometry
* ros_control http://wiki.ros.org/ros_control
  > Chitta, Sachin, et al. "ros_control: A generic and simple control framework for ROS." (2017).

💡 2D Laser Based Odometry
* rf2o https://github.com/MAPIRlab/rf2o_laser_odometry
  > M. Jaimez, J. Monroy, J. Gonzalez-Jimenez, Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach, IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, pp. 4479-4485, 2016. 

📷 3D Visual Based Odometry
* VINS-Mono https://github.com/HKUST-Aerial-Robotics/VINS-Mono
  > Qin, Tong, Peiliang Li, and Shaojie Shen. "Vins-mono: A robust and versatile monocular visual-inertial state estimator." IEEE Transactions on Robotics 34.4 (2018): 1004-1020.
* VISO2 http://wiki.ros.org/viso2_ros | http://www.cvlibs.net/software/libviso/
  > Geiger, Andreas, Julius Ziegler, and Christoph Stiller. "Stereoscan: Dense 3d reconstruction in real-time." 2011 IEEE Intelligent Vehicles Symposium (IV). Ieee, 2011.
* RotRocc+, RotRocc, ROCC, MonoROCC
  > M. Buczko and V. Willert: Flow-Decoupled Normalized Reprojection Error for Visual Odometry. 19th IEEE Intelligent Transportation Systems Conference (ITSC) 2016.  
  > M. Buczko, V. Willert, J. Schwehr and J. Adamy: Self-Validation for Automotive Visual Odometry. IEEE Intelligent Vehicles Symposium (IV) 2018.  
  > M. Buczko and V. Willert: Monocular Outlier Detection for Visual Odometry. IEEE Intelligent Vehicles Symposium (IV) 2017.  
  > M. Buczko and V. Willert: How to Distinguish Inliers from Outliers in Visual Odometry for High-speed Automotive Applications. IEEE Intelligent Vehicles Symposium (IV) 2016.  

📸 3D RGB-D/Stereo Based Odometry
* VINS-Fusion https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
  > Qin, Tong, and Shaojie Shen. "Online temporal calibration for monocular visual-inertial systems." 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.
* DVO https://github.com/tum-vision/dvo
  > Kerl, Christian, Jürgen Sturm, and Daniel Cremers. "Robust odometry estimation for RGB-D cameras." 2013 IEEE International Conference on Robotics and Automation. IEEE, 2013.
  > Steinbrücker, Frank, Jürgen Sturm, and Daniel Cremers. "Real-time visual odometry from dense RGB-D images." 2011 IEEE international conference on computer vision workshops (ICCV Workshops). IEEE, 2011.
* SOFT https://github.com/Mayankm96/Stereo-Odometry-SOFT
  > Cvišic, Igor, et al. "Soft-slam: Computationally efficient stereo visual slam for autonomous uavs." Journal of field robotics (2017).  
  > Cvišić, Igor, and Ivan Petrović. "Stereo odometry based on careful feature selection and tracking." 2015 European Conference on Mobile Robots (ECMR). IEEE, 2015.  
* VISO2 http://wiki.ros.org/viso2_ros | http://www.cvlibs.net/software/libviso/
  > Geiger, Andreas, Julius Ziegler, and Christoph Stiller. "Stereoscan: Dense 3d reconstruction in real-time." 2011 IEEE Intelligent Vehicles Symposium (IV). Ieee, 2011.  

🔅 3D LiDAR Based Odometry
* LOAM & V-LOAM https://github.com/laboshinl/loam_velodyne
  > J Zhang, S Singh, "LOAM: Lidar Odometry and Mapping in Real-time", Robotics: Science and Systems Conference (RSS 2014)  
  > J Zhang, S Singh, "Visual-lidar Odometry and Mapping: Low-drift, Robust, and Fast", IEEE International Conference on Robotics and Automation (ICRA)  
  > J. Zhang, M. Kaess and S. Singh: Real-time Depth Enhanced Monocular Odometry. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2014.  
* LIMO https://github.com/johannes-graeter/limo
  > Graeter, Johannes, Alexander Wilczynski, and Martin Lauer. "Limo: Lidar-monocular visual odometry." 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.

 🤖 Learning Based Odometry
* DeepVO https://github.com/ChiWeiHsiao/DeepVO-pytorch | https://github.com/ildoonet/deepvo
  > S. Wang, R. Clark, H. Wen and N. Trigoni, "DeepVO: Towards end-to-end visual odometry with deep Recurrent Convolutional Neural Networks," 2017 IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017, pp. 2043-2050.
* VINET https://github.com/HTLife/VINet
  > Clark, Ronald, et al. "VINet: Visual-Inertial Odometry as a Sequence-to-Sequence Learning Problem." AAAI. 2017.

🍥 Odometry Fusion
* EKF | "robot_pose_ekf": http://wiki.ros.org/robot_pose_ekf
* EKF & UKF | "robot_localization": http://docs.ros.org/melodic/api/robot_localization/html/index.html
  > Moore, Thomas, and Daniel Stouch. "A generalized extended kalman filter implementation for the robot operating system." Intelligent autonomous systems 13. Springer, Cham, 2016. 335-348.

# 6_SLAM
🏛 SLAM Theorem & Tutorial
* T. Bailey and H. F. Durrant-Whyte, “Simultaneous localisation and map- ping (SLAM): Part II”, IEEE Robot. Auton. Syst., vol. 13, no. 3, pp. 108–117, 2006.  
* H. F. Durrant-Whyte and T. Bailey, “Simultaneous localisation and map- ping (SLAM): Part I”, IEEE Robot. Autom. Mag., vol. 13, no. 2, pp. 99–110, Jun. 2006
* Strasdat, Hauke, José MM Montiel, and Andrew J. Davison. "Visual SLAM: why filter?." Image and Vision Computing 30.2 (2012): 65-77. (comparison between filter and graph)
* Grisetti, Giorgio, et al. "A tutorial on graph-based SLAM." IEEE Intelligent Transportation Systems Magazine 2.4 (2010): 31-43.

📚 SLAM Survey Paper
* Cesar Cadena ; Luca Carlone ; Henry Carrillo ; Yasir Latif ; Davide Scaramuzza ; José Neira ; Ian Reid ; John J. Leonard, “Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age”, IEEE Transactions on RoboticsYear: 2016, Volume: 32, Issue: 6Pages: 1309 - 1332
* Jamiruddin, Redhwan, et al. "Rgb-depth slam review." arXiv preprint arXiv:1805.07696 (2018).
* Zollhöfer, Michael, et al. "State of the Art on 3D Reconstruction with RGB‐D Cameras." Computer graphics forum. Vol. 37. No. 2. 2018.

☠︎ SLAM Backbone (Back-End)
* Kalman Filter Family
  * ```Kalman Filter (KF), Extend Kalman Filte (EKF), Unscented Kalman Filte (UKF)```
  * ```Extended Information Filter (EIF), Sparse Extended Information Filter (SEIF)```
* Particle Filter
  * ```Gmapping```: Grisettiyz, Giorgio, Cyrill Stachniss, and Wolfram Burgard. "Improving grid-based slam with rao-blackwellized particle filters by adaptive proposals and selective resampling." Proceedings of the 2005 IEEE international conference on robotics and automation. IEEE, 2005.
  * ```FastSLAM```: Montemerlo, Michael, et al. "FastSLAM: A factored solution to the simultaneous localization and mapping problem." Aaai/iaai 593598 (2002).
  * ```FastSLAM 2.0```: Montemerlo, Michael, et al. "FastSLAM 2.0: An improved particle filtering algorithm for simultaneous localization and mapping that provably converges." IJCAI. 2003.
* Graph Optimization
  * Regression: ```Gaussian Netwon (GN), Leverberg Marquert(LM)```
  * Efficiently Solving: ```Cholesky Factorization, QR Decomposition, Conjugate Gradients```
  * [Ceres Solver Library](http://ceres-solver.org/): S. Agarwal and M. Keir. "Ceres solver." [online]. Available: http://<span></span>ceres-solver.org/
  * [g2o Library](https://github.com/RainerKuemmerle/g2o): Kümmerle, Rainer, et al. "g 2 o: A general framework for graph optimization." 2011 IEEE International Conference on Robotics and Automation. IEEE, 2011.
  * [GTSAM](https://gtsam.org/): Dellaert, Frank. Factor graphs and GTSAM: A hands-on introduction. Georgia Institute of Technology, 2012.
  * [iSAM](http://people.csail.mit.edu/kaess/isam/): (1)Kaess, M., Ranganathan, A., and Dellaert, F. (2008). iSAM: Incremental smoothing and mapping.IEEE Trans. Robotics, 24(6):1365–1378. (2)Kaess, M., Johannsson, H., Roberts, R., Ila, V., Leonard, J., and Dellaert, F. (2012). iSAM2:Incremental smoothing and mapping using the Bayes tree.Intl. J. of Robotics Research, 31:217–236. (iSAM2 is available as part of the GTSAM)
  * [SLAM++](https://sourceforge.net/p/slam-plus-plus/wiki/Home/): Ila, Viorela, et al. "SLAM++-A highly efficient and temporally scalable incremental SLAM framework." The International Journal of Robotics Research 36.2 (2017): 210-230.
* Learning Based

📐 SLAM Benchmark (Dataset)
* The KITTI Vision Benchmark & Dataset http://www.cvlibs.net/datasets/kitti/
  > Geiger, Andreas, Philip Lenz, and Raquel Urtasun. "Are we ready for autonomous driving? the kitti vision benchmark suite." 2012 IEEE Conference on Computer Vision and Pattern Recognition. IEEE, 2012.
* MIT Stata Center Dataset https://projects.csail.mit.edu/stata/#
  > Fallon, Maurice, et al. "The mit stata center dataset." The International Journal of Robotics Research 32.14 (2013): 1695-1699.
* Radish Dataset http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
  > Howard  and  N.  Roy,  “The  robotics  data  set  repository  (Radish),”2003. [Online]. Available: http://<span></span>radish.sourceforge.net/
* TUM RGB-D SLAM Benchmark & Dataset https://vision.in.tum.de/data/datasets/rgbd-dataset
  > Sturm, Jürgen, et al. "A benchmark for the evaluation of RGB-D SLAM systems." 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
* ICL-NUIM RGB-D Benchmark & Dataset https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html 
  > A. Handa, T. Whelan, J. McDonald, and A. J. Davison, “A bench-mark for rgb-d visual odometry, 3d reconstruction and slam,” inRobotics and automation (ICRA), 2014 IEEE international conferenceon. IEEE, 2014, pp. 1524–1531.
* EuRoC MAV Dataset https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 
  > Burri, Michael, et al. "The EuRoC micro aerial vehicle datasets." The International Journal of Robotics Research 35.10 (2016): 1157-1163.
* Benchmark 
  > R.K ̈ummerle,  B.Steder,  C.Dornhege,  M.Ruhnke,  G.Grisetti, C.Stachniss, and A.Kleiner, "On measuring the accuracy of SLAM algorithms," Autonomous Robots, vol. 27, no. 4, pp. 387–407, 2009.
* Survey Paper 
  > Cai, Ziyun, et al. "RGB-D datasets using microsoft kinect or similar sensors: a survey." Multimedia Tools and Applications 76.3 (2017): 4313-4355.

💡 2D Laser Based SLAM
* Cartographer https://google-cartographer-ros.readthedocs.io/en/latest/
  > Wolfgang Hess ; Damon Kohler ; Holger Rapp ; Daniel Andor, “Real-time loop closure in 2D LIDAR SLAM ”, 2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016, pp. 1271-1278.
* Gmapping http://wiki.ros.org/gmapping
  > G. Grisetti, C. Stachniss and W. Burgard, "Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters," IEEE Transactions on Robotics, vol. 23, no. 1, pp. 34-46, Feb. 2007.
* Hector http://wiki.ros.org/hector_slam
  > S. Kohlbrecher, O. von Stryk, J. Meyer and U. Klingauf, "A flexible and scalable SLAM system with full 3D motion estimation," 2011 IEEE International Symposium on Safety, Security, and Rescue Robotics, Kyoto, 2011, pp. 155-160.
* Karto http://wiki.ros.org/slam_karto
  > Vincent, R., Limketkai, B., & Eriksen, M. (2010, April). Comparison of indoor robot localization techniques in the absence of GPS. In Detection and Sensing of Mines, Explosive Objects, and Obscured Targets XV (Vol. 7664, p. 76641Z). International Society for Optics and Photonics.

📷 3D Visual Based SLAM
* ORB-SLAM https://github.com/raulmur/ORB_SLAM2
  > Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015.  
  > Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188-1197, 2012.  

📸 3D RGB-D/Stereo Based SLAM
* ORB-SLAM2 https://github.com/raulmur/ORB_SLAM2
  > Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics, vol. 33, no. 5, pp. 1255-1262, 2017.
* DVO-SLAM https://github.com/tum-vision/dvo_slam
  > Kerl, Christian, Jürgen Sturm, and Daniel Cremers. "Dense visual SLAM for RGB-D cameras." 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2013.
* RGBDSLAMv2 https://felixendres.github.io/rgbdslam_v2/
  > Endres, Felix, et al. "3-D mapping with an RGB-D camera." IEEE transactions on robotics 30.1 (2013): 177-187.
* RTAB-Map http://introlab.github.io/rtabmap/ | https://github.com/introlab/rtabmap_ros
  > M. Labbé and F. Michaud, “RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation,” in Journal of Field Robotics, vol. 36, no. 2, pp. 416–446, 2019. (Wiley) Universit ́e de Sherbrooke  
  > M. Labbé and F. Michaud, “Long-term online multi-session graph-based SPLAM with memory management,” in Autonomous Robots, vol. 42, no. 6, pp. 1133-1150, 2018.  
  > M. Labbé and F. Michaud, “Online Global Loop Closure Detection for Large-Scale Multi-Session Graph-Based SLAM,” in Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014.  
  > M. Labbé and F. Michaud, “Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation,” in IEEE Transactions on Robotics, vol. 29, no. 3, pp. 734-745, 2013.  
  > M. Labbé and F. Michaud, “Memory management for real-time appearance-based loop closure detection,” in Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, 2011, pp. 1271–1276.  
* KinectFusion https://www.microsoft.com/en-us/research/project/kinectfusion-project-page/
  > Izadi, Shahram, et al. "KinectFusion: real-time 3D reconstruction and interaction using a moving depth camera." Proceedings of the 24th annual ACM symposium on User interface software and technology. ACM, 2011.
  > Newcombe, Richard A., et al. "Kinectfusion: Real-time dense surface mapping and tracking." ISMAR. Vol. 11. No. 2011. 2011.
* ElasticFusion https://github.com/mp3guy/ElasticFusion
  > Whelan, Thomas, et al. "ElasticFusion: Dense SLAM without a pose graph." Robotics: Science and Systems, 2015.
* BundleFusion http://graphics.stanford.edu/projects/bundlefusion/
  > Dai, Angela, et al. "Bundlefusion: Real-time globally consistent 3d reconstruction using on-the-fly surface reintegration." ACM Transactions on Graphics (ToG) 36.3 (2017): 24.
* KO-Fusion https://www.youtube.com/watch?v=yigoIYoY7Wg (mobile manipulator)
  > Houseago, Charlie, Michael Bloesch, and Stefan Leutenegger. "KO-Fusion: Dense Visual SLAM with Tightly-Coupled Kinematic and Odometric Tracking." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.
* arm-slam  https://www.youtube.com/watch?v=QrFyaxFUs9w (manipulator)
  > M. Klingensmith, S. S. Sirinivasa and M. Kaess, "Articulated Robot Motion for Simultaneous Localization and Mapping (ARM-SLAM)," in IEEE Robotics and Automation Letters, vol. 1, no. 2, pp. 1156-1163, July 2016.
* Dense RGB-D-Inertail
  > Laidlow, Tristan, et al. "Dense RGB-D-inertial SLAM with map deformations." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.  
  > Hsiao, Ming, Eric Westman, and Michael Kaess. "Dense planar-inertial slam with structural constraints." 2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2018.  

🔅 3D LiDAR Based SLAM
* Zebedee https://research.csiro.au/robotics/zebedee/ (handheld device)
  > M. Bosse, R. Zlot and P. Flick, "Zebedee: Design of a Spring-Mounted 3-D Range Sensor with Application to Mobile Mapping," in IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1104-1119, Oct. 2012.
* Kaarta https://www.kaarta.com/ (handheld device)
  > Zhang, Ji, and Sanjiv Singh. "Laser–visual–inertial odometry and mapping with high robustness and low drift." Journal of Field Robotics 35.8 (2018): 1242-1264.
* hdl_graph_slam https://github.com/koide3/hdl_graph_slam
  > Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Advanced Robotic Systems, 2019
* BLAM https://github.com/erik-nelson/blam
  > E. Nelson, BLAM: berkeley localization and mapping, [online]. Available: https://<span></span>github.com/erik-nelson/blam.
* Lego-LOAM https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
  > T. Shan and B. Englot, "LeGO-LOAM: Lightweight and Ground- Optimized Lidar Odometry and Mapping on Variable Terrain," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, 2018, pp. 4758- 4765.
* Cartographer https://google-cartographer-ros.readthedocs.io/en/latest/
  > Wolfgang Hess ; Damon Kohler ; Holger Rapp ; Daniel Andor, “Real-time loop closure in 2D LIDAR SLAM ”, 2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016, pp. 1271-1278.
* IMLS-SLAM
  > Deschaud, Jean-Emmanuel. "IMLS-SLAM: scan-to-model matching based on 3D data." 2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2018.

🐭 Cognitive Related SLAM
* ViTa-SLAM
  > Struckmeier, Oliver, et al. "ViTa-SLAM: A Bio-inspired Visuo-Tactile SLAM for Navigation while Interacting with Aliased Environments." 2019 IEEE International Conference on Cyborg and Bionic Systems (CBS). IEEE, 2019.

🏷 Semantic Related SLAM
* Kimera https://github.com/MIT-SPARK/Kimera
  > A. Rosinol, M. Abate, Y. Chang, L. Carlone. Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping. arXiv preprint arXiv:1910.02490.

# 7_Localization
📌 Localization on 2D Occupancy Grid Map
* AMCL: Adaptive (or KLD-sampling) Monte Carlo Localization: http://wiki.ros.org/amcl
  > S. Thrun, W. Burgard, and D. Fox. Probabilistic Robotics. MIT Press, 2005.
* mrpt_localization: http://wiki.ros.org/mrpt_localization  
  > J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "Optimal Filtering for Non-Parametric Observation Models: Applications to Localization and SLAM", The International Journal of Robotics Research (IJRR), vol. 29, no. 14, 2010.  
  > J. Gonzalez-Jimenez, J.L. Blanco, C. Galindo, A. Ortiz-de-Galisteo, J.A. Fernandez-Madrigal, F.A. Moreno, J. Martinez, "Mobile Robot Localization based on Ultra-Wide-Band Ranging: A Particle Filter Approach", Robotics and Autonomous Systems, vol. 57, no. 5, pp. 496--507, 2009.  

🌲 SLAM Algorithms Support Pure Localization: 
   * ```Cartographer, ORB_SLAM2, RTAB-Map```

# 8_Mapping
📍 Basic Mapping Backbones
* 2D Occupancy Grid Map (Binary/Probability)
* 3D Occupancy Grid Map (Binary/Probability)
* Octomap ```(for collision checking)``` https://octomap.github.io/ 
  * An Efficient Probabilistic 3D Mapping Framework Based on Octrees / 3D Probability Occupancy Grid Map
  > Hornung, Armin & Wurm, Kai & Bennewitz, Maren & Stachniss, Cyrill & Burgard, Wolfram, "OctoMap: An efficient probabilistic 3D mapping framework based on octrees. Autonomous Robots.", Autonomous Robots Journal (2013). 34. 10.1007/s10514-012-9321-0. 

🗺 Basic Mapping Methods
* map_server: http://wiki.ros.org/map_server ```(loading, saving)```
* octomap_server: http://wiki.ros.org/octomap_server ```(loading, saving, mapping)```

📍 Advanced 3D Mapping Backbones
* Surfels
  > Pfister, Hanspeter, et al. "Surfels: Surface elements as rendering primitives." Proceedings of the 27th annual conference on Computer graphics and interactive techniques. 2000.
* Truncated Signed Distance Function (SDF)
  > Curless, Brian, and Marc Levoy. "A volumetric method for building complex models from range images." Proceedings of the 23rd annual conference on Computer graphics and interactive techniques. 1996.
* Truncated Signed Distance Function (TSDF)
  > R. A. Newcombe, S. Izadi, O. Hilliges, D. Molyneaux, D. Kim, A. J.Davison, P. Kohi, J. Shotton, S. Hodges, and A. Fitzgibbon, “Kinect-fusion: Real-time dense surface mapping and tracking,” in Mixed and augmented reality (ISMAR), 2011 10th IEEE international symposiumon, pp. 127–136, IEEE, 2011
* Euclidean Signed Distance Fields (ESDFs) ```(for collision checking)```
  > Ratliff, Nathan, et al. "CHOMP: Gradient optimization techniques for efficient motion planning." 2009 IEEE International Conference on Robotics and Automation. IEEE, 2009.

🗺 Advanced 3D Mapping Methods
* voxblox (ESDF&TSDF based): https://github.com/ethz-asl/voxblox
  > Helen Oleynikova, et al. “Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning”, in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017.
* OpenChisel (TSDF based): https://github.com/personalrobotics/OpenChisel
  > Klingensmith, Matthew, et al. "Chisel: Real Time Large Scale 3D Reconstruction Onboard a Mobile Device using Spatially Hashed Signed Distance Fields." Robotics: science and systems. Vol. 4. 2015.
* DenseSurfelMapping (Surfel based): https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping
  > Wang, Kaixuan, Fei Gao, and Shaojie Shen. "Real-time scalable dense surfel mapping." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

# 9_Navigation
🚗 ROS Navigation Stack (move_base architecture) https://github.com/ros-planning/navigation  
* move_base: http://wiki.ros.org/move_base
* move_base_flex: http://wiki.ros.org/move_base_flex
<img src="http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf.png" width=700>

🚘 Global Planner:  
* ```global_planner, carrot_planner, navfn, sbpl_lattice_planner, srl_global_planner, voronoi_planner```  
* A* (A Star)
  > Hart, Peter E., Nils J. Nilsson, and Bertram Raphael. "A formal basis for the heuristic determination of minimum cost paths." IEEE transactions on Systems Science and Cybernetics 4.2 (1968): 100-107.
* Dijkstra's
  > Dijkstra, Edsger W. "A note on two problems in connexion with graphs." Numerische mathematik 1.1 (1959): 269-271.

🚘 Local Planner:  
* ```dwa_local_planner, teb_local_planner, base_local_planner, eband_local_planner, robotino_local_planner, asr_ftc_local_planner, simple_local_planner```  
* Timed Elastic Band http://wiki.ros.org/teb_local_planner
  > C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.  
  > C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.  
  > C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.  
  > C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.  
  > C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.  
* Dynamic Window Approach http://wiki.ros.org/dwa_local_planner
  > D. Fox, W. Burgard and S. Thrun, "The dynamic window approach to collision avoidance," in IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23-33, March 1997.

🚘 Recovery Behavior:  
* ```rotate_recovery, move_slow_and_clear, stepback_and_steerturn_recovery```  

🏎️ Novel Navigation Strategy 
* MIT AerospaceControlsLab DRL navigation
    > Chen, Yu Fan, et al. "Decentralized non-communicating multiagent collision avoidance with deep reinforcement learning." 2017 IEEE international conference on robotics and automation (ICRA). IEEE, 2017. | https://www.youtube.com/watch?v=BryJ9jeBkbU | https://www.youtube.com/watch?v=PS2UoyCTrSw  
    > Chen, Yu Fan, et al. "Socially aware motion planning with deep reinforcement learning." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017. | https://www.youtube.com/watch?v=CK1szio7PyA  
    > M. Everett, et al. "Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018 | https://www.youtube.com/watch?v=XHoXkWLhwYQ  
* Google AI Research PRM-RL navigation https://ai.googleblog.com/2019/02/long-range-robotic-navigation-via.html
  > A. Faust et al. "PRM-RL: Long-range Robotic Navigation Tasks by Combining Reinforcement Learning and Sampling-Based Planning," 2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, 2018, pp. 5113-5120.  
  > H. L. Chiang, et al. "Learning Navigation Behaviors End-to-End With AutoRL," in IEEE Robotics and Automation Letters, vol. 4, no. 2, pp. 2007-2014, April 2019.  
  > Francis, Anthony, et al. "Long-range indoor navigation with PRM-RL." IEEE Transactions on Robotics (2020).  
* ETHz Autonomous System Lab navigation
  > Pfeiffer, Mark, et al. "Predicting actions to act predictably: Cooperative partial motion planning with maximum entropy models." 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2016. | https://www.youtube.com/watch?v=GPp5mnybm8g | https://www.youtube.com/watch?v=h1rm0BW3eVE  
  > Pfeiffer, Mark, et al. "From perception to decision: A data-driven approach to end-to-end motion planning for autonomous ground robots." 2017 ieee international conference on robotics and automation (ICRA). IEEE, 2017. | https://www.youtube.com/watch?v=ZedKmXzwdgI

🧹 Coverage Navigation (cleaning or weeding robot)
* Survey Paper:
  > Galceran, Enric, and Marc Carreras. "A survey on coverage path planning for robotics." Robotics and Autonomous systems 61.12 (2013): 1258-1276.

# 10_Manipulation
🖐️ ROS Moveit (move_group architecture) https://github.com/ros-planning/moveit.git
<img src="https://moveit.ros.org/assets/images/diagrams/moveit_pipeline.png" width=700>

📚 Planner Library
* Open Motion Planning Library (OMPL): https://ompl.kavrakilab.org/
  * Intro: https://moveit.ros.org/assets/pdfs/2013/icra2013tutorial/OMPLoverview-ICRA2013.pdf
  * Roadmap Based Planner: PRM, PRM*, Lazy-PRM, LazyPRM*
  * Tree Based Planner: RRTConnect (default), RRT, RRT*, T-RRT, Bi-TRRT, LB-TRRT, SBL, STRIDE, KPIECE, B-KPIECE, LB-KPIECE, EST, Bi-EST, Proj-EST, PDST, SPARS, SPARS2
* Search Based Planning Library (SBPL): http://www.sbpl.net/
  * Intro: https://www.cs.cmu.edu/~maxim/files/tutorials/robschooltutorial_oct10.pdf
  * Search Based Planner: ARA*, Anytime D*, R*
* Covariant Hamiltonian Optimiza-tion for Motion Planning (CHOMP)
  * Intro: https://www.ri.cmu.edu/pub_files/2009/5/icra09-chomp.pdf

# 11_Others_Non_Tech_Part
## 11-1_Famous Robotics Related Company
🏬 Robotic Companies
| categories | companies |
| --------   | -------- |
| Research center | Toyota_Research_Institute(TRI), Microsoft_Research, Google_AI, DeepMind, Facebook_Artificial_Intelligence_Research(FAIR), Berkeley_Artificial_Intelligence_Research (BAIR), Nvidia_Research |
| Manipulator | ABB, FANUC, KUKA, YASKAWA, Techman_Robot, HIWIN, Universal_Robots, Innfos |
| Mobile Robot(AGV, base only) | Omron_Robotics, Clearpath_Robotics&OTTO_Motors, Amazon_Robotics(Kiva_System), Yujin_Robotics, ROBOTIS, Fetch_Robotics, GreenTrans, KUKA, iRobot, Pal_Robotics, Robotnik | 
| Service robot(with torso) | Willow_Garage, Softbank_Robotics, Fetch_Robotics, Pal_Robotics, Innfos, Robotnik |  
| Dual Arms | ABB, Rethink_Robotics |
| Humanoid | Boston_Dynamics, Softbank_Robotics, Pal_Robotics, UBTECH_Robotics |
| Quadruped | Boston_Dynamics, Unitree_Robotics, MIT_Cheetah, ANYrobotics(ANYmal), Standford＿Doggo, Innfos |
| Educational Rotbot | Willow_Garage(Pr2), Facebook(pyrobot), ROBOTIS(turtlebot3), Fetch_Robotics, Robotnik(RB-1) |
| Drone | Dji, Tello |
| ROS2.0 | ADLINK(DDS), ROBOTIS(Turtlebot3) |  
| CleaningBot | iRobot, Xiaomi | 
| Gripper | ROBOTIQ, TOYO |  
| Self-Driving Cars | Alphabet_Waymo, Uber_ATG, Apple_Project_Titan, Tesla, Toyota_Research_Institute(TRI) |

## 11-2_Famous Robotics Publications
📝 Top conferences:  
 * IEEE International Conference on Robotics and Automation (ICRA)  
 * IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)  

🏠 Related Societies:

| Society  | Website  | Conferences / Transactions
| -------- | -------- | ------------
| IEEE Robotics and Automation Society (RAS) | https://www.ieee-ras.org/ | https://ras.papercept.net/conferences/scripts/start.pl  
| IEEE Industrial Electronics Society (IES)  | http://www.ieee-ies.org/  | http://www.ieee-ies.org/conferences  
| IEEE Control System Society (CSS)          | http://www.ieeecss.org/   | http://ieeecss.org/conferences/general-information
| IEEE Systems, Man and Cybernetics (SMC)    | https://www.ieeesmc.org/  | https://www.ieeesmc.org/conferences/calendar/
| AAAS Science Robotics                      | https://robotics.sciencemag.org/  | https://www.sciencemag.org/journals/robotics/call-for-papers

🛠 Tools:  
 * Google Scholar H5-Index Rank on Robotics: https://scholar.google.com/citations?view_op=top_venues&hl=en&vq=eng_robotics  
 * Compress pdf online: https://www.pdf2go.com/compress-pdf  

## 11-3_Famous Robotics Competition
🌎 Global:
 * "DARPA Robotics Challenge": https://en.wikipedia.org/wiki/DARPA_Robotics_Challenge
 * "RoboCup": https://en.wikipedia.org/wiki/RoboCup
 * "Amazon Robotics/Picking Challenge": http://amazonpickingchallenge.org/
 * "ICRA Robot Competitions: including lots of competitions would be different every years"
 * "IROS Robot Competitions: including lots of competitions would be different every years"

🇹🇼 Taiwan:
 * "SKS 新光保全智慧型保全機器人競賽": https://www.facebook.com/sksrobot/
 * "PMC 全國智慧機器人競賽 Robot competition": http://www.pmccontest.com/
 * "HIWIN 上銀智慧機械手實作競賽": http://www.hiwin.org.tw/Awards/HIWIN_ROBOT/Original.aspx
 * "SiliconAwards 旺宏金矽獎"http://www.mxeduc.org.tw/SiliconAwards/

## 11-4_Famous ROS Organizations & Activities
🚀 ROS Related Work:
 * "ROS-industrial": https://rosindustrial.org/
 * "ROS2.0": https://design.ros2.org/
 * "ROS-H": https://acutronicrobotics.com/technology/H-ROS/"

🏢 Organizations/Communities:
 * "Open Source Robotics Foundation (OSRF)": https://www.openrobotics.org/
 * "Open Source Robotics Corporation (OSRC)": https://www.openrobotics.org/
 * "ROS.Taiwan": https://www.facebook.com/groups/ros.taiwan/
 * "ROS.Taipei": https://www.facebook.com/groups/ros.taipei/

🎪 Activities: 
 * "ROScon": https://roscon.ros.org/
 * "ROSDevCon": http://www.theconstructsim.com/ros-developers-online-conference-2019-rdc-worldwide/
 * "ROS Summer School(CN)": http://www.roseducation.org/

-----

## License

[![CC0](http://i.creativecommons.org/p/zero/1.0/88x31.png)](http://creativecommons.org/publicdomain/zero/1.0/)
