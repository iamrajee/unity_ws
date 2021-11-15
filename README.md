# =================== Download Unity3D ==================
## Following this tutorial: https://linuxhint.com/install-unity-2020-2-1f1-ubuntu-20-04/  
Download Unity Hub (UnityHubBeta.tar.gz)  
tar -xzf UnityHubBeta.tar.gz  
mv cd Unity\ Hub/ Unity-Hub/  
cd Unity-Hub/  

chmod +x *.sh  
./INSTALL.sh  

Open UnityHub from application  
add and install latest LTS editor  
mkdir -p ~/Unity/Projects  

# =================== Setup Unity workspace ===============
mkdir -p unity_ws/src  
cd unity_ws/src  
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git  

### Make the un-used recouring package private or move somewhere else
cd Unity-Robotics-Hub/tutorials/ros_unity_integration/  
mv ros2_packages/ .ros2_packages/  
cd Unity-Robotics-Hub/tutorials/pick_and_place/  
mv PickAndPlaceProject/ ~/Unity/Projects/PickAndPlaceProject/  

catkin_make  
./rosdep.sh  
catkin_make  
sudo apt-get install ros-noetic-moveit-commander  
sudo apt install ros-noetic-moveit-simple-controller-manager  

# ===================== Running tutorial ==================
## Following these tutorials: https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials  

1. Add Urdf importer and other package to unity: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md  
2. Follow below Pick&Tutorials(Part1,Part2,Part3): https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md  

Part 1: Importing urdf and setting up unity scene  
Part 2: Integrating ROS with Unity, by adding TCP connection, publisher & subscriber  
        roslaunch niryo_moveit part_2.launch
Part 3: Running Pick and Place demo  
        roslaunch niryo_moveit part_3.launch
Part 4: Hardware implementation (ignore!)  



# =============== Unity-Technologies/Robotics-Object-Pose-Estimation ===============

cd unity_ws/src   
git clone git clone --recurse-submodules https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation.git    
./rosdep.sh  

### Make the un-used recouring package private or move somewhere else
cd Robotics-Object-Pose-Estimation/ROS/src/  
mv moveit_msgs/ .moveit_msgs/  
mv ros_tcp_endpoint/ .ros_tcp_endpoint/  
cd Robotics-Object-Pose-Estimation/ 
mv PoseEstimationDemoProject/ ~/Unity/Projects/PoseEstimationDemoProject/  

Download model from https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation/releases/download/v0.0.1/UR3_single_cube_model.tar  
cd ur3_moveit/  
mkdir models  
cd models  
mv ~/Downloads/UR3_single_cube_model.tar .  

catkin_make  