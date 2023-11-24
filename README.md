# BebopNoetic
``` bash
mkdir -p bebop_ws/src && cd bebop_ws/src
git clone https://github.com/Gfernandes10/BebopNoetic.git
```
``` bash
sudo apt update
sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros 
sudo apt-get install libavahi-client-dev
sudo apt install ros-noetic-joy ros-noetic-joy-teleop ros-noetic-teleop-twist-joy
```
Add this line in your bash.rc 
``` bash
source ~/bebop_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/bebop_ws
```

To run the simulator use
``` bash
roslaunch Interface_Bebop bebopgazebo.launch
```
