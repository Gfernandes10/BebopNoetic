# BebopNoetic
``` bash
mkdir -p bebop_ws/src && cd bebop_ws/src
git clone https://github.com/Gfernandes10/BebopNoetic.git
git clone https://github.com/pal-robotics/aruco_ros.git -b noetic-devel
```
``` bash
sudo apt update
sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros 
sudo apt-get install libavahi-client-dev
sudo apt install ros-noetic-joy ros-noetic-joy-teleop ros-noetic-teleop-twist-joy
pip install filterpy

```
Add this line in your bash.rc 
``` bash
source ~/bebop_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/bebop_ws
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/bebop_ws/devel/lib/parrot_arsdk
```

To run the simulator use
``` bash
roslaunch Interface_Bebop bebopgazebo.launch
```

To run the simulator with aruco tag detection use
``` bash
roslaunch Interface_Bebop bebopgazeboaruco.launch 
```

To connect with the real drone use
``` bash
roslaunch Interface_Bebop bebopreal.launch
```
