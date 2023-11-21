# BebopNoetic
``` bash
mkdir -p bebop_ws/src && cd bebop_ws/src
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
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path_to_your_catkin_ws>/devel/lib/parrot_arsdk
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/gabriel/wsbebop/src/BebopNoetic
```

Source your workspace
``` bash
source devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
```


To run the simulator use
``` bash
roslaunch Interface_Bebop bebopgazebo.launch
```
