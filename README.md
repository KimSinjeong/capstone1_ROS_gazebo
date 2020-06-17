# Team E ME400 Final Code

## 0. Requirements
* Required Ubuntu 18.04 Bionic on amd64 architecture system

* Install ROS Melodic
```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
chmod 755 ./install_ros_melodic.sh
bash ./install_ros_melodic.sh
```

* Install additional dependencies
```
sudo apt install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

* Restart your machine

`sudo shutdown -r now`

* Move to catkin workspace

`cd catkin_ws/src`

* Install and initialize git

```
sudo apt install git
git config --global user.name <YOURNAME>
git config --global user.email <YOUREMAIL>
git init
```

## 1. Download

* Download repository

`git clone https://github.com/KimSinjeong/capstone1_ROS_gazebo.git`

## 2. Build

* Build the packages

`catkin_make`

## 3. Run

* Run all components at once
```
roslaunch data_integrate system_run.launch
```

* Run only ball harvesting part
```
roslaunch data_integrate ball_harvesting_run.launch
```

## For Evaluation of Vibration (Dedicated for ME400 TAs)

* IMU topic is `/imu_test`