# ROS Workspace for Autonomous Kart Project

This repository contains the ROS workspace for the Autonomous Kart Project.

## Installation Steps:

- It is better to follow the steps using Ubuntu 18.04 LTS or Ubuntu 20.04 LTS. In case you are a windows machine user, you can use Ubuntu 20.04 LTS on Windows 10 using WSL2 or install a virtual machine with Ubuntu 20.04 LTS
- From Ubuntu 20.04, install ROS Melodic or ROS Noetic. Follow the steps from [here](https://linuxopsys.com/topics/install-ros-noetic-on-ubuntu)
- For Mac users, follow the [following video](https://www.youtube.com/watch?v=zF7Pbq4Puvg&amp;ab_channel=BorisMeinardus&ab_channel=BorisMeinardus) to install ROS on your machine natively
- run the following command:
```bash
git clone https://github.com/ahmedmbakr/autonomous_kart_ros_vis.git
cd autonomous_kart_ros_vis
catkin_make
```
- The last command should generate two folders: `build` and `devel`
- run the following command:
```bash
sudo apt-get install ros-noetic-rosbridge-suite
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
- The expected output is the following image:
![rosbridge_server](imgs/rosbridge_command_expected_output.png)
- Download the testrosbag from this link: [testrosbag](https://www.dropbox.com/s/8vwkero6boujtzb/UrbanNav-HK_CHTunnel-20210518_sensors.bag?dl=0). Refer to [this link](https://github.com/IPNL-POLYU/UrbanNavDataset/blob/master/README.md) for the complete sensor setup and rosbag description
- Update line 26 in `talker2.py` with the path to the downloaded rosbag
- Download and install anaconda from [here](https://repo.anaconda.com/archive/Anaconda3-2022.10-Linux-x86_64.sh)
- Install `Anaconda` using the following commands:
```bash
bash ./Anaconda3-2022.10-Linux-x86_64.sh
conda env create -f environment.yaml
conda activate kart_env
```
- Open another terminal and run the following command:
```bash
source devel/setup.bash
rosrun first_package talker2.py
```
- The expected output is the following image:
![rosbridge_server](imgs/talker2_ros_run_expected_output.png)
- Open `foxglove`, choose `Open Connection` and write the following URL: `ws://localhost:9090` under `Rosbridge (Ros1 & 2) tab, as shown in the attached image:
![rosbridge_server](imgs/foxglove_url.png)

## Normal Run:

This section describes how you normally run the program after you have followed the installation steps successfully.

- In a new terminal, execute the following commands:
```bash
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
- In a new terminal, execute the following command to run the rosnode
```bash
cd autonomous_kart_ros_vis
conda activate kart_env
rosrun first_package talker2.py
```

## Known Issues:
- If you face error when running any `sudo` command in Ubunu, refer to the [following video](https://youtu.be/jZGHtuxpaP4) to solve the issue, thanks to Kate Sanborn

## Maintainers
- Ahmed Bakr (ambakr@crimson.ua.edu) or (ahmed.m.ali.bakr@gmail.com)
