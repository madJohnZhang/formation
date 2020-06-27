# UAV Formation Visual Tracking System
### System Overview
The system is to let a number of UAVs with calibrated cameras track the selected target.

![https://github.com/madJohnZhang/formation/blob/master/img/flow.jpg]
System running process.

![https://github.com/madJohnZhang/formation/blob/master/img/structure.jpg]
System structure.

### Getting Started
#### Environment
1. Ubuntu 16.04 LTS
2. ROS kinetic
3. cmake

#### Denpendancy
1. DJI Onboard SDK 3.7 [download](https://developer.dji.com/cn/).
2. serial communication toolbox.
3. libusb.
4. SLAMTEC rplidar ROS SDK [download](https://github.com/slamtec/rplidar_ros).

#### Run
1. use command `catkin_make` to compile the project in the root directory
2. run the init script `initCen.sh` and `init.sh` for the centralized and the decentralized position estimation respectively.
3. start the communication process first and choose the target bounding box after the communication of all the nodes stablize.
