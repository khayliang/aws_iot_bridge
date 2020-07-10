# aws_iot_bridge
A ROS Package to bridge the connection between AWS IOT Core and ROSBridge Websocket.

# Installation
Git clone repo into your catkin_ws/src.
```
cd ~/catkin_ws/src
git clone https://github.com/khayliang/aws_iot_bridge.git
```
catkin_make the package.
```
cd ..
catkin_make
source devel/setup.bash
```
TODO: Setup thing and linking with the package
# Usage
Specify the topics and services you would like to expose inside `configs/bridge_config.yaml`. A template is already present for you to modify.

Once done, launch the node together with ROSBridge websocket. That is accomplished by launching the launch file.
```
roslaunch aws_iot_bridge aws_iot_bridge.launch
```

