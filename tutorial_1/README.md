# Tutorial 1 - How to run the code
For all instructions, it is assumed the ROS packages are already built. Also, it is assumed that the ROS workspace is called *catkin_ws*.
## Keyboard teleoporation (Exercise 2)
1. Open two terminals and enter the following commands in all terminals:
```
cd ~/catkin_ws
. ./devel/setup.bash
```
2. Type the below in terminal **A** to launch object server node:
```
roslaunch object_server object_server.launch
```
3. Type the below in terminal **B** to launch rover node:
```
roslaunch rover rover_teleop.launch
```
4. Then click on terminal **B** before sending keyboard commands.
## Autonomous control (Exercise 3)
1. Open three terminals and enter the following commands in all terminals:
```
cd ~/catkin_ws
. ./devel/setup.bash
```
2. Type the below in terminal **A** to launch object server node:
```
roslaunch object_server object_server.launch
```
3. Type the below in terminal **B** to launch rover controller node:
```
roslaunch rover_controller rover_controller.launch
```
4. Type the below in terminal **C** to add objects to the world:
```
rosservice call /add_objects <tab-tab>
```
5. Make sure to click on at least one of the obstacles on RViZ, otherwise the feedback service in ROS does not work, and the code will not register the existence of the objects.
6. Set a goal position on RViZ.
