# Authors
1. Nilp Amin (Matrikel-Nr: 03784634)
2. Margaux Edwards (Matrikel-Nr: 03784879)

# Instructions
All the below exercises are compiled using the below instructions. It is assumed that this package is already in the Tiago ROS docker workspace. The below instructions only have to be run once to use all the exercise code.
1. Navigate to the home directory of the ROS workspace
```
cd ~/tiago_public_ws
```
2. Build the ROS packages
```
catkin build
```
3. Source the ROS workspace
```
. ./devel/setup.sh
```
## Exercise 1: Simulation Environment
**Robot used:** Tiago

**Required packages:** 
1. ics_gazebo
2. controllers_tutorials
3. All packages from Tiago docker image

To view the simulation environment created, follow the below commands. Open two terminal windows.

1. Navigate to the home directory of the ROS workspace in terminal A
```
cd ~/taigo_public_ws
. ./devel/setup.sh
```
2. Launch the environment with the robot
```
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2
```
3. Run the following commands in terminal B to control the robot in the environment
```
rosrun key_telop key_telop.py
```
## Exercise 2: Robot in RViz 
**Robot used:** Tiago

**Required packages:**
1. ics_gazebo
2. controllers_tutorials
3. All packages from Tiago docker image

To view the robot in RViz, follow the below commands. Open two terminal windows.

1. Navigate to the home directory of the ROS workspace in terminal A
```
cd ~/taigo_public_ws
. ./devel/setup.sh
```
2. Launch the environment with the robot
```
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2
```
3. Click on the RViz window to see the Taigo robot and its sensor readings as visuals. If nothing is visible, click the *File* tab on the top left, then *Open Config* and look for the rviz configuration at the below location
```
~/tiago_public_ws/src/ics_gazebo/config/tiago.rviz
```
4. Run the following commands in terminal B to control the robot in the environment and see the sensor readings change as it approaches different objects
```
rosrun key_telop key_telop.py
```
## Exercise 3: Custom Controller Plugin
**Robot used:** Tiago

**Required packages:**
1. ics_gazebo
2. controllers_tutorials
3. All packages from Tiago docker image

To check out the new controller, follow the below commands. Open two terminal windows.

1. Navigate to the home directory of the ROS workspace in terminal A
```
cd ~/taigo_public_ws
. ./devel/setup.sh
```
2. Launch the robot in an empty world with the below command
```
roslaunch ics_gazebo tiago.launch
```
3. In terminal B kill the head controller
```
rosrun controller_manager controller_manager kill head_controller
``` 
4. Run the custom controller with the following command
```
roslaunch controllers_tutorials new_head_controller_tiago.launch
```
5. You should see the head of the Tiago robot in Gazebo and RViz move in a sinusoidal pattern