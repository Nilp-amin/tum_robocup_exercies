# Authors
1. Nilp Amin (Matrikel-Nr: 03784634)

# Instructions
All the below exercises are compiled using the below instructions. It is assumed that darknet_ros is installed in a ROS workspace called **catkin_ws**. This is also where these packages should be placed, as it depends on darknet_ros to compile and function. It is then assumed that there is a seperate workspace with the files from the Tiago docker workspace, e.g. **tiago_public_ws**.


1. Navigate to the home directory of the ROS workspace where these packages and darknet_ros is located.

```
cd ~/catkin_ws
```
2. Build the workspace. 
```
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
```
3. Source the **tiago_public_ws** first.
```
. <path-to-tiago-public-ws>/devel/setup.bash
```
4. Source this ROS workspace now.
```
. ./devel/setup.bash --extend
```

Now, once all the above commands are executed, the below exercises can be ran.

## Exercise 1, 2, & 3: Object Segementation and Classifying 
**Robot used:** Tiago

**Required packages:** 
1. darknet_ros
2. object_detection_world
3. object_labeling
4. plane_segmentation
5. All packages from Tiago docker image

Since the objective of this tutorial was to segement and classify objects seen by the Tiago robot in simulation. Only the instructions to run the last task is provided, which incorporates all the functionalities of the previous tasks. So to start off, open 4 terminal windows, which will be referred to as; A, B, C, and D.

**NOTE: It is assumed that the initial setup steps were conducted and that the sourcing of the directories was conducted exactly like that shown in the instructions above first.**

1. Navigate to the home directory of this ROS workspace in terminal A and run the below commands. Make sure the ROS workspaces have already been sourced exactly as shown above.
```
roslaunch object_detection_world tiago.launch world_suffix:=table_v3
```
2. In terminal B run the below commands. Make sure the ROS workspaces have already been sourced exactly as shown above.
```
roslaunch plane_segmentation plane_segmentation_tiago.launch
```
3. In terminal C run the below commands. Make sure the ROS workspaces have already been sourced exactly as shown above.
```
roslaunch object_detection_world object_detection.launch
```
4. In terminal D run the below commands. Make sure the ROS workspaces have already been sourced exactly as shown above.
```
roslaunch object_labeling object_labeling.launch
```

After running the above steps, the Gazebo and RViZ envrionments should appear, with the objects being detected shown with labels in RViZ.