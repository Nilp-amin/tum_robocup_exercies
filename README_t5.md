# Authors
1. Nilp Amin (Matrikel-Nr: 03784634)
2. Margaux Edwards (Matrikel-Nr: 03784879)

# Instructions
All the below exercises are compiled using the below instructions. It is assumed that darknet_ros is installed in a ROS workspace called **catkin_ws**. This is also where these packages should be placed, as it depends on darknet_ros to compile and function. The **tmc_wrs_gazebo_world** package should also be in this **catkin_ws**. It is then assumed that there is a seperate workspace with the files from the Tiago docker workspace, e.g. **tiago_public_ws**.


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

Now, once all the above commands are executed, the below exercises can be run.

## Exercise 1, 2, 3, 4 & 5: Integrated System
**Robot used:** Tiago

**Required packages:**
1. darknet_ros
2. object_detection_world
3. object_labeling
4. plane_segmentation
5. tmc_wrs_gazebo_world
6. tiago_localization 
7. All packages from Tiago docker image

Only instructions for the final task, which runs and incorporates all the other tasks is provided below. So to start off, open 4 terminal windows, which be referred to as; A, B, C, and D.

**NOTE: It is assumed that the initial setup steps were conducted and that the sourcing of the directories was conducted exactly like that shown in the instructions above first.**

1. Navigate to the home directory of this ROS workspace in terminal A and run the below commands. Wait for the Tiago arm to be tucked before moving on to next step.
```
roslaunch object_detection_world tiago_wrs.launch world_suffix:=wrs2020 lost:=true
```
2. In terminal B run the below commands to localise Tiago. It takes around 200-300 AMCL iterations to converge. If the script does not exit by then, you can manually ctrl-c the script when the **conv_sum** < 0.008. Re-run this script if the robot gets stuck localising in the incorrect position with no other particles in different locations.
```
roslaunch tiago_localization tiago_localization.launch
``` 
3. In terminal C run the below command to launch the FSM stack.
```
roslaunch tiago_task_manager task_manager.launch
```
4. In terminal D run the below command to launch darknet.
```
roslaunch object_detection_world tiago_cv.launch
```
5. Tiago will oscillate between two tables until it finds a cup to pick up. Tiago will only pick up a cup. Place a cup on either of the two tables such that the darknet is able to detect and identify the cup. The cups to use are in the **tmc_wrs_gazebo_world** package and are labeled as **ycb_065-[a-j]_cups**. You can look at the shared video for greater detail on which tables and cups were being used.

**Note: The darknet package can die sometimes. If the darknet is able to detect the cup but no msg is being published on the topic `/text_markers` then relaunch darknet with step 4. Also, the location of the cup can sometimes not be in the workspace of the robot; hence, the robot will not be able to obtain a solution to the IK of its arm. If this occurs, let the robot reach its final state of dropping the cup off - regardless of picking up the cup or not, at the table (you will know the final state is reached when the robot stops moving and can be seen in terminal C). Then relaunch the FSM stack using step 3.**