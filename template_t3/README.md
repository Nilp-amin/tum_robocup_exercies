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

## Exercise 1 & 2: 
**Robot used:** Tiago

**Required packages:** 
1. tiago_localization and associated map files
2. All packages from Tiago docker image

Since exercise 1 and 2 are linked through the map creation procedure, the commands for launch exercise 2 is provided, from which the output of exercise 1 can be viewed. Open two terminal windows.

1. Navigate to the home directory of the ROS workspace in terminal A
```
cd ~/taigo_public_ws
. ./devel/setup.sh
```
2. Execute the following command in terminal A. Two map folders are provided with the submission. They are located at the path relative to the submission folder *template_t3/template/tiago_localization/launch/default_office* or *template_t3/template/tiago_localization/launch/tutorial_office*. 
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=<a-map-folder>
```
3. In terminal B run the below command to start the localization procedure.
```
roslaunch tiago_localization tiago_localization.launch
```

**Note: the localization does not work very well with the self created maps, e.g. default_office and tutorial_office. Use the .pal/tiago_maps/ maps for seeing how well the localization implementation actually works.**

You should be able to see the self scanned map in the RViZ application and also the localization attempts.

## Exercise 3 & 4: 
**Robot used:** Tiago

**Required packages:** 
1. tiago_move and associated map files
2. All packages from Tiago docker image

Since exercise 4 is a build on to exercise 3, only the instructions to run exercise 4 are provided. The behaviour of exercise 4 contains the behaviour of exercise 3 (e.g. moving between navigiation goals indefinitely).

1. Navigate to the home directory of the ROS workspace
```
cd ~/tiago_public_ws
. ./devel/setup.sh
```
2.  Execute the following command to start exercise 3 plus 4 together
```
roslaunch tiago_move tiago_exercise_4.launch
```

The Tiago robot should now move between different navigation goals. The robot does not try to lift the table as it was communicated that this was not required, but instead simply moves the end effector to different poses.