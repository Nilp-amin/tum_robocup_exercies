import rospy
import smach
import smach_ros
import moveit_commander
import random
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
import numpy as np

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import (Constraints, 
                             PositionConstraint, 
                             BoundingVolume, 
                             PlanningScene,
                             CollisionObject,
                             AttachedCollisionObject)

# define states
class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded'], output_keys=["navGoalInd", "targetMarker"])
        # define publisher to move the robots head down to look at the table
        self.head_pub = rospy.Publisher("/head_controller/command", JointTrajectory, queue_size=10)

        # define the goal object to pick up
        self.goal_object = "cup"

    def _look_down(self):
        # Set the desired joint angles (in radians) to look down
        head_joint_traj = JointTrajectory()
        head_joint_traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.75]
        point.time_from_start = rospy.Duration(1.0)
        head_joint_traj.points.append(point)

        # publish goal position and wait for the head to reach the goal position
        self.head_pub.publish(head_joint_traj)
        rospy.sleep(2.0)

    def _look_up(self):
        # Set the desired joint angles (in radians) to look down
        head_joint_traj = JointTrajectory()
        head_joint_traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = rospy.Duration(1.0)
        head_joint_traj.points.append(point)

        # publish goal position and wait for the head to reach the goal position
        self.head_pub.publish(head_joint_traj)
        rospy.sleep(2.0)

    # replace this part by your method
    def execute(self, userdata):
        rospy.loginfo('Executing state FindTargat')

        # make Tiago look down at table
        rospy.loginfo("looking down")
        self._look_down()

        status = "aborted"
        # check if the object is detected
        # if object is detected, store the centroid of the detected object in userdata
        # set the return status to succeeded

        try:
            marker_array_msg = rospy.wait_for_message("/text_markers", MarkerArray, timeout=1.0)
            rospy.loginfo(f"Found {len(marker_array_msg.markers)} objects in vision.")
            for marker in marker_array_msg.markers:
                print(marker.text)
                if marker.text == self.goal_object:
                    rospy.loginfo(f"Found target object: {self.goal_object}.")
                    userdata.targetMarker = marker
                    status = "succeeded"
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout reached. No message received within 5 seconds. Error: {e}")


        # move the head up  and return the status
        if status != "succeeded":
            rospy.loginfo("looking up")
            self._look_up()

        return status

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=["navGoalInd", "targetMarker"])
        # moveit group planning interface
        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(10.0)
        # self.move_group.set_goal_orientation_tolerance(np.deg2rad(5))
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

        # create publisher for moving the gripper
        self.gripper_cmd_pub = rospy.Publisher("/parallel_gripper_controller/command", JointTrajectory, queue_size=1)

        # set the names of the gripper joints
        self.left_gripper_name = "gripper_left_finger_joint"
        self.right_gripper_name = "gripper_right_finger_joint"

    def _create_planning_scene(self, marker_pose):
        # add box into planning scene for table
        table_verties_msg = rospy.wait_for_message("/table_vertices", PointCloud2, timeout=5.0)
        pc_data = pc2.read_points_list(table_verties_msg, field_names=("x", "y", "z"), skip_nans=True)
        
        min_x, min_y, min_z = pc_data[0]
        max_x, max_y, max_z = pc_data[1]

        # Define the box dimensions
        # box_size = [max_x - min_x, max_y - min_y, max_z - min_z]
        box_size = [max_x - min_x, max_y - min_y, 0.05]

        # Define the pose of the box
        box_pose = PoseStamped()
        box_pose.header.frame_id = table_verties_msg.header.frame_id 
        box_pose.pose.position.x = (max_x + min_x) / 2
        box_pose.pose.position.y = (max_y + min_y) / 2
        box_pose.pose.position.z = (max_z + min_z) / 2
        box_pose.pose.orientation.w = 1.0

        # Add the box to the planning scene using PlanningSceneInterface
        self.planning_scene.add_box("obstacle_box", box_pose, size=box_size)

        # add box into planning scene for object 
        box_size = [0.08, 0.08, 0.08]
        box_pose.pose.position.x = marker_pose.x
        box_pose.pose.position.y = marker_pose.y
        box_pose.pose.position.z = marker_pose.z - 0.1
        self.planning_scene.add_box("object_box", box_pose, size=box_size)

    def _move_home_pose(self):
        pose_goal_msg = PoseStamped()
        pose_goal_msg.header.stamp = rospy.Time.now() 
        pose_goal_msg.header.frame_id = "base_footprint" 
        quaternion = quaternion_from_euler(1.055, -1.477, 2.09) 
        pose_goal_msg.pose.orientation.x = quaternion[0] 
        pose_goal_msg.pose.orientation.y = quaternion[1] 
        pose_goal_msg.pose.orientation.z = quaternion[2] 
        pose_goal_msg.pose.orientation.w = quaternion[3] 
        pose_goal_msg.pose.position.x = 0.148 
        pose_goal_msg.pose.position.y = 0.16 
        pose_goal_msg.pose.position.z = 0.61 
        self.move_group.set_pose_target(pose_goal_msg)

        rospy.loginfo(f"planning and executing movement to home pose")
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo(f"planning and execution result: {success}")

    def _grasp_object(self, joint_angles: list):
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.joint_names = [self.left_gripper_name, self.right_gripper_name]

        p = JointTrajectoryPoint()
        p.positions = joint_angles 
        p.time_from_start = rospy.Duration(0.2)

        joint_traj_msg.points.append(p)
        self.gripper_cmd_pub.publish(joint_traj_msg)

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')
        marker = userdata.targetMarker
        marker_pos = marker.pose.position

        pose_goal_msg = PoseStamped()
        pose_goal_msg.header.stamp = marker.header.stamp
        pose_goal_msg.header.frame_id = marker.header.frame_id
        # quaternion = quaternion_from_euler(1.5707, 0.0, 0.0)
        quaternion = quaternion_from_euler(3.1415, -1.5707, 0.0, axes="szyx")
        pose_goal_msg.pose.orientation.x = quaternion[0] 
        pose_goal_msg.pose.orientation.y = quaternion[1] 
        pose_goal_msg.pose.orientation.z = quaternion[2] 
        pose_goal_msg.pose.orientation.w = quaternion[3] 
        # pose_goal_msg.pose.position.x = marker_pos.x - 0.2
        pose_goal_msg.pose.position.x = marker_pos.x + 0.035
        pose_goal_msg.pose.position.y = marker_pos.y
        # pose_goal_msg.pose.position.z = marker_pos.z + 0.15
        pose_goal_msg.pose.position.z = marker_pos.z + 0.35
        self.move_group.set_pose_target(pose_goal_msg)


        self._create_planning_scene(marker_pos)

        # # now calling planner to exectue movement
        plan_success, plan, _, _ = self.move_group.plan()

        # Execute the trajectory
        if plan_success:
            self.move_group.execute(plan, wait=True)
            rospy.loginfo("Planning and execution successful. Moved to the goal pose.")
        else:
            rospy.logwarn("Planning failed or execution error. The goal pose may be unreachable.")
        self.move_group.clear_pose_targets()

        self.planning_scene.clear()
        self.move_group.shift_pose_target(2, -0.22)
        self.move_group.go(wait=True)

        # grasp the object
        self._grasp_object([0.0, 0.0])
        rospy.sleep(1.0)

        self.move_group.shift_pose_target(2, 0.05)
        self.move_group.go(wait=True)

        return 'succeeded'

class Drop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"], input_keys=["navGoalInd"])
        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(10.0)
        self.move_group.set_goal_orientation_tolerance(np.deg2rad(5))

        # create publisher for moving the gripper
        self.gripper_cmd_pub = rospy.Publisher("/parallel_gripper_controller/command", JointTrajectory, queue_size=1)

        # set the names of the gripper joints
        self.left_gripper_name = "gripper_left_finger_joint"
        self.right_gripper_name = "gripper_right_finger_joint"

    def _move_home_pose(self):
        pose_goal_msg = PoseStamped()
        pose_goal_msg.header.stamp = rospy.Time.now() 
        pose_goal_msg.header.frame_id = "base_footprint" 
        quaternion = quaternion_from_euler(1.055, -1.477, 2.09) 
        pose_goal_msg.pose.orientation.x = quaternion[0] 
        pose_goal_msg.pose.orientation.y = quaternion[1] 
        pose_goal_msg.pose.orientation.z = quaternion[2] 
        pose_goal_msg.pose.orientation.w = quaternion[3] 
        pose_goal_msg.pose.position.x = 0.148 
        pose_goal_msg.pose.position.y = 0.16 
        pose_goal_msg.pose.position.z = 0.61 
        self.move_group.set_pose_target(pose_goal_msg)

        rospy.loginfo(f"planning and executing movement to home pose")
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo(f"planning and execution result: {success}")

    def _grasp_object(self, joint_angles: list):
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.joint_names = [self.left_gripper_name, self.right_gripper_name]

        p = JointTrajectoryPoint()
        p.positions = joint_angles 
        p.time_from_start = rospy.Duration(1.0)

        joint_traj_msg.points.append(p)
        self.gripper_cmd_pub.publish(joint_traj_msg)

    def execute(self, userdata):
        rospy.loginfo("Executing Drop Off state.")
        rospy.loginfo(f"Dropping object off at table {userdata.navGoalInd}")

        # release object
        self._grasp_object([0.15, 0.15])
        rospy.sleep(1.0)

        # move arm to home pose
        self.move_group.shift_pose_target(2, 0.2)
        self.move_group.go(wait=True)
        self.move_group.shift_pose_target(0, -0.2)
        self.move_group.go(wait=True)
        self._move_home_pose()

        return "succeeded"
    pass


# main
def main():
    rospy.init_node('smach_example_state_machine')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    # Open the container
    with sm:

        # Navigation callback
        def nav_cb(userdata, goal):
            navGoal = MoveBaseGoal()
            navGoal.target_pose.header.frame_id = "map"
            if userdata.navGoalInd == 1:
                rospy.loginfo('Navagate to table one')
                waypoint = rospy.get_param('/way_points/table_one')
                userdata.navGoalInd = 2
            elif userdata.navGoalInd == 2:
                rospy.loginfo('Navagate to table two')
                waypoint = rospy.get_param('/way_points/table_two')
                userdata.navGoalInd = 1
            navGoal.target_pose.pose.position.x = waypoint["x"]
            navGoal.target_pose.pose.position.y = waypoint["y"]
            navGoal.target_pose.pose.orientation.z = waypoint["z"]
            navGoal.target_pose.pose.orientation.w = waypoint["w"]
            return navGoal

        
        # Add states to the container and define the trasitions
        # Navigate to user defined waypoint with callback
        smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_ONE',
                                            'aborted':'aborted'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_TWO',
                                            'aborted':'aborted'})

        smach.StateMachine.add('FIND_TARGET_ON_TABLE_ONE', FindTargat(),
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_TWO'})
        smach.StateMachine.add('FIND_TARGET_ON_TABLE_TWO', FindTargat(), 
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_ONE'})
        smach.StateMachine.add('GRASP', Grasp(), 
                                transitions={"succeeded":"DROP_OFF"})
        smach.StateMachine.add("DROP_OFF", smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':"DROP",
                                            'aborted':'aborted'})
        smach.StateMachine.add("DROP", Drop(),
                                transitions={"succeeded":"succeeded"})
    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()