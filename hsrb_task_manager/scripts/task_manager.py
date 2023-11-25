import rospy
import smach
import smach_ros
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from visualization_msgs.msg import MarkerArray

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
        point.positions = [0.0, -0.8]
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
            marker_array_msg = rospy.wait_for_message("/text_markers", MarkerArray, timeout=5.0)
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
        rospy.loginfo("looking up")
        self._look_up()

        return status


        if random.random() < 0.9:
            rospy.loginfo('Target not found')
            return 'aborted'
        else:
            rospy.loginfo('Target found')
            return 'succeeded'
        
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')
        return 'succeeded'


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
                                transitions={'succeeded':'succeeded'})
    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()