from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))



from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Mover:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mover", anonymous=True)
        self.reset = rospy.Service('reset', Empty, self.reset_fn)



        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=20,)

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")


        group_name = "interbotix_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)


        self.table_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names




    def add_box(self, timeout=4):
        
        table_name = self.table_name
        scene = self.scene

        
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.orientation.w = 0
        table_pose.pose.position.z = 0  # above the panda_hand frame
        table_name = "table"
        scene.add_box(table_name, table_pose, size=(0.2, 0.02, 0.02))
        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.table_name = table_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    
    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = rosparam.load("~/waypoints.yaml")

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction



    def reset_fn(self, req):
        rospy.loginfo("reset is working")
