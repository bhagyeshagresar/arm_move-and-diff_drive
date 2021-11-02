
#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from arm_move.srv import Step
from std_srvs.srv import Empty, EmptyResponse






try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))



from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Move:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("mover", anonymous=True)
        #self.waypoints = rospy.get_param("/waypoints")
        self.reset = rospy.Service('reset', Empty, self.reset_fn)
        self.step = rospy.Service('step', Step, self.step_fn)
        self.follow = rospy.Service('follow', Empty, self.follow_fn)
        



        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        

        #display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=20,)

        self.move_group = moveit_commander.MoveGroupCommander("interbotix_arm")

        self.move_group_2 = moveit_commander.MoveGroupCommander("interbotix_gripper")


        


        self.box_name = ""
        self.box_name2 = ""
        #self.robot = robot
        #self.scene = scene
        #self.move_group = move_group

        #self.scene2 = scene2
       

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        
        box_name = self.box_name
        scene = self.scene

        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        rospy.sleep(3)
        
        box_name = self.box_name
        scene = self.scene

        
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "world"
        self.box_pose.pose.orientation.w = 1.0
        self.box_pose.pose.position.x = 0
        self.box_pose.pose.position.y = 0
        self.box_pose.pose.position.z = 0  # above the panda_hand frame
        box_name = "box"
        
        self.box_name = box_name
        while not self.wait_for_state_update(box_is_known=True):
            scene.add_box(box_name,self.box_pose, size=(0.26, 0.26, 0))


    def wait_for_state_update2(self, box_is_known=False, box_is_attached=False, timeout=4):
        
        box_name2 = self.box_name2
        scene = self.scene

        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name2])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name2 in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_box2(self, timeout=4):
        rospy.sleep(3)
        
        box_name2 = self.box_name2
        scene = self.scene

        
        self.box_pose2 = geometry_msgs.msg.PoseStamped()
        self.box_pose2.header.frame_id = "world"
        self.box_pose2.pose.orientation.w = 1.0
        self.box_pose2.pose.position.x = 0
        self.box_pose2.pose.position.y = 0.2
        self.box_pose2.pose.position.z = 0  # above the panda_hand frame
        box_name2 = "box_2"
        
        self.box_name2 = box_name2
        while not self.wait_for_state_update2(box_is_known=True):
            scene.add_box(box_name2,self.box_pose2, size=(0.024, 0.0381, 0.0381))
    

    def reset_fn(self, req):
        rospy.loginfo("reset is working")
        #pose_goal = geometry_msgs.msg.Pose()
        #pose_goal.orientation.w = 1.0
        #pose_goal.position.x = 0.4
        #pose_goal.position.y = 0.1
        #pose_goal.position.z = 0
        #self.scene.remove_world_object(self, name = "box_2")
        #self.scene.add_box(self, name = "box_2", pose = self.pose, size=(0.14, 0.09, 0.05))
        self.move_group.set_named_target("Home")
        self.move_group.go()
        #self.waypoints = []

        #if empty_list == 1:
        #    self.waypoints = []
        #self.move_group.set_named_target("Home")
        #self.move_group.go()
        return EmptyResponse()


    def step_fn(self, req):
        #Take user input
        rospy.loginfo("step is working")
        pose_goal = geometry_msgs.msg.Pose()

        
        pose_goal.position.x = req.x
        pose_goal.position.y = req.y
        pose_goal.position.z = req.z
        pose_goal.orientation.x = req.x1
        pose_goal.orientation.y = req.y1
        pose_goal.orientation.z = req.z1
        pose_goal.orientation.w = req.w
        set_gripper = req.a

        

        
        #set_gripper = req.a
        self.waypoints_list = []
        self.move_group.set_pose_target(pose_goal, self.move_group.get_end_effector_link())
        (a, b, c, d) = self.move_group.plan()

        if d.val == 1:
            
            self.move_group.go(pose_goal)
            self.waypoints_list.append(pose_goal)
            if set_gripper == False:
                self.move_group_2.set_named_target("Open")
                self.move_group_2.go()
            else:
                self.move_group_2.set_named_target("Closed")
                self.move_group_2.go()
            #self.waypoints_list.append(set_gripper)
            #rospy.set_param('/waypoints', self.waypoints_list)
            #self.waypoints.append(pose_goal)
        else:
            self.move_group.stop()

        return 0
        #self.move_group.set_pose_target(pose_goal)
        #lan = self.move_group.go(pose_goal)

        #self.move_group.stop()
        #self.move_group.clear_pose_targets()

        
        #if set_gripper ==1:
        #    self.move_group_2.set_named_target("open")
        #    self.move_group_2.go()
        #else:
        #    self.move_group_2.set_named_target("closed")
        #    self.move_group_2.go()
        #plan the trajectory
        #error_code, _, _, _ = plan()
        #plan = self.move_group.go(wait=True)
        #self.move_group.go(pose_goal)
        #self.move_group.stop()
        #self.move_group.clear_pose_targets()
    
    def follow_fn(self, req):
        waypoints2 = [[0.197, 0.149, 0.1303, 1.4929e-05, -4.4539e-05, 0.31782, 0.94815, False],
        [-0.019251, 0.15409, 0.11174, -0.27583, 0.24351, 0.69707, 0.6154, False],
        [0.13443, 0.17873, 0.012205, 0.6894, 0.34411, -0.57032, 0.2848, True],
        [0.21605, -0.15652, 0.040133, -0.0030176, -0.0093084, -0.30836, 0.95122, False]]
        for i in waypoints2:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = i[0]
            pose_goal.position.y = i[1]
            pose_goal.position.z = i[2]
            pose_goal.orientation.x = i[3]
            pose_goal.orientation.y = i[4]
            pose_goal.orientation.z = i[5]
            pose_goal.orientation.w = i[6]
            set_gripper = i[7]
            self.move_group.go(pose_goal)
            if set_gripper == False:
                self.move_group_2.set_named_target("Open")
                self.move_group_2.go()
            else:
                self.move_group_2.set_named_target("Closed")
                self.move_group_2.go()
        
        return EmptyResponse()
        #self.move_group_2.execute(plan, wait=True)

        #if error_code == 1:
        #    self.waypoints.append(pose_goal)
        #else:
        #    pass
        #self.move_group.go(pose_goal)
        #plan = self.move_group.go(wait=True)

    


    
    
    
    
def main():
    n = Move()
    n.add_box()
    n.add_box2()
    rospy.spin()
    #rospy.Service('reset', Empty, self.reset_fn)


            




if __name__ == "__main__":
    try:
        #waypoints = rospy.get_param("/waypoints")
        main()
    except rospy.ROSInterruptException:
        pass
