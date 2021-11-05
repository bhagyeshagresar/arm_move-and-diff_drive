#!/usr/bin/env python
"""
Flip node causes the robot to move back and forth along a straight line by flipping the robot instead of 
making a turn to move in the opposite direction. The robot is driven using the Gazebo Differential Drive Plugin

Publisher :
topic - cmd_vel  message type - geometry_msgs/Twist



"""



import rospy
import geometry_msgs
from geometry_msgs.msg import Twist, TransformStamped, Pose, Vector3
import tf2_ros
import tf_conversions
from std_srvs.srv import Empty, EmptyResponse, SetBool
from tf_conversions import transformations



class Flip:
    """
    Published geometry_msgs/Twist messages at a fixed rate

    """

    def __init__(self):
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity = Twist()
        rospy.init_node("flip")
        self.rate = rospy.Rate(20)


        
    def move(self, v, t):
        """
        Move function takes velocity 'v' and time 't' as input to publish velocities on topic cmd_vel.
        The robot moves at 5 m/s in the forward direction for 2 secs and then waits and travels in the 
        opposite direction for two seconds.

        Args:
        v : velocity in m/s
        t : time in seconds

        """
        current_time = rospy.Time.now()
        duration = rospy.Duration(t)
        final_time = current_time + duration
        while(rospy.Time.now() < final_time):
            self.velocity.linear = Vector3(v, 0, 0)
            self.velocity.angular = Vector3(0, 0, 0)
            self.velocity_pub.publish(self.velocity)
            self.rate.sleep()
            

        




    def main_loop(self):
        while not rospy.is_shutdown():
            self.move(5, 2)
            self.move(0, 2)
            self.move(-5, 2)
            self.move(0, 2)


        


if __name__ == "__main__":

    n = Flip()
    n.main_loop()
    rospy.spin()
