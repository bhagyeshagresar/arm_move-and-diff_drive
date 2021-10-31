import rospy
import geometry_msgs
from geometry_msgs.msg import Twist, TransformStamped, Pose, Vector3
import tf2_ros
import tf_conversions
from std_srvs.srv import Empty, EmptyResponse, SetBool
from tf_conversions import transformations



class Flip:

    def __init__(self):
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity = Twist()
        rospy.init_node("flip")
        self.rate = rospy.Rate(20)

        
    




    def main_loop(self):
        while not rospy.is_shutdown():
            t = rospy.get_time()
            while (t < 5.0):
                self.velocity.linear = Vector3(-0.5, 0, 0)
                self.velocity.angular = Vector3(0, 0, 0)
                self.velocity_pub.publish(self.velocity)
                self.rate.sleep()


        


if __name__ == "__main__":

    n = Flip()
    n.main_loop()
    rospy.spin()
