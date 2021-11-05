#!/usr/bin/env python
""" Create a unittest node that tests the step and reset service of mover node """

import unittest
import rospy
from arm_move.srv import Step, Reset


class ArmTest(unittest.TestCase):
    def __init__(self, *args):
        super(ArmTest, self).__init__(*args)
        rospy.init_node("services_test")

    def test_step_service_fail(self):
        """
        Test to assert failure of step service

        """
        rospy.wait_for_service("step")
        step_object = rospy.ServiceProxy("step", Step)
        error = step_object.call(x =1, y =2, z =3, x1 = 1, y1 = 0, z1 = 0, w = 0, True)
        self.assertEqual(error, -1)
    
    def test_step_service_success(self):
        """
        Test to assert success of step service


        """
        rospy.wait_for_service("step")
        step_object = rospy.ServiceProxy("step", Step)
        error = step_object.call(0.042171, 0.15788, 0.17028, -1.7918e-05, 2.3332e-05, 0.60908, 0.79311, False)
        self.assertEqual(error, 1)
    
    def test_reset_check(self):
        """
        Test to run reset service

        """
        rospy.wait_for_service("reset")
        reset_object = rospy.ServiceProxy("reset", Reset)
        reset_object.call(data = True, x = 0.2, y = 0.2, z = 0.025)


if __name__ == "__main__":
    import rostest
   
    rostest.rosrun('arm_move', "services_test", ArmTest)