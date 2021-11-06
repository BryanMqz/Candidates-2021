#!/usr/bin/env python3
import rospy
from system_stimuli.srv import FK, FKResponse
from moveit_msgs.msg import Grasp

def handle_newGrasp(req):
                    print("\nAdjusting arm goal\n")
                    return FKResponse(req)
def postGrasp():
    rospy.init_node('FK_server')
    s = rospy.Service('FK', FK, handle_newGrasp)
    print(" Ready to execute Forward Kinematics by service\n")
    rospy.spin()

if __name__ == "__main__":
    postGrasp()