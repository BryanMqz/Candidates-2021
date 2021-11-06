#!/usr/bin/env python3
# license removed for brevity
import os
import rospy
import time
import sys
import random
import numpy as np
#Brings in the SimpleActionClient
import actionlib
#Brings in the messages
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import Grasp
from std_msgs.msg import UInt16
import system_stimuli.msg
from system_stimuli.srv import FK

def arm_movement(stat):
    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('arm_server', system_stimuli.msg.armAction)
    #Waits until the action server has started up and started listening for goals.
    print('\nWaiting for arm server: \n')
    client.wait_for_server()
    #Creates a goal to send to the action server.
    grasp = Grasp()
    grasp.id = "ARM_ACTION"
    print('Grasp result: \n', grasp)
    goal = system_stimuli.msg.armGoal(grasp)
    #Sends the goal to the action server
    client.send_goal(goal)
    
    # Client-Service of FK continues here
    def postGrasp():
        print('\nWaiting for FK service: \n')
        #rospy.wait_for_service('FK')
        try:
            fk = rospy.ServiceProxy('FK', FK)
            grasp.post_grasp_retreat.desired_distance = np.float32(random.uniform(1.5, 15.0))
            grasp.post_grasp_retreat.direction.vector.x = np.float64(random.uniform(1.0, 15.0))
            grasp.post_grasp_retreat.direction.vector.y = np.float64(random.uniform(1.0, 15.0))
            grasp.post_grasp_retreat.direction.vector.z = np.float64(random.uniform(1.0, 15.0))
            respl = grasp
            print('FK successfully done\nPost Grasp Retreat Modificated')
            turn = 'rest'
            return respl
        except rospy.ServiceException as e:
            print("FK Service call failed: %s"%e)
    postGrasp()

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("\nArm Movement: ",result, "\nNext arm goal is:\n")
    print(" Desired distance: ",grasp.post_grasp_retreat.desired_distance, "\n")
    print(" X Axis direction: ",grasp.post_grasp_retreat.direction.vector.x, "\n")
    print(" Y Axis direction: ",grasp.post_grasp_retreat.direction.vector.y, "\n")
    print(" Z Axis direction: ",grasp.post_grasp_retreat.direction.vector.z, "\n")

def neck_movement(stat):
    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('neck_server', system_stimuli.msg.neckAction)

    #Waits until the action server has started up and started listening for goals.
    print('\nWaiting for neck server: \n')
    client.wait_for_server()
    #Creates a goal to send to the action server.
    quat = Quaternion()
    if stat == "free":
        quat.x = np.float64(random.uniform(0.0, 1.0))
        quat.y = np.float64(random.uniform(0.0, 1.0))
        quat.z = np.float64(random.uniform(0.0, 1.0))
        quat.w = np.float64(random.uniform(0.0, 1.0))
    if stat == "secure":#Safe position after receiving an ENcountered error
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0
    print('Quat result: \n', quat)
    goal = system_stimuli.msg.neckGoal(quat)
    #Sends the goal to the action server
    client.send_goal(goal)

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("\nNeck Final Orientation: ",result)


def elevator_movement(stat):
    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('elevator_server', system_stimuli.msg.elevatorAction)
    #Waits until the action server has started up and started listening for goals.
    print('\nWaiting for elevator server: \n')
    client.wait_for_server()
    #Creates a goal to send to the action server.
    p = Pose()
    if stat == "free":
        p.position.x = np.float64(random.uniform(0.0, 1.0))
        p.position.y = np.float64(random.uniform(0.0, 1.0))
        p.position.z = np.float64(random.uniform(0.0, 1.0))
        p.orientation.x = np.float64(random.uniform(0.0, 1.0))
        p.orientation.y = np.float64(random.uniform(0.0, 1.0))
        p.orientation.z = np.float64(random.uniform(0.0, 1.0))
        p.orientation.w = np.float64(random.uniform(0.0, 1.0))
    if stat == "secure":#Safe position after receiving an ENcountered error
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
    print('Pose result: \n', p)
    goal = system_stimuli.msg.elevatorGoal(p)
    #Sends the goal to the action server
    client.send_goal(goal)

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("\nElevator Final Position: ",result)

def callback(data):
    #Definition of data
    # 0 - 'System is stable'
    # 1 - 'System encountered an error'
    print('\nSystem status received')
    print('Data= ',data.data)
    if data.data == 0:
        rospy.loginfo('\nSystem is stable\n')
    elif data.data == 1:
        rospy.loginfo('\nSystem encountered an error\n')
        safeCond = "secure" #Place all mechanisms in their safe position
        arm_movement(safeCond)
        neck_movement(safeCond)
        elevator_movement(safeCond)
        print("*******MECHANISM IN SECURE POSITION********")
        print("*******SHUTDOWN OCCURRED********")
        sys.exit()
    else:
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('mechanical_client', anonymous=True)
        rospy.Subscriber('system_health', UInt16, callback)
        Cond = "free" #free condition to place movement wherever is requested
        while not rospy.is_shutdown():
            arm_movement(Cond)
            neck_movement(Cond)
            elevator_movement(Cond)
            time.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("\nCtrl-C caught. Quitting\n")