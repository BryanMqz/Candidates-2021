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
    ##Creates the SimpleActionClient, passing the type of the action
    #client = actionlib.SimpleActionClient('arm_server', system_stimuli.msg.armAction)
    ##Waits until the action server has started up and started listening for goals.
    #print('\nWaiting for arm server: \n')
    #client.wait_for_server()
    ##Creates a goal to send to the action server.
    rospy.loginfo('Arm movement\n')
    grasp = Grasp()
    grasp.id = "ARM_ACTION"
    #print('Grasp result: \n', grasp)
    goal = system_stimuli.msg.armGoal(grasp)
    #Sends the goal to the action server
    armclient.send_goal(goal)
    # Client-Service of FK continues here
    def postGrasp():
        rospy.loginfo('\nWaiting for FK service: \n')
        #rospy.wait_for_service('FK')
        try:
            fk = rospy.ServiceProxy('FK', FK)
            grasp.post_grasp_retreat.desired_distance = np.float32(random.uniform(1.5, 15.0))
            grasp.post_grasp_retreat.direction.vector.x = np.float64(random.uniform(1.0, 15.0))
            grasp.post_grasp_retreat.direction.vector.y = np.float64(random.uniform(1.0, 15.0))
            grasp.post_grasp_retreat.direction.vector.z = np.float64(random.uniform(1.0, 15.0))
            respl = grasp
            rospy.loginfo('FK successfully done\nPost Grasp Retreat Modificated')
            turn = 'rest'
            return respl
        except rospy.ServiceException as e:
            rospy.loginfo("FK Service call failed: %s"%e)
    if stat == "free":
        postGrasp()
    if stat == "secure":#Safe position after receiving an Encountered error
        grasp = Grasp()

    ## Prints out the feedback of executing the action
    fb = armclient.get_state()
    rospy.loginfo("Arm Movement feedback message #"+str(fb)+ "\nNext arm goal is:\n\
    Desired distance: "+str(grasp.post_grasp_retreat.desired_distance)+ "\n\
    X Axis direction: "+str(grasp.post_grasp_retreat.direction.vector.x)+ "\n\
    Y Axis direction: "+str(grasp.post_grasp_retreat.direction.vector.y)+ "\n\
    Z Axis direction: "+str(grasp.post_grasp_retreat.direction.vector.z)+ "\n")

    #The following 2 lines have the same purpose as Shutdown_system
    #data = rospy.Publisher('system_health', UInt16, queue_size=10)
    rospy.loginfo('Also checking robot arm state...')
    if(fb == 3):
        data.publish(1)
    else:
        data.publish(0)
    rospy.loginfo("System status published.")

def neck_movement(stat):
    ##Creates the SimpleActionClient, passing the type of the action
    #client = actionlib.SimpleActionClient('neck_server', system_stimuli.msg.neckAction)

    ##Waits until the action server has started up and started listening for goals.
    #print('\nWaiting for neck server: \n')
    #client.wait_for_server()
    ##Creates a goal to send to the action server.
    rospy.loginfo('Neck movement\n')
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
    rospy.loginfo('\nQuat result: \n'+ str(quat))
    goal = system_stimuli.msg.neckGoal(quat)
    #Sends the goal to the action server
    neckclient.send_goal(goal)

    ## Prints out the feedback of executing the action
    fb = armclient.get_state()
    rospy.loginfo("Neck Feedback message #"+str(fb))

    #Following 2 lines have the same purpose as Shutdown_system
    #data = rospy.Publisher('system_health', UInt16, queue_size=10)
    rospy.loginfo('Also checking robot neck state...')
    if(fb == 2):
        data.publish(1)
    else:
        data.publish(0)
    rospy.loginfo("System status published.")

def elevator_movement(stat):
    ##Creates the SimpleActionClient, passing the type of the action
    #client = actionlib.SimpleActionClient('elevator_server', system_stimuli.msg.elevatorAction)
    ##Waits until the action server has started up and started listening for goals.
    #print('\nWaiting for elevator server: \n')
    #client.wait_for_server()
    ##Creates a goal to send to the action server.
    rospy.loginfo('Elevator movement\n')
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
    rospy.loginfo('\nPose result: \n'+str(p))
    goal = system_stimuli.msg.elevatorGoal(p)
    #Sends the goal to the action server
    elevatorclient.send_goal(goal)

    elevatorclient.wait_for_result()

    ## Prints out the feedback of executing the action
    fb = armclient.get_state()
    rospy.loginfo("Elevator Feedback message #"+str(fb))

    #Following 2 lines have the same purpose as Shutdown_system
    rospy.loginfo('Also checking robot elevator state...')
    if(fb == 2):
        data.publish(1)
    else:
        data.publish(0)
    rospy.loginfo("System status published.")

def callback(data):
    #Definition of data
    # 0 - 'System is stable'
    # 1 - 'System encountered an error'
    rospy.loginfo('System status received')
    rospy.loginfo('Data= '+str(data.data))
    if data.data == 0:
        rospy.loginfo('System is stable\n')
    elif data.data == 1:
        rospy.loginfo('System encountered an error\n')
        rospy.signal_shutdown("System encountered an error")
        safeCond = "secure" #Place all mechanisms in their safe position
        arm_movement(safeCond)
        neck_movement(safeCond)
        elevator_movement(safeCond)
        print("*******MECHANISM IN SECURE POSITION********")
        print("*******SHUTDOWN OCCURRED********")
        sys.exit(1)
    else:
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('mechanical_client', anonymous=True)
        rospy.Subscriber('system_health', UInt16, callback)
        ##Arm Action
        armclient = actionlib.SimpleActionClient('arm_server', system_stimuli.msg.armAction)
        rospy.loginfo("Waiting for arm server") 
        #waitForServer se puede tener antes de entrar al loop, 1 vez por nodo
        armclient.wait_for_server()
        rospy.loginfo("Arm Server ready")

        ##Neck Action
        neckclient = actionlib.SimpleActionClient('neck_server', system_stimuli.msg.neckAction)
        #Waits until the action server has started up and started listening for goals.
        rospy.loginfo('Waiting for neck server')
        neckclient.wait_for_server()
        rospy.loginfo("Neck Server ready")

        ##Elevator Action
        elevatorclient = actionlib.SimpleActionClient('elevator_server', system_stimuli.msg.elevatorAction)
        #Waits until the action server has started up and started listening for goals.
        rospy.loginfo('Waiting for elevator server')
        elevatorclient.wait_for_server()
        rospy.loginfo("Elevator Server ready")

        data = rospy.Publisher('system_health', UInt16, queue_size=10)
        Cond = "free" #free condition to place movement wherever is requested
        while not rospy.is_shutdown():
            arm_movement(Cond)
            neck_movement(Cond)
            elevator_movement(Cond)
            time.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("\nCtrl-C caught. Quitting\n")