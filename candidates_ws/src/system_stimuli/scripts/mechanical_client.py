#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
#Brings in the SimpleActionClient
import actionlib
#Brings in the messages
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import Grasp
import system_stimuli.msg

def arm_movement():
    #pub_arm_movement = rospy.Publisher('arm_movement', Grasp, queue_size=10)
    #grasp = Grasp()
    #pub_arm_movement.publish(grasp)
    #rospy.loginfo("Arm movement stimuli published.")

    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('arm_server', system_stimuli.msg.armAction)
    #Waits until the action server has started up and started listening for goals.
    print('Waiting for arm server: \n')
    client.wait_for_server()
    #Creates a goal to send to the action server.
    grasp = Grasp()
    print('Grasp result: ', grasp)
    goal = system_stimuli.msg.armGoal(grasp)
    #Sends the goal to the action server
    client.send_goal(goal)

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("Arm Movement: ",result)

def neck_movement():
    '''pub_neck_movement = rospy.Publisher('neck_movement', Quaternion, queue_size=10)
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    quat.w = 1.0
    pub_neck_movement.publish(quat)
    rospy.loginfo("Neck movement stimuli published.")
    '''
    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('neck_server', system_stimuli.msg.neckAction)
    #Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    #Creates a goal to send to the action server.
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    quat.w = 1.0
    goal = system_stimuli.msg.neckGoal(quat)
    #Sends the goal to the action server
    client.send_goal(goal)

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("Neck Final Orientation: ",result)


def elevator_movement():
    '''pub_elevator_movement = rospy.Publisher('elevator_movement', Pose, queue_size=10)
    p = Pose()
    p.position.x = 0.0
    p.position.y = 0.4
    p.position.z = 0.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 1.0
    pub_elevator_movement.publish(p)
    rospy.loginfo("Elevator movement stimuli published.")
    '''
    #Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('elevator_server', system_stimuli.msg.elevatorAction)
    #Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    #Creates a goal to send to the action server.
    p = Pose()
    
    p.position.x = 0.0
    p.position.y = 0.4
    p.position.z = 0.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 1.0
    goal = system_stimuli.msg.elevatorGoal(p)
    #Sends the goal to the action server
    client.send_goal(goal)

    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    print("Elevator Final Position: ",result)

if __name__ == '__main__':
    try:
        rospy.init_node('mock_mechanical_client', anonymous=True)
        while not rospy.is_shutdown():
            arm_movement()
            neck_movement()
            elevator_movement()
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass