#!/usr/bin/env python3
#Bryan

import rospy
import actionlib
from moveit_msgs.msg import Grasp
from system_stimuli.msg import armGoal, armResult, armFeedback, armAction
import system_stimuli.msg


class ArmServer:
    #create messages that are used to publish feedback/result
    _feedback = system_stimuli.msg.armFeedback()
    _result = system_stimuli.msg.armResult()
    def __init__(self):
        self.server = actionlib.SimpleActionServer('arm_server', armAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()
        #rospy.spin()
    def execute_cb(self, goal):
        #validate goal
        if goal.batch_size > 2 or goal.batch_size < 0:
            print("Goal not accepted, limit of batch size exceeded")
            self.server.set_aborted()
            return
        #helper variables
        r = rospy.Rate(1)
        success = True
        #append data
        self._feedback.state = 0
        self._result.result = 0
        print('Executing arm grasp movement')
        response = 0
        i = 0
        #i = goal.batch_size
        while i in range(0,2):
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                print('Server preempted')
                self.server.set_preempted()
                success = False
                break

            # update feedback & result
            if i == 0:
                print('------------ Calculating FK ------------')
                self._feedback.state = 0
                #self._result.result = 0
            if i == 1:
                print('-------- Executing arm movement --------')
                self._feedback.state = 1
                #self._result.result = 1
            if i == 2:
                print('-------- Executing grip movement --------')
                self._feedback.state = 2
                #self._result.result = 2
            
            self.server.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.result = 0 #TCP Correctly positioned
            rospy.loginfo('Succeeded checking arm.')
            self.server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('arm_server')
    server = ArmServer()
    rospy.spin()