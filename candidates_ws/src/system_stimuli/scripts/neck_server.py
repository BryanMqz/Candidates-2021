#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Quaternion
import system_stimuli.msg
from system_stimuli.msg import neckGoal, neckResult, neckFeedback, neckAction

class NeckServer:
    #create messages that are used to publish feedback/result
    _feedback = system_stimuli.msg.neckFeedback()
    _result = system_stimuli.msg.neckResult()
    def __init__(self):
        self.server = actionlib.SimpleActionServer('neck_server', neckAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()
        #rospy.spin()
    def execute_cb(self, goal):
        #validate goal
        if goal.batch_size > 1 or goal.batch_size < 0:
            print("Goal not accepted, limit of batch size exceeded")
            self.server.set_aborted()
            return
        #helper variables
        r = rospy.Rate(1)
        success = True
        #append data
        self._feedback.state = 0
        self._result.result = 0
        print('Executing neck movement')
        response = 0
        i = 0
        #i = goal.batch_size
        while i in range(0,1):
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
                print('-------- Executing neck movement --------')
                self._feedback.state = 1
                #self._result.result = 1

            self.server.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.result = 0 #Neck Correctly positioned
            rospy.loginfo('Succeeded checking neck posture.')
            self.server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('neck_server')
    server = NeckServer()
    rospy.spin()