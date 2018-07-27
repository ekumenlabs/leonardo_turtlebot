#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray


class AutoDocking:
    BATTERY_THRESHOLD = 4

    def __init__(self):
        self._ros_node = rospy.init_node('dock_drive_client_py', anonymous=True)
        self._client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
        self._diagnostic_agg_sub = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._listen_batteries)
        self._go_docking = False

    def _doneCb(self, status, result):
        if status == GoalStatus.PENDING   : state='PENDING'
        elif status == GoalStatus.ACTIVE    : state='ACTIVE'
        elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
        elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
        elif status == GoalStatus.ABORTED   : state='ABORTED'
        elif status == GoalStatus.REJECTED  : state='REJECTED'
        elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
        elif status == GoalStatus.RECALLING : state='RECALLING'
        elif status == GoalStatus.RECALLED  : state='RECALLED'
        elif status == GoalStatus.LOST      : state='LOST'
        # Print state of action server
        print 'Result - [ActionServer: ' + state + ']: ' + result.text

    def _feedbackCb(self, feedback):
        print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

    def _dock_drive_client(self):
        if rospy.is_shutdown(): 
            return        
        goal = AutoDockingGoal()
        self._client.send_goal(goal, done_cb=self._doneCb, feedback_cb=self._feedbackCb)
        print 'Goal: Sent.'
        rospy.on_shutdown(self._client.cancel_goal)
        self._go_docking = False

    
    def _listen_batteries(self, data):
        batteries_names = ['/Power System/Laptop Battery', "/Power System/Battery"]
        batteries_values = [element.values for element in data.status if element.name in batteries_names]
        laptop_battery_percentage = float(filter(lambda x: x.key == "Percentage (%)", batteries_values[0])[0].value)
        kuboki_battery_percentage = float(filter(lambda x: x.key == "Percent", batteries_values[1])[0].value)
        
        if self._go_docking is False:
            if kuboki_battery_percentage < self.BATTERY_THRESHOLD or laptop_battery_percentage < self.BATTERY_THRESHOLD:
                print kuboki_battery_percentage
                self._dock_drive_client()
                self._go_docking = True

    def run(self):
        rospy.spin()



if __name__ == '__main__':
    try:
        autodock = AutoDocking()
        autodock.run()
    except rospy.ROSInterruptException: 
        print "program interrupted before completion"