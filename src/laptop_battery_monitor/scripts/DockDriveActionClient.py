#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray

def doneCb(status, result):
    if 0:   
        print ''
    elif status == GoalStatus.PENDING   : state='PENDING'
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

def activeCb():
    if 0:
        print 'Action server went active.'

def feedbackCb(feedback):
    # Print state of dock_drive module (or node.)
    print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

def dock_drive_client():
    # add timeout setting
    client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
    while not client.wait_for_server(rospy.Duration(5.0)):
        if rospy.is_shutdown(): 
            return
        print 'Action server is not connected yet. still waiting...'

        goal = AutoDockingGoal()
        client.send_goal(goal, doneCb, activeCb, feedbackCb)
        print 'Goal: Sent.'
        rospy.on_shutdown(client.cancel_goal)
        client.wait_for_result()

        print '    - status:', client.get_goal_status_text()
    return client.get_result()


def callback(data):
    batteries_names = ['/Power System/Laptop Battery', "/Power System/Battery"]
    batteries_values = [element.values for element in data.status if element.name in batteries_names]
    laptop_battery_percentage = filter(lambda x: x.key == "Percentage (%)", batteries_values[0])[0].value
    kuboki_battery_percentage = filter(lambda x: x.key == "Percent", batteries_values[1])[0].value
    if kuboki_battery_percentage < 20 or laptop_battery_percentage < 20:
        print 'go dock yourself'
        dock_drive_client()

if __name__ == '__main__':
    try:
        rospy.init_node('dock_drive_client_py', anonymous=True)
        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, callback)
        rospy.spin()
    except rospy.ROSInterruptException: 
        print "program interrupted before completion"