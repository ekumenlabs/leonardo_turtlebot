#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Description: This code creates a ros node in charge of watching the turtlebot Leonardo's batteries
# and autodocking it in it's charging station if the battery level is too low. This code has been influenced by the kobuki
# autodocking routine form https://github.com/yujinrobot/kobuki/blob/devel/kobuki_auto_docking/scripts/DockDriveActionClient.py
#
# author: Patricio Tula
# reviewed by: Ernesto Corbellini / Julian Mateu
#
# TODO's in general
# +(tul1) Add mechanism that allow to run this routine externally by running a command or publishing in a topic. This is going to be very helpful
#   to debug in Leonardo independently of its battery charge.
# +
#


import rospy

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class AutoDocking(object):
    def __init__(self):
        # Node
        self._ros_node = rospy.init_node("dock_drive_client", anonymous=True)
        # Action clients 
        self._docking_client = actionlib.SimpleActionClient("dock_drive_action", AutoDockingAction)
        self._navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Subscribed topics
        self._diagnostic_agg_sub = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._listen_batteries)
        # Logic attributes
        self._doing_docking = False
        # Constants
        # TODO(tul1) replace 100 for this -> rospy.get_param("~battery_threshold", 20)
        self.BATTERY_THRESHOLD = 100 
        # TODO(tul1) set POSITION_GOAL through ros params
        # TODO(tul1) find out the right position and orientation 
        self.POSITION_GOAL = {"position": [0, 0, 0], "orientation":[0,0,0,0]}

    def _done_docking(self, status, result):
        """Callback that prints the action goal status and change the state of _doing_docking attribute. 
        This function is called when the action goal is reached."""
        if status == GoalStatus.PENDING: state="PENDING"
        elif status == GoalStatus.ACTIVE: state="ACTIVE"
        elif status == GoalStatus.PREEMPTED: state="PREEMPTED"
        elif status == GoalStatus.SUCCEEDED: state="SUCCEEDED"
        elif status == GoalStatus.ABORTED: state="ABORTED"
        elif status == GoalStatus.REJECTED: state="REJECTED"
        elif status == GoalStatus.PREEMPTING: state="PREEMPTING"
        elif status == GoalStatus.RECALLING: state="RECALLING"
        elif status == GoalStatus.RECALLED: state="RECALLED"
        elif status == GoalStatus.LOST: state="LOST"
        rospy.logdebug("Result - [ActionServer: " + state + "]: " + result.text)
        self._doing_docking = False

    def _feedback_docking(self, feedback):
        """Callback that just prints the action feedback."""
        rospy.logdebug("Feedback: [DockDrive: " + feedback.state + "]: " + feedback.text)

    def _done_navigating(self, status, result):
        """Runs auto docking routine for a turtlebot."""
        rospy.logdebug("Result - [ActionServer: " + state + "]: " + result.text)
        goal = AutoDockingGoal()
        self._docking_client.send_goal(goal, done_cb=self._done_docking, feedback_cb=self._feedback_docking)
        rospy.logdebug("Autodocking Goal: Sent.")
        rospy.on_shutdown(self._docking_client.cancel_goal)

    def _go_dock(self):
        """Runs auto docking routine for a turtlebot."""
        if rospy.is_shutdown(): 
            return        
        goal = MoveBaseGoal()
        # TODO(tul1) I suspect that 'frame_id' could the problem here
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.POSITION_GOAL["position"][0]
        goal.target_pose.pose.position.y = self.POSITION_GOAL["position"][1]
        goal.target_pose.pose.position.z = self.POSITION_GOAL["position"][2]
        goal.target_pose.pose.orientation.x = self.POSITION_GOAL["orientation"][0]
        goal.target_pose.pose.orientation.y = self.POSITION_GOAL["orientation"][1]
        goal.target_pose.pose.orientation.z = self.POSITION_GOAL["orientation"][2]
        goal.target_pose.pose.orientation.w = self.POSITION_GOAL["orientation"][3]
        self._navigation_client.send_goal(goal, done_cb=self._done_navigating)
        rospy.on_shutdown(self._navigation_client.cancel_goal)
    
    def _listen_batteries(self, data):
        """Callback that checks the batteries charge and triggers the auto docking routine if the charge is low."""
        if self._doing_docking is False:
            batteries_names = ["/Power System/Laptop Battery", "/Power System/Battery"]
            batteries_values = [element.values for element in data.status if element.name in batteries_names]
            if len(batteries_values) == 2:
                laptop_battery_percentage = float(filter(lambda x: x.key == "Percentage (%)", batteries_values[0])[0].value)
                kobuki_battery_percentage = float(filter(lambda x: x.key == "Percent", batteries_values[1])[0].value)
                if kobuki_battery_percentage < self.BATTERY_THRESHOLD or laptop_battery_percentage < self.BATTERY_THRESHOLD:
                    self._go_dock()
                    self._doing_docking = True

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        autodock = AutoDocking()
        autodock.run()
    except rospy.ROSInterruptException: 
        rospy.logdebug("program interrupted before completion")
