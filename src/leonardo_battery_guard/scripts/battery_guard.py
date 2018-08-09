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

import rospy
import ast

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse


class AutoDocking(object):
    def __init__(self):
        # Node
        self._ros_node = rospy.init_node("dock_drive_client", anonymous=True)
        # Services
        self._service = rospy.Service('run_autodocking', Trigger, self._run_service)
        # Action clients 
        self._docking_client = actionlib.SimpleActionClient("dock_drive_action", AutoDockingAction)
        self._docking_client.wait_for_server()
        self._navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._navigation_client.wait_for_server()
        # Subscribed topics
        self._diagnostic_agg_sub = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self._listen_batteries)
        # Logic attributes
        self._doing_docking = False
        # Constants
        self.BATTERY_THRESHOLD = rospy.get_param("~battery_threshold", 10)
        self.POSITION_GOAL_DEFAULT = "{'position': [-1.017, -0.414, 0.0],'orientation':[0.0, 0.0, 0.95, 0.312]}"
        position_goal = rospy.get_param("~position_goal", self.POSITION_GOAL_DEFAULT)
        self.POSITION_GOAL = ast.literal_eval(position_goal)
        
    def _run_service(self, req):
        """Run docking routine"""
        self._go_dock()
        return TriggerResponse(True, "success")

    def _parse_GoalStatus(self, status):
        """Convert GoalStatus into strings"""
        state = ""
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
        return state

    def _done_docking(self, status, result):
        """Callback that prints the action goal status and change the state of _doing_docking attribute. 
        This function is called when the action goal is reached."""
        state = self._parse_GoalStatus(status)
        rospy.logdebug("Docking - Result - [ActionServer: " + state + "]: " + result.text)
        self._doing_docking = False

    def _feedback_docking(self, feedback):
        """Callback that just prints the action feedback."""
        rospy.logdebug("Docking - Feedback: [DockDrive: " + feedback.state + "]: " + feedback.text)

    def _done_navigating(self, status, result):
        """Runs auto docking routine for a turtlebot."""
        state = self._parse_GoalStatus(status)
        rospy.logdebug("Navigation - Result - [ActionServer: " + str(state) + "]: " + str(result))
        goal = AutoDockingGoal()
        self._docking_client.send_goal(goal, done_cb=self._done_docking, feedback_cb=self._feedback_docking)
        rospy.logdebug("Autodocking Goal: Sent.")
        rospy.on_shutdown(self._docking_client.cancel_goal)

    def _feedback_navigating(self, feedback):
        """Callback that just prints the action feedback."""
        rospy.logdebug("Navigation - Feedback: [Navigation: " + str(feedback.base_position) + "]")

    def _go_dock(self):
        """Runs auto docking routine for a turtlebot."""
        if rospy.is_shutdown(): 
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.POSITION_GOAL["position"][0]
        goal.target_pose.pose.position.y = self.POSITION_GOAL["position"][1]
        goal.target_pose.pose.position.z = self.POSITION_GOAL["position"][2]
        goal.target_pose.pose.orientation.x = self.POSITION_GOAL["orientation"][0]
        goal.target_pose.pose.orientation.y = self.POSITION_GOAL["orientation"][1]
        goal.target_pose.pose.orientation.z = self.POSITION_GOAL["orientation"][2]
        goal.target_pose.pose.orientation.w = self.POSITION_GOAL["orientation"][3]
        self._navigation_client.send_goal(goal, done_cb=self._done_navigating, feedback_cb=self._feedback_navigating)
        rospy.logdebug("Navigation Goal: Sent.")
        rospy.logdebug(goal)
        rospy.on_shutdown(self._navigation_client.cancel_goal)
    
    def _listen_batteries(self, data):
        """Callback that checks the batteries charge and triggers the auto docking routine if the charge is low."""
        if self._doing_docking is False:
            batteries_names = ["/Power System/Laptop Battery", "/Power System/Battery"]
            batteries_values = [element.values for element in data.status if element.name in batteries_names]
            if len(batteries_values) == 2:
                charging_state = str(filter(lambda x: x.key == "Charging State", batteries_values[1])[0].value)
                batteries_charging = (charging_state != "Not Charging")
                if batteries_charging is False:
                    laptop_battery_percentage = float(filter(lambda x: x.key == "Percentage (%)", batteries_values[0])[0].value)
                    kobuki_battery_percentage = float(filter(lambda x: x.key == "Percent", batteries_values[1])[0].value)
                    if kobuki_battery_percentage < self.BATTERY_THRESHOLD or laptop_battery_percentage < self.BATTERY_THRESHOLD:
                        self._go_dock()
                        self._doing_docking = True

    def run(self):
        """Runs the node threat"""
        rospy.spin()


if __name__ == "__main__":
    try:
        autodock = AutoDocking()
        autodock.run()
    except rospy.ROSInterruptException: 
        rospy.logdebug("program interrupted before completion")
