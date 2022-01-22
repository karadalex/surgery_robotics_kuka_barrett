#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import actionlib

# import actions
from kuka_barrett.msg import *


# All available states
HOME_POSITION = 'HOME_POSITION'
TOOL_HOME_POSITION = 'TOOL_HOME_POSITION'
TOOL_TABLE_SCANNING = 'TOOL_TABLE_SCANNING'
TOOL_PICKING = 'TOOL_PICKING'
TOOL_PLACING = 'TOOL_PLACING'
PIVOT_MOTION_PLANNING = 'PIVOT_MOTION_PLANNING'
PIVOT_MOTION_EXECUTION = 'PIVOT_MOTION_EXECUTION'
REMOVE_TOOL_FROM_PATIENT = 'REMOVE_TOOL_FROM_PATIENT'

# All available outcomes
abort = 'abort'
failed = 'failed'
success = 'success'
tool_not_found = 'tool not found'
tool_found = 'tool found'
max_retries = 'maximum retries'


class HomePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success])

    def action_client(self):
        # Creates the SimpleActionClient, passing the type of the action to the constructor.
        client = actionlib.SimpleActionClient('GoToHomePosition', GoToHomePositionAction)
        rospy.loginfo("Waiting for GoToHomePositionAction server")
        client.wait_for_server()
        goal = GoToHomePositionGoal(goal=0)
        client.send_goal(goal)
        rospy.loginfo("Waiting for GoToHomePositionAction result")
        client.wait_for_result()
        response = client.get_result()
        return response.result

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', HOME_POSITION)
        result = self.action_client()
        rospy.loginfo('GoToHomePositionAction result was %s', result)
        if result == 1:
            return success
        else:
            return failed


class ToolHomePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success, abort])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', TOOL_HOME_POSITION)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return failed
        else:
            return success


class ToolTableScanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[tool_not_found, tool_found, abort])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', TOOL_TABLE_SCANNING)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return tool_not_found
        else:
            return tool_found


class ToolPicking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success, max_retries, abort])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', TOOL_PICKING)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return failed
        else:
            return success


class ToolPlacing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success, abort])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', TOOL_PLACING)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return failed
        else:
            return success


class PivotMotionPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success, abort])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', PIVOT_MOTION_PLANNING)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return failed
        else:
            return success


class ToolRemoval(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[failed, success])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', REMOVE_TOOL_FROM_PATIENT)
        time.sleep(2)
        if self.counter < 3:
            self.counter += 1
            return failed
        else:
            return success



# Create and run State Machine
def main():
    rospy.init_node('smach_surgery_tasks')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            HOME_POSITION, HomePosition(),
            transitions={
                failed: HOME_POSITION, 
                success: TOOL_HOME_POSITION
            }
        )
        smach.StateMachine.add(
            TOOL_HOME_POSITION, ToolHomePosition(),
            transitions={
                failed: TOOL_HOME_POSITION, 
                success: TOOL_TABLE_SCANNING,
                abort: HOME_POSITION
            }
        )
        smach.StateMachine.add(
            TOOL_TABLE_SCANNING, ToolTableScanning(),
            transitions={
                tool_not_found: TOOL_TABLE_SCANNING, 
                tool_found: TOOL_PICKING,
                abort: HOME_POSITION
            }
        )
        smach.StateMachine.add(
            TOOL_PICKING, ToolPicking(),
            transitions={
                failed: TOOL_PICKING, 
                success: TOOL_PLACING,
                max_retries: TOOL_TABLE_SCANNING,
                abort: HOME_POSITION
            }
        )
        smach.StateMachine.add(
            TOOL_PLACING, ToolPlacing(),
            transitions={
                failed: TOOL_PLACING, 
                success: PIVOT_MOTION_PLANNING,
                abort: REMOVE_TOOL_FROM_PATIENT
            }
        )
        smach.StateMachine.add(
            PIVOT_MOTION_PLANNING, PivotMotionPlanning(),
            transitions={
                failed: PIVOT_MOTION_PLANNING, 
                success: PIVOT_MOTION_PLANNING,
                abort: REMOVE_TOOL_FROM_PATIENT
            }
        )
        smach.StateMachine.add(
            REMOVE_TOOL_FROM_PATIENT, ToolRemoval(),
            transitions={
                failed: REMOVE_TOOL_FROM_PATIENT, 
                success: HOME_POSITION,
            }
        )

    # Create and start the introspection server, to view the state machine with smach_viewer
    sis = smach_ros.IntrospectionServer('kuka_barrett_surgery', sm, '/SURGERY_TASKS')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    # Stop the IntrospectionServer
    sis.stop()


if __name__ == '__main__':
    main()
