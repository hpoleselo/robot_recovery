#! /usr/bin/env python
import time
import rospy
import actionlib
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
from rosgraph_msgs.msg import Log


def handle_message(data):
    log = Log()
    log = data
    check_rosout(log)


def check_rosout(log):
    client = actionlib.SimpleActionClient("franka_control/error_recovery", ErrorRecoveryAction)
    client.wait_for_server()
    goal = ErrorRecoveryActionGoal()
    robot_state = log.level
    goal.header.stamp.secs = 0
    goal.header.stamp.nsecs = 0
    goal.header.frame_id = ''
    goal.goal_id.stamp.secs = 0
    goal.goal_id.stamp.nsecs = 0
    goal.goal_id.id = ''
    goal.goal = {}
    print 'Pandas Level: ', robot_state
    if robot_state == 8:
        print 'Sending state recover to our kind Panda!'
        # 2s delay to wait for the robot state to be published as ERROR on the Logging and then send the Action
        time.sleep(2)
        client.send_goal(goal)

    '''
    command = raw_input("Do you want to recover one more time? y/n")
    if command=='y':
        client.send_goal(goal)
    else:
        pass
    '''

def rossetup():
    rospy.init_node('panda_state_tracker')
    sub = rospy.Subscriber('rosout', Log, handle_message)
    rospy.spin()


if __name__ == '__main__':
    rossetup()
