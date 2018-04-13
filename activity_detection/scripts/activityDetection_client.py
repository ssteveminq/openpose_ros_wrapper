#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from activity_detection.msg import activityDetectionAction, activityDetectionGoal
from openpose_ros_wrapper_msgs.msg import Persons
from openpose_ros_wrapper_msgs.msg import PersonDetection
from openpose_ros_wrapper_msgs.msg import BodyPartDetection

def activity_detection_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('activity_detection', activityDetectionAction)
    
    print('client waiting for server')
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    print('client sending goal')
    # Creates a goal to send to the action server.
    goal = activityDetectionGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    print('client waiting for result')
    # Waits for the server to finish performing the action.
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    print('Returning')
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    print('here')
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('activity_detection_client_py')
        print('trying..')
        result = activity_detection_client()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
        print('printing result..')
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
