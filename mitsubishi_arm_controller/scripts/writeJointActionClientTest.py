#! /usr/bin/env python

import roslib; 
import rospy

# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

def JointTrajectoryActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.

    client = actionlib.SimpleActionClient('mitsubishi_joint_trajectory_server', control_msgs.msg.JointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    first_point=trajectory_msgs.msg.JointTrajectoryPoint()
    first_point.positions=[0.1,0.7,0.0,0.2,-0.4,1.5]

    second_point=trajectory_msgs.msg.JointTrajectoryPoint()
    second_point.positions=[0.1,0.65,0.0,0.2,-0.4,1.5]

    third_point=trajectory_msgs.msg.JointTrajectoryPoint()
    third_point.positions=[0.1,0.6,0.0,0.2,-0.4,1.5]

    forth_point=trajectory_msgs.msg.JointTrajectoryPoint()
    forth_point.positions=[0.1,0.55,0.0,0.2,-0.4,1.5]

    fifth_point=trajectory_msgs.msg.JointTrajectoryPoint()
    fifth_point.positions=[0.1,0.5,0.0,0.2,-0.4,1.5]

    sixth_point=trajectory_msgs.msg.JointTrajectoryPoint()
    sixth_point.positions=[0.0,0.0,0.0,0.0,0.0,0.0]


    goal = control_msgs.msg.JointTrajectoryGoal()
    goal.trajectory.points.append(first_point)
    goal.trajectory.points.append(second_point)
    goal.trajectory.points.append(third_point)
    goal.trajectory.points.append(forth_point)
    goal.trajectory.points.append(fifth_point)
    goal.trajectory.points.append(sixth_point)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mitsubishi_joint_trajectory_client')
        result = JointTrajectoryActionClient()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
