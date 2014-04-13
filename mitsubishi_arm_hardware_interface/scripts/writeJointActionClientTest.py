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

    joint_max_limit=1.0
    i=0
    resolution=2
    goal = control_msgs.msg.JointTrajectoryGoal()
    while rospy.is_shutdown()==False:
      if i==resolution:
        i=0
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        print client.get_result()  # A FibonacciResult
        #print goal
        goal.trajectory.points=[]
      # Creates a goal to send to the action server.
      trajectory_point=trajectory_msgs.msg.JointTrajectoryPoint()
      trajectory_point.positions=[0.1,i*(joint_max_limit/resolution),0.0,i*(joint_max_limit/resolution),-0.4,1.5]
      goal.trajectory.points.append(trajectory_point)
      
      i=i+1

    
    trajectory_point.positions=[0.1,i*(joint_max_limit/resolution),0.0,i*(joint_max_limit/resolution),-0.4,1.5]

    # Sends the goal to the action server.





if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mitsubishi_joint_trajectory_client')
        result = JointTrajectoryActionClient()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
