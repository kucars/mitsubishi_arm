#! /usr/bin/env python

import roslib; 
import rospy

# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import time
# goal message 

'''
    trajectory_msgs/JointTrajectory trajectory
     std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
     string[] joint_names
     trajectory_msgs/JointTrajectoryPoint[] points
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration time_from_start
     control_msgs/JointTolerance[] path_tolerance
      string name
      float64 position
      float64 velocity
      float64 acceleration
     control_msgs/JointTolerance[] goal_tolerance
      string name
      float64 position
      float64 velocity
      float64 acceleration
     duration goal_time_tolerance
'''



def JointTrajectoryActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.

    client = actionlib.SimpleActionClient('/mitsubishi_arm/mitsubishi_arm_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.

    client.wait_for_server()

    joint_max_limit=1.0
    i=0
    resolution=100
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    #time_from_start=rospy.Duration(10)
    goal.trajectory.joint_names=['j1','j2','j3','j4','j5','j6']
    #To be reached 1 second after starting along the trajectory

    while rospy.is_shutdown()==False:
      if i==resolution:
        i=0
        print time.time()
        #When to start the trajectory: 1s from now
        goal.trajectory.header.stamp= rospy.Time.now() + rospy.Duration(2)
        client.send_goal(goal)
        #time_from_start=rospy.Time.now()
        print 'waiting for result'
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        print 'got result'
        # Prints out the result of executing the action
        print client.get_result()  # A FibonacciResult
        #print goal
        goal.trajectory.points=[]
        break;


      # Creates a goal to send to the action server.
      trajectory_point=trajectory_msgs.msg.JointTrajectoryPoint()
      trajectory_point.positions=[0.1,i*(joint_max_limit/resolution),0.0,i*(joint_max_limit/resolution),-0.4,1.5]

      trajectory_point.time_from_start=rospy.Duration(2)
      print trajectory_point.time_from_start.to_sec()
      goal.trajectory.points.append(trajectory_point)

      #print trajectory_point
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
