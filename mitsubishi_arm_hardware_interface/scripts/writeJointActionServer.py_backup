#! /usr/bin/env python

import roslib;
import rospy
import serial
import time
import actionlib
from decimal import Decimal
import sensor_msgs.msg
import math
import control_msgs.msg

is_moving=False

class JointTrajectoryActionServer(object):
  # create messages that are used to publish feedback/result
  _feedback = control_msgs.msg.JointTrajectoryFeedback()
  _result   = control_msgs.msg.JointTrajectoryResult()
 
  def __init__(self, name):
    self.last_joint_states_msg=sensor_msgs.msg.JointState()
    self.joint_states_msg=sensor_msgs.msg.JointState()
    self.joint_states_msg.name=['j1','j2','j3','j4','j5','j6']
    self.pub = rospy.Publisher('joint_states', sensor_msgs.msg.JointState)
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.JointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self.COM =serial.Serial('/dev/ttyUSB0',9600)
    self.COM.parity = 'E'
    self.COM.stopbits = 2
    self.COM.timeout= 2.0
    #self.COM.write("2\r\n") # tells slave to init
    print self.COM
  def execute_cb(self, goal):
    global is_moving
    is_moving=True
    # helper variables
    #r = rospy.Rate(1)
    success = True
    
  
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, creating JointTrajectory with %i points' % (self._action_name, len(goal.trajectory.points)))
    
    # start executing the action
    for i in range(len(goal.trajectory.points)):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
      self.sendJointState(goal.trajectory.points[i])
      rospy.loginfo(rospy.get_caller_id()+" Done")
      #self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
      # publish the feedback
      #self._as.publish_feedback(self._feedback)
      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      #r.sleep()
      
    if success:
      #self._result.status = True
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    is_moving=False

  def sendJointState(self,data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.positions)

    buffer=""
    i=0
    teste=self.COM.write("1\r\n")

    while 1:
      A = self.COM.read(1);
      buffer=buffer + str(A);

      if 'A\r\n' in buffer:
        #print "GOT an A: ", A
        #print "data pos 0:", data.position
        S = str(data.positions[0]) + ',' + str(data.positions[1]) + ',' + str(data.positions[2]) + ',' + str(data.positions[3]) + ',' + str(data.positions[4]) + ',' + str(data.positions[5]) + "\r\n"
        #print "SENDING:",S
        self.COM.write(S)
        joint_values_buffer = ""
        # Get joint state
        time.sleep(1)
        while '\n' not in joint_values_buffer:
          A=self.COM.read(1)
          joint_values_buffer=joint_values_buffer+str(A)
          #print 'inloop',A
        #print 'just before:',joint_values_buffer
        self.write_joint_state_msg(joint_values_buffer)
        #print 'JOINTS:',joint_values_buffer
        return
    return

  def publish_last_state(self):
        self.pub.publish(self.last_joint_states_msg)

  def write_joint_state_msg(self,joint_values_buffer):

        self.joint_states_msg.position=[]
        #print 'buffer before:',joint_values_buffer
        joint_values_buffer = str(joint_values_buffer[:-2])
        joint_values_buffer = joint_values_buffer.replace(chr(0),' ')
        joint_values_buffer = joint_values_buffer.replace('(',' ')
        joint_values_buffer = joint_values_buffer.replace(')',' ')

        #print buffer      
        nbuffer = joint_values_buffer.split(' ')
        nbuffer = joint_values_buffer.split(',')
        #print 'buffer after:',nbuffer
        for i in nbuffer:
         #print 'i:',i
         if i != '':
          try:
           self.joint_states_msg.position.append(math.radians(float(Decimal(i))))
          except:
           return
        self.joint_states_msg.header.stamp=rospy.Time.now()
        if len(self.joint_states_msg.position)!=6:
          return
        self.last_joint_states_msg=self.joint_states_msg
        self.pub.publish(self.joint_states_msg)
      
if __name__ == '__main__':
  rospy.init_node('mitsubishi_joint_trajectory_server')
  joint_trajectory_server=JointTrajectoryActionServer(rospy.get_name())

  r=rospy.Rate(10) # 10 hz
  while rospy.is_shutdown()==False:
    #print 'ola'
    global is_moving
    if not is_moving:
      joint_trajectory_server.publish_last_state()
    r.sleep()


