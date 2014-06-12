#! /usr/bin/env python
import roslib;
import rospy
import serial
import time
import actionlib
from decimal import Decimal
import sensor_msgs.msg


if __name__=="__main__":
  rospy.init_node('mitsubishi_joint_state_publisher')
  r=rospy.Rate(100) # 10 hz

  COM =serial.Serial('/dev/ttyUSB0',9600)
  COM.parity = 'E'
  COM.stopbits = 2
  COM.timeout = 0.5
  pub = rospy.Publisher('joint_states', sensor_msgs.msg.JointState)
  #COM.baudrate=115200
  joint_states_msg = sensor_msgs.msg.JointState()
  joint_states_msg.name=['j1','j2','j3','j4','j5','j6']
  buffer=""
  while rospy.is_shutdown()==False:
    try:
      COM.write("1\r\n")
      A = COM.read(1)
      #print A
      buffer=buffer + str(A)
      if '\n' in buffer:
        joint_states_msg.position=[]
        buffer = str(buffer[:-2])
        buffer = buffer.replace(chr(0),' ')
        #print buffer      
        nbuffer = buffer.split(' ')
        for i in nbuffer:
         if i != '':
          try:
           joint_states_msg.position.append(float(Decimal(i)))
          except:
           buffer=""
           continue
        joint_states_msg.header.stamp=rospy.Time.now()
        if len(joint_states_msg.position)!=6:
          continue
        pub.publish(joint_states_msg)
        #print joint_states_msg
        buffer=""
      r.sleep()
    except:
      buffer=""
      pass
  COM.close()
