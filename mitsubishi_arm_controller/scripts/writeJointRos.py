#!/usr/bin/env python
import roslib;
import rospy
import serial
import time

#COM.baudrate=115200


#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

global COM

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.position)
    global COM
    buffer=[]
    j =[0.7,0.5,0.0,0.2,-0.4,1.5]
    j[0]=data.position[0]
    while 1:
      A = COM.read(1)
      print A
      buffer.append(A)
      if A == '\n':
        if "A" in buffer:
          #print "GOT an A: ", buffer
          #print "data pos 0:", data.position
          S = str(data.position[0]) + ',' + str(data.position[1]) + ',' + str(j[2]) + ',' + str(j[3]) + ',' + str(j[4]) + ',' + str(j[5]) + "\r\n"
          print "SENDING:",S
          COM.write(S)
          return
        print buffer
        buffer = []
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    global COM
    rospy.Subscriber("joint_angles", JointState, callback)
    COM =serial.Serial('/dev/ttyUSB0',9600)
    COM.parity = 'E'
    COM.stopbits = 2
    print COM
    buffer = []
    #r=rospy.Rate(10) # 10 hz
    COM.write("1\r\n") # tells slave to init
    r=rospy.Rate(10) # 10 hz
    while rospy.is_shutdown()==False:
      r.sleep()
      # spin() simply keeps python from exiting until this node is stopped
      #rospy.spinOnce()
        
if __name__ == '__main__':
    listener()


if __name__=="__main__":
  rospy.init_node('cyber_glove_teleop')
  init_pos=[0.5,0.5,0.0,0.2,-0.4,1.5]



