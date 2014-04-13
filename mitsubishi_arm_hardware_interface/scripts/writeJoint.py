#!/usr/bin/env python
import rospy
import serial
import time

#COM.baudrate=115200


if __name__=="__main__":
  rospy.init_node('cyber_glove_teleop')
  j =[0.0,0.0,0.0,0.0,0.0,0.0]
  #j2 =[0.2,0.2,0.0,0.2,-0.3,1.0]

  COM =serial.Serial('/dev/ttyUSB0',9600)
  COM.parity = 'E'
  COM.stopbits = 2
  COM.timeout= 0.0
  print COM
  buffer = []
  r=rospy.Rate(10) # 10 hz

  counter=0;
  init=0
  while rospy.is_shutdown()==False:
    if init==0:
        COM.write("2\r\n") # tells slave to init
    init=1
    counter = counter+1
    adder = counter % 2
    print 'reading'
    A = COM.read(1)
    #A = COM.read(1)
    print 'read'
    buffer.append(A)
    if A == '\n':
      if "M" in buffer:
        print "GOT an M: ", buffer
	if adder==1:
          S = str(j[0]) + ',' + str(j[1]+adder) + ',' + str(j[2]) + ',' + str(j[3]) + ',' + str(j[4]) + ',' + str(j[5]) + "\r\n"
        else:
          S = str(j[0]-1.0) + ',' + str(j[1]) + ',' + str(j[2]) + ',' + str(j[3]) + ',' + str(j[4]) + ',' + str(j[5]) + "\r\n"
        print "SENDING:",S
        COM.write(S)
        print 'SENT'
	init=0
        A = []
      print buffer
      buffer = []
    #r.sleep()
print "FINISH, i will close the serial port now"
COM.close()
	
