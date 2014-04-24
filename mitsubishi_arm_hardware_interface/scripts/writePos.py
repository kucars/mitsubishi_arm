#!/usr/bin/env python

###############################################################################################
#                                                                                             #
# Mitsubishi RV-6SDL pose writing and command example                                         #
#                                                                                             #
# by Rui P. de Figueiredo and Tarek Taha, Dongming Gan, Khalifa University Robotics Institute #
#                                                                                             #                                                           
###############################################################################################

import rospy
import serial
import time

# Open serial port connection
baudrate=19200
COM =serial.Serial('/dev/ttyUSB0',baudrate)
COM.parity = 'E'
COM.stopbits = 2
COM.timeout= 0.0

#########################################
# Read end effector POSE values routine #
#########################################

buffer =[]

# Handshake: Send "1\r\n" to enter reading routine (read mode) and wait for response
COM.write("1\r\n") 
while 1: # Get response
  read_char = COM.read(1)
  buffer.append(read_char) #
    
  if read_char == '\n': # finish reading
    break

if "R" not in buffer:   # READ ("R") MODE
  exit(1) # something went wrong while reading
# End handshake

# Get end-effector pose values
pose_values_buffer = ""

while '\n' not in pose_values_buffer:
  read_char=COM.read(1)
  pose_values_buffer=pose_values_buffer+str(read_char)

print pose_values_buffer # 
# End get end-effector values

# Finalize reading routine by reading a "E" character
buffer = []
while '\n' not in buffer: # Get response
  read_char = COM.read(1)
  buffer.append(read_char) #

if "E" not in buffer:   # READ ("R") MODE
  exit(1) # something went wrong while reading
# End reading routine

####################################
# Send desired pose values routine #
####################################

buffer =[]

# Handshake: Send "2\r\n" to enter commanding routine (move mode) and wait for response
COM.write("2\r\n") 
while 1: # Get response
 
  read_char = COM.read(1)
  buffer.append(read_char) #
    
  if read_char == '\n': # finish reading
    break

print buffer
if "M" not in buffer: # READ END ("E") CHARACTER
  exit(1) # something went wrong while reading
# End handshake

command = [5.31,0.52,1243.94,0.06,3.78,0.01] # IN MILIMETERS
S = str(command[0]) + ',' + str(command[1]) + ',' + str(command[2]) + ',' + str(command[3]) + ',' + str(command[4]) + ',' + str(command[5]) + "\r\n"
COM.write(S) # Send command

# Finalize reading routine by reading a "E" character
buffer = []
while '\n' not in buffer: # Get response
  read_char = COM.read(1)
  buffer.append(read_char) 

if "E" not in buffer: # READ END ("E") CHARACTER
  exit(1) # something went wrong while reading
# End reading routine




	
