#!/usr/bin/env python

#################################################################################
#                                                                               #
# Mitsubishi RV-6SDL joint position reading and command example                 #
#                                                                               #
# by Rui P. de Figueiredo and Tarek Taha, Khalifa University Robotics Institute #
#                                                                               #                                                           
#################################################################################

import rospy
import serial
import time

# Open serial port connection
baudrate=19200
COM =serial.Serial('/dev/ttyUSB1',baudrate)
COM.parity = 'E'
COM.stopbits = 2
COM.timeout= 0.0

###################################
# Read joint angle values routine #
###################################

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

# Get joint values
joint_values_buffer = ""

while '\n' not in joint_values_buffer:
  read_char=COM.read(1)
  joint_values_buffer=joint_values_buffer+str(read_char)

print joint_values_buffer # 
# End get joint values

# Finalize reading routine by reading a "E" character
buffer = []
while '\n' not in buffer: # Get response
  read_char = COM.read(1)
  buffer.append(read_char) #

if "E" not in buffer:   # READ ("R") MODE
  exit(1) # something went wrong while reading
# End reading routine

###########################################
# Send desired joint angle values routine #
###########################################

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

command = [0.0,0.0,0.0,0.0,0.0,0.0]
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

COM.close()


	
