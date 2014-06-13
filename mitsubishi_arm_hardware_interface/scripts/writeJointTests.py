import serial
import time
COM =serial.Serial('/dev/ttyUSB0',9600)
COM.parity = 'E'
COM.stopbits = 2
COM.timeout = 0.5
#COM.baudrate=115200
j = [0.5,0.5,0.0,0.2,-0.4,1.5]
print COM

def splitCount(s, count):
     return [''.join(x) for x in zip(*[list(s[z::count]) for z in range(count)])]

buffer= ""
while 1:
  COM.write("1\r\n")
  A = COM.read(1)
  
  buffer=buffer + str(A)
  #if A!='\n' or A!='\r':
  #  buffer=buffer + str(A)
    
  if '\n' in buffer:
    #print buffer
    #buffer = []
    buffer = str(buffer[:-2])

    #print 'before:',buffer, ' leng:', str(len(buffer))
    #buffer=splitCount(buffer,14)
    #print 'after:',buffer
    buffer_list=[]
    buffer=buffer.split('+')
    for x in buffer:
      x = x.split('-')
      buffer_list.append(x)
      print "Spit Sec:", x
    for n in buffer_list:     
      if len(n) == 1:
        if n[0] == "":
          continue
        print "N1:", (float(n[0].strip(' '))) 
      else:
        if n[0] == "" or n[1] == "":
          continue
        print "N1:", (float(n[0].strip(' ')))
        print "N2:",(-float(n[1].strip(' ')))  
    #splitstr = str(buffer.split('+'));
    #splitstr = splitstr.split('-');
    #for x in splitstr:
    # x = x.strip(" ")
    # x = x.strip('-')
     #x = float(x)
    # print "Spit Sec:",str(x)
    buffer = ""
  time.sleep(0.01)
print "FINISH, i will close the serial port now"
COM.close()
	
