import os
import sys
import errno
import re

FIFO_R = "/tmp/fifo_c-p"
FIFO_W = "/tmp/fifo_p-c"

#os.mkfifo(FIFO_W)
try:
	os.mkfifo(FIFO_W)
	os.mkfifo(FIFO_R)
except OSError as oe:
	if oe.errno != errno.EEXIST:
		raise

def sendMessage(message):
	fifo2=open(FIFO_W,'w')
	fifo2.write(string)
	fifo2.close()

def ReceiveMessage():
	fifo=open(FIFO_R,'r')
	str=fifo.read()
	fifo.close();
	return str;

#str = ReceiveMessage() 
#id = str.split(',')[0]
#print(str)

while(True):
	#string=raw_input("Enter String to be reversed: ")
	string = "start"
	#sendMessage(string)
	if(string == "start"):
		while(1):
			str = ReceiveMessage() 
			id = str.split(',')[0]
			if(int(id) == 15):
				print("no tag detected");
			else:
				print(str)







