import os
import sys

FIFO_R = "/tmp/fifo_c-p"
FIFO_W = "/tmp/fifo_p-c"

os.mkfifo(FIFO_W)

fifo=open(FIFO_R,'r')

str=fifo.read()

fifo.close();

print str

fifo2=open(FIFO_W,'w')
string=raw_input("Enter String to be reversed:\t ")
fifo2.write(string)
fifo2.close()





