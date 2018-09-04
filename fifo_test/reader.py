import os
import sys

path= "/home/vinod/myProgram.fifo"

fifo=open(path,'r')
str=fifo.read()
revstr=str[::-1]
print "The Reversed String is",revstr
fifo.close()