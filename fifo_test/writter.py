import os
path= "/home/vinod/myProgram.fifo"
os.mkfifo(path)
fifo=open(path,'w')
string=raw_input("Enter String to be reversed:\t ")
fifo.write(string)
fifo.close()