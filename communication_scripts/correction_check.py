import numpy as np
import os, errno
from communication.quadcopter import Quadcopter
from util.util import *
import time

copter = Quadcopter("/dev/ttyS0",57600)



FIFO_R = '/tmp/fifo_c-p'
FIFO_W = '/tmp/fifo_p-c'


try:
    os.mkfifo(FIFO_R)
    os.mkfifo(FIFO_W)

except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise
        
wfifo =open(FIFO_W, "w")
wfifo.write("start")
wfifo.close()

def getTagInfo(tagID):
    readFifo = open(FIFO_R, 'r')
    while (True):
        raw_data = readFifo.read().strip().split(",")
        if (raw_data[0] == '2'):
            break
    else:
        readFifo.close()
        return "TIMEOUT"

    readFifo.close()

    return ",".join(raw_data[2:5])
    
while True:
  raw_data = getTagInfo(7)
  #print (copter.att_roll_deg, copter.att_pitch_deg, yaw)
  time.sleep(1)
  try:
    data = [float(i.strip().replace("\x00","")) for i in raw_data.split(",")]
    timeoutCount = 0
    foundTag = True
    seeked = False
  except ValueError as ve:
    continue
    
  roll=copter.att_roll_deg*np.pi/180
  pitch=copter.att_pitch_deg*np.pi/180
  yaw=0
  
  R_yaw = [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
            ]

  R_pitch = [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
            ]

  R_roll = [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
            ]
            
  rotation = np.matrix(R_yaw)*np.matrix(R_pitch)*np.matrix(R_roll)
  
  print rotation
  
  coord = getVectorInEarthFrameComp(roll, pitch, data[0], data[1], data[2])
  print coord
  print("data-------------------------------------------",coord[0],coord[1],coord[2])