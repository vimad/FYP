import numpy as np

def rotmat(roll, pitch, yaw = 0):
    '''this function returns a rotation matrix for given euler angles'''
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

    return np.matrix(R_yaw)*np.matrix(R_pitch)*np.matrix(R_roll)


def convertToRadians(degrees):
    return degrees*np.pi/180

def convertToDegrees(rads):
    return rads*180/np.pi


def rotmat_inverse(rotmat):
    '''this returns inverse of a given rotation matrix'''
    return rotmat.transpose()


def getVectorInEarthFrameComp(roll, pitch, x, y, z, yaw = 0,):
    rot = rotmat(roll, pitch, yaw)
    P_Aorg = np.matrix([[x], [y], [z]])
    P_Borg = rot*P_Aorg
    return P_Borg.transpose().tolist()[0]

def getVectorInDroneFrameComp(roll, pitch, x, y, z, yaw = 0):
    rot = rotmat(roll, pitch, yaw).transpose()
    P_Borg = np.matrix([[x], [y], [z]])
    P_Aorg = rot*P_Borg
    return P_Aorg.transpose().tolist()[0]
    
def getVectorInEarthFrame(roll, pitch, vect, yaw = 0):
    return getVectorInEarthFrameComp(roll, pitch, vect[0], vect[1], vect[2], yaw)
    
def getVectorInDroneFrame(roll, pitch, vect, yaw = 0):
    return getVectorInDroneFrameComp(roll, pitch, vect[0], vect[1], vect[2], yaw)