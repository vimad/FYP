import math

def rotmat(alpha_deg, beta_deg, gamma_deg):
    '''this function returns a rotation matrix for given euler angles'''

    alpha = convertToRadians(alpha_deg)
    beta = convertToRadians(beta_deg)
    gamma = convertToRadians(gamma_deg)

    R_gamma = [
            [math.cos(gamma), -math.sin(gamma), 0],
            [math.sin(gamma), math.cos(gamma), 0],
            [0, 0, 1]
            ]

    R_beta = [
            [math.cos(beta), 0, math.sin(beta)],
            [0, 1, 0],
            [-math.sin(beta), 0, math.cos(beta)]
            ]

    R_alpha = [
            [1, 0, 0],
            [0, math.cos(alpha), -math.sin(alpha)],
            [0, math.sin(alpha), math.cos(alpha)]
            ]

    R1 = mat_mul(R_alpha, R_beta)
    R = mat_mul(R, R_gamma)


def convertToRadians(degrees):
    return degrees*math.pi/180

def convertToDegrees(rads):
    return rads*180/math.pi


def rotmat_inverse(rotmat):
    '''this returns inverse of a given rotation matrix'''
    return mat_trans(rotmat)




def mul_mat(mat1, mat2):

    if (len(mat1[0]) != len(mat2)):
        return [[1, 0, 0],[0, 1, 0],[0, 0, 1]]

    result = []
    for i in range(len(mat1)):
        row = []
        for j in range(len(mat2[0])):
            res = 0
            for k in range(len(mat1[0])):
                res += mat1[i][k]*mat2[k][j]
            row.append(res)
        result.append(row)

    return result


def mat_trans(mat):
    return [[mat[j][i] for j in range(len(mat))] for i in range(len(mat[0]))]


def getCordinatesInEarthFrame(alpha, beta, gamma, x, y, z):
    rotmat = rotmat(alpha, beta, gamma)
    P_Aorg = [[-x], [-y], [-z]]
    P_Borg = mat_mul(rotmat, P_Aorg)
    return P_Borg
