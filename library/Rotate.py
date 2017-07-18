from math import cos, sin

import numpy

import library.Eulerangles


def rotate_x(alpha, vector):
    alpha = numpy.deg2rad(alpha)
    matrix = numpy.array([[1, 0, 0],
                          [0, cos(alpha), -sin(alpha)],
                          [0, sin(alpha), cos(alpha)]])
    vector = numpy.asmatrix(vector)
    vector = numpy.transpose(vector)
    new = matrix * vector
    new = numpy.transpose(new)
    return new, matrix


def rotate_y(alpha, vector):
    alpha = numpy.deg2rad(alpha)
    matrix = numpy.array([[cos(alpha), 0, sin(alpha)],
                          [0, 1, 0],
                          [-sin(alpha), 0, cos(alpha)]])
    vector = numpy.asmatrix(vector)
    vector = numpy.transpose(vector)
    new = matrix * vector
    new = numpy.transpose(new)
    return new, matrix


def rotate_z(alpha, vector):
    alpha = numpy.deg2rad(alpha)
    matrix = numpy.array([[cos(alpha), -sin(alpha), 0],
                          [sin(alpha), cos(alpha), 0],
                          [0, 0, 1]])
    vector = numpy.asmatrix(vector)
    vector = numpy.transpose(vector)
    new = matrix * vector
    new = numpy.transpose(new)
    return new, matrix


def rotation_matrix_xyz(rx, ry, rz):
    rx = numpy.deg2rad(rx)
    ry = numpy.deg2rad(ry)
    rz = numpy.deg2rad(rz)
    matrix = Eulerangles.euler2mat(rz, ry, rx)
    return matrix


def matrix2euler(matrix):
    euler = Eulerangles.mat2euler(matrix)
    euler = Eulerangles.euler2angle_axis(euler[0], euler[1], euler[2])
    alpha = numpy.asmatrix(euler[0])
    alpha = alpha * 180/numpy.pi
    axis = euler[1]
    axis = axis/numpy.max(axis)
    euler = (alpha[0, 0], axis)
    return euler


def rotate_xyz(rx, ry, rz, vector):
    vector = numpy.asmatrix(vector)
    matrix = rotation_matrix_xyz(rx, ry, rz)
    new = matrix * numpy.transpose(vector)
    new = numpy.transpose(new)
    return new, matrix

# rx = 90
# ry = 0
# rz = 0
# V = np.asmatrix([0,0,1])
# RotateXYZ(rx,ry,rz,V)
# M = RotationMatrixXYZ(rx,ry,rz)
# print Matrix2Euler(M)

##TEST FOR EQUIVALENCE
##V = np.asmatrix([1,0,1])
##
##rx = np.deg2rad(50)
##ry = np.deg2rad(50)
##rz = np.deg2rad(50)
##MAT = Eulerangles.euler2mat(rx,ry,rz)
##
##V1 = MAT * np.transpose(V)
##V1 = np.transpose(V1)
##
##V2,M = RotateZ(50,V)
##V2,M = RotateY(50,V2)
##V2,M = RotateX(50,V2)
##
##print V
##print V1
##print V2




    
