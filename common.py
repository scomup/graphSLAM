#!/usr/bin/python
# coding: UTF-8

import numpy as np
import tf
from math import *

def getDeltaRt(R_pre, t_pre, R_cur, t_cur):
    pre = np.c_[R_pre, t_pre.T]
    pre = np.r_[pre, np.array([[0.,0.,1.]])]
    cur = np.c_[R_cur, t_cur.T]
    cur = np.r_[cur, np.array([[0.,0.,1.]])]
    pre_inv = np.linalg.inv(pre)
    delta = np.dot(pre_inv, cur)
    delta[0:2, 0:2]
    return delta[0:2, 0:2], delta[0:2, 2]

def getDeltaM(pre, cur):
    pre_inv = np.linalg.inv(pre)
    delta = np.dot(pre_inv, cur)
    return delta

def getRtFromM(M):
    return M[0:2, 0:2], M[0:2, 2]

def getMFromRt(R,t):
    M = np.eye(3)
    M[0:2,0:2] = R.copy()
    M[0:2,2] = t.copy()
    return M

def pose_to_matrix(pose):
    return v2t(pose)
    
def matrix_to_pose(m):
    return t2v(m)

def t2v(A):
    # T2V homogeneous transformation to vector
    v = np.zeros((3,1), dtype=np.float64)
    v[:2, 0] = A[:2,2]
    v[2] = np.arctan2(A[1,0], A[0,0])
    return v

def v2t(v):
    # V2T vector to homogeneous transformation
    c = np.cos(v[2])
    s = np.sin(v[2])
    A = np.array([[c, -s, v[0]],
         [s,  c, v[1]],
         [0,  0,  1]], dtype=np.float64)
    return A


def fix_angle(angle):
    return atan2(sin(angle),cos(angle))

def angle_diff(a, b):
    a = fix_angle(a)
    b = fix_angle(b)
    d1 = a-b
    d2 = 2*np.pi - fabs(d1)
    if(d1 > 0):
        d2 = -d2
    if(fabs(d1) < fabs(d2)):
        return d1
    else:
        return d2
    
def get_motion_vector(pose1, pose2):
    return t2v(getDeltaM(v2t(pose2),v2t(pose1)))


def transpose(R, t, scan):
    return np.dot(R, scan.T).T + t

def transpose_by_pose(pose, scan):
    m = v2t(pose)
    scan_tmp = np.hstack((scan,np.ones((scan.shape[0],1))))
    scan_tmp = np.dot(m,scan_tmp.T).T
    return scan_tmp[:,0:2]

if __name__ == "__main__":
    pose = [2. , 1.3 ,1.4]
    a = pose_to_matrix(pose)
    print matrix_to_pose(a)
    R, t = getRtFromM(a)
    print a
    print getMFromRt(R, t)

