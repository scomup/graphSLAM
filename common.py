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

# see g2o.pdf in https://github.com/RainerKuemmerle/g2o/tree/master/doc
def apply_motion_vector(pose, motion):
    return np.array([(pose[0]+motion[0]*np.cos(motion[2])-motion[1]*np.sin(motion[2])), \
            (pose[1]+motion[0]*np.sin(motion[2])+motion[1]*np.cos(motion[2])), \
            np.mod(pose[2]+motion[2], 2*np.pi)-np.pi])
    
def get_motion_vector(pose1, pose2):
    return np.array([((pose1[0]-pose2[0])*np.cos(pose2[2])+(pose1[1]-pose2[1])*np.sin(pose2[2])),\
            (-(pose1[0]-pose2[0])*np.sin(pose2[2])+(pose1[1]-pose2[1])*np.cos(pose2[2])),\
            np.mod(pose2[2]-pose1[2], 2*np.pi)-np.pi])
    
###


def transpose(R, t, scan):
    return np.dot(R, scan.T).T + t

if __name__ == "__main__":
    pose = [2. , 1.3 ,1.4]
    a = pose_to_matrix(pose)
    print matrix_to_pose(a)
    R, t = getRtFromM(a)
    print a
    print getMFromRt(R, t)

