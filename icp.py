#!/usr/bin/python
# coding: UTF-8

from scipy.spatial import KDTree
from scipy.spatial.distance import cdist
from pyflann import *

import numpy as np
from math import sin, cos

import time

from readbag import BagReader
from common import *

def calcRigidTranformation(MatA, MatB):
    A, B = np.copy(MatA), np.copy(MatB)

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    A -= centroid_A
    B -= centroid_B

    H = np.dot(A.T, B)
    U, S, V = np.linalg.svd(H)
    R = np.dot(V.T, U.T)
    T = np.dot(-R, centroid_A) + centroid_B

    return R, T

class ICP(object):
    def __init__(self, pointsA, pointsB):
        self.pointsA = pointsA
        self.pointsB = pointsB
        

    def nearest_neighbor(self, src, dst):
        all_dists = cdist(src, dst, 'euclidean')
        indices = all_dists.argmin(axis=1)
        #distances = all_dists[np.arange(all_dists.shape[0]), indices]
        return  indices

    def calculate(self, iter, init_R = None, init_t = None, tolerance=0.01):
        if init_R is None and init_t is None:
            old_points = np.copy(self.pointsB)
            new_points = np.copy(self.pointsB)
            R = np.eye(self.pointsA.shape[1])
            T = np.zeros(self.pointsA.shape[1])
        else:
            R = init_R
            T = init_t
            old_points = np.dot(R, self.pointsB.T).T + T
            new_points = np.copy(self.pointsB)
        for i in range(iter):
            #neighbor_idx = self.kdtree.query(old_points)[1]
            #targets = self.pointsB[neighbor_idx]
            #indices = self.nearest_neighbor(old_points, self.pointsA)
            #indices, dists = flann.nn(self.pointsA, old_points, 1, algorithm="kmeans", branching=32, iterations=5, checks=16)
            targets = self.pointsA[indices]
            r, t = calcRigidTranformation(old_points, targets)
            new_points = np.dot(r, old_points.T).T + t
            #print("--- %s seconds ---" % (time.time() - start_time))

            R = np.dot(R,r)
            T = T+t
            #print i
            cost = np.sum(np.abs(old_points - new_points))
            #print i
            if cost < tolerance:
                break

            old_points = np.copy(new_points)

        return R, T, cost


import matplotlib.pyplot as plt

if __name__ == "__main__":
    # generate data
    x1 = np.random.rand(100)*0.5
    y1 = np.random.rand(100)
    x2 = np.random.rand(100)*0.5 + 0.5
    y2 = np.random.rand(100)
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)

    t1 = 700
    t2 = 800
    #bagreader = BagReader('h1.bag', 'scan', 'odom',0,800)
    #np.save("scan1",bagreader.data[t1][0])
    #np.save("scan2",bagreader.data[t2][0])
    #np.save("odom1",bagreader.data[t1][1])
    #np.save("odom2",bagreader.data[t2][1])
    scan1 = np.load("scan1.npy")
    scan2 = np.load("scan2.npy")
    odom1 = np.load("odom1.npy")
    odom2 = np.load("odom2.npy")
    M = getDeltaM(odom1,odom2)
    dR,dt = getRtFromM(M)


    #scan1 = scan1[:,0:2]
    #scan2 = scan2[:,0:2]
    #gui = graphSLAM_GUI_Thread()
    icp = ICP(scan1, scan2)
    start_time = time.time()
    #R, t , cost = icp.calculate(200)
    #R, t , cost = icp.calculate(200,dR, dt )
    R, t , cost = icp.calculate(200)
    print("--- %s seconds ---" % (time.time() - start_time))
    print 'cost:', cost
    new_points1 = np.dot(dR, scan2.T).T + dt
    new_points2 = np.dot(R, scan2.T).T + t

    print 'scan1:',scan1[0,:]
    ax.scatter(scan1[:,0],scan1[:,1], c='red',s = 10)
    ax.scatter(scan2[:,0],scan2[:,1], c='blue',s = 10)
    ax.scatter(new_points2[:,0],new_points2[:,1], c='yellow',s = 10)
    print 'new_points2:',new_points2[0,:]
    #ax.scatter(cost[:,0],cost[:,1], c='yellow',s = 10)
    ax.scatter(new_points1[:,0],new_points1[:,1], c='green',s = 10)

    plt.show()
    #gui.start()

