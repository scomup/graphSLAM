#!/usr/bin/python
# coding: UTF-8

from scipy.spatial import KDTree
from scipy.spatial.distance import cdist

import numpy as np
from math import sin, cos

import time


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

    def calculate(self, iter):
        old_points = np.copy(self.pointsA)
        new_points = np.copy(self.pointsA)
        R = np.eye(self.pointsA.shape[1])
        T = np.zeros(self.pointsA.shape[1])
        for i in range(iter):
            #neighbor_idx = self.kdtree.query(old_points)[1]
            #targets = self.pointsB[neighbor_idx]
            indices = self.nearest_neighbor(old_points, self.pointsB)
            targets = self.pointsB[indices]
            r, t = calcRigidTranformation(old_points, targets)
            new_points = np.dot(r, old_points.T).T + t
            #print("--- %s seconds ---" % (time.time() - start_time))

            R = np.dot(R,r)
            T = T+t
            #print i
            cost = np.sum(np.abs(old_points - new_points))
            if cost < 0.0001:
                break

            old_points = np.copy(new_points)

        return R, T, cost

def icp_test():
    #Y, X = np.mgrid[0:100:5, 0:100:5]
    #Z = Y ** 2 + X ** 2
    #A = np.vstack([Y.reshape(-1), X.reshape(-1), Z.reshape(-1)]).T
    #A = A.astype(float)
    #R = np.array([
    #    [cos(50), -sin(50), 0],
    #    [sin(50), cos(50), 0],
    #    [0, 0, 1]
    #])

    A = np.load("scan1.npy")
    B = np.load("scan2.npy")

    T = np.array([5.0, 20.0, 10.0])
    #B = np.dot(R, A.T).T + T

    icp = ICP(A, B)
    start_time = time.time()
    R, T , cost = icp.calculate(20)
    print("--- %s seconds ---" % (time.time() - start_time))
    print 'cost:', cost
    new_points = np.dot(R, A.T).T + T
    from matplotlib import pyplot
    from mpl_toolkits.mplot3d import Axes3D

    fig = pyplot.figure()
    ax = Axes3D(fig)

    ax.set_label("x - axis")
    ax.set_label("y - axis")
    ax.set_label("z - axis")

    ax.plot(A[:,0], A[:,1], A[:,2], "o", color="#cccccc", ms=4, mew=0.5)
    ax.plot(new_points[:,0], new_points[:,1], new_points[:,2], "o", color="#0000ff", ms=4, mew=0.5)
    ax.plot(B[:,0], B[:,1], B[:,2], "o", color="#ff0000", ms=4, mew=0.5)

    pyplot.show()

if __name__ == "__main__":
    icp_test()
