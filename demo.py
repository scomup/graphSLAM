#!/usr/bin/python
# coding: UTF-8

import time
import numpy as np

from icp import ICP
from gui import graphSLAM_GUI_Thread
from readbag import BagReader

#bagreader = BagReader('h1.bag', 'scan', 'odom',0,800)


import matplotlib.pyplot as plt


if __name__ == "__main__":
    # generate data
    x1 = np.random.rand(100)*0.5
    y1 = np.random.rand(100)
    x2 = np.random.rand(100)*0.5 + 0.5
    y2 = np.random.rand(100)
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)


    #bagreader = BagReader('h1.bag', 'scan', 'odom',0,20)
    scan1 = np.load("scan1.npy")
    scan2 = np.load("scan2.npy")
    #scan1 = scan1[:,0:2]
    #scan2 = scan2[:,0:2]
    #gui = graphSLAM_GUI_Thread()
    icp = ICP(scan1, scan2)
    start_time = time.time()
    R, t , cost = icp.calculate(20)
    print("--- %s seconds ---" % (time.time() - start_time))
    print 'cost:', cost
    new_points2 = np.dot(R, scan1.T).T + t


    ax.scatter(scan1[:,0],scan1[:,1], c='red',s = 10)
    ax.scatter(scan2[:,0],scan2[:,1], c='blue',s = 10)
    ax.scatter(new_points2[:,0],new_points2[:,1], c='green',s = 10)
    plt.show()
    #gui.start()
