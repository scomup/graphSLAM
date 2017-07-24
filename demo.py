#!/usr/bin/python
# coding: UTF-8

import time
import numpy as np
from icp import ICP
from gui import graphSLAM_GUI_Thread
from readbag import BagReader
from common import *


class graphSLAM():
    def __init__(self, raw_data, gui):
        self.raw_data = raw_data
        self.gui = gui
        self.idx = 0
        yaw = 0.0
        x = 0.0
        y = 0.0
        self.scan_base = np.array([[np.cos(yaw), -np.sin(yaw),x], 
            [np.sin(yaw),  y],
            [0.,0.,1.]])
        self.size = len(self.raw_data)
        self.dur = time.time() - time.time()
        self.pose_matrix = np.eye(3)


    def run(self):
        while self.idx < self.size:
            if self.idx % 10 == 0:
                self.step()
            self.idx += 1
        print("icp %s seconds" % self.dur)

    def step(self):
        scan, odom = self.raw_data[self.idx]


        try: 
            last_odom = self.last_odom
            last_scan = self.last_scan
            self.last_odom = odom
            self.last_scan = scan
        except AttributeError:
            self.last_odom = odom
            self.last_scan = scan
            return
        if self.idx > 1400:
            return 
        delta_odom = getDeltaM(last_odom, odom)
        delta_rot, delta_trans = getRtFromM(delta_odom)
        #start_time = time.time()
        icp = ICP(last_scan, scan)
        delta_rot_sm, delta_trans_sm, cost = icp.calculate(20, delta_rot, delta_trans, 0.01)
        
        delta_odom_sm = getMFromRt(delta_rot_sm, delta_trans_sm)
        self.pose_matrix = np.dot(self.pose_matrix, delta_odom_sm)
        #print self.pose_matrix
        R_sm, t_sm = getRtFromM(self.pose_matrix)

        
        scan_t = transpose(R_sm, t_sm, scan)
        self.gui.setpcd(scan_t)
        self.gui.setrobot(matrix_to_pose(self.pose_matrix))
        print self.idx, cost
        raw_input("Press Enter to continue...")
        #end_time = time.time()
        #self.dur += (end_time - start_time)

        
        #print cost





if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()

    bagreader = BagReader('h1.bag', 'scan', 'odom',0,800)
    start_time = time.time()
    slam = graphSLAM(bagreader.data, gui)
    start_time = time.time()
    slam.run()
    print("--- %s seconds ---" % (time.time() - start_time))