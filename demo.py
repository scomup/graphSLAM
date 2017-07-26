#!/usr/bin/python
# coding: UTF-8

import time
import numpy as np
from icp import ICP
from gui import graphSLAM_GUI_Thread
from readbag import BagReader
from common import *
from posegraph import PoseGraph, PoseEdge
from pyflann import FLANN

from scipy.spatial import distance
##########################
#bagfile = '/home/liu/tokyo_bag/lg_2.bag'
bagfile = '/home/liu/tokyo_bag/lg_2.bag'
scan_topic = '/scan' 
odom_topic = '/odom'
#bagfile = '/home/liu/bag_kusatsu/rp_kusatsu_C5_1.bag'
#bagfile = '/home/liu/tokyo_bag/rp_1.bag'
#
#bagfile = '/home/liu/tokyo_bag/ap/remote2.bag'
#scan_topic = '/Rulo/laser_scan'
#odom_topic = '/Rulo/odom'
#bagfile = '/home/liu/test.bag'
#scan_topic = '/myRobot/laser_scan'
#odom_topic = '/myRobot/odom'
start_time = 0
end_time = 800
##########################
base_x_ = 0.
base_y_ = 0.
base_yaw_ = 0.03
#base_yaw_ = 0.03
#base_yaw_ = -3.12
#base_x_ = -0.11
#base_y_ = 0.
#base_yaw_ = -1.54

##########################
d_thresh_ = .5
a_thresh_ = np.pi/6
##########################


class graphSLAM():
    def __init__(self, raw_data,gui):
        self.gui = gui
        x = base_x_
        y = base_y_
        yaw = base_yaw_
        self.scan_base = np.array([[np.cos(yaw), -np.sin(yaw),x], 
            [np.sin(yaw),  np.cos(yaw),  y],
            [0.,0.,1.]])
        self.pg = PoseGraph() # LS-SLAM pose graph
        self.data = self.readdata(raw_data)
        self.creategraph(self.data)
        self.data = []
        self.node = []
        self.edge = []
        adj = np.array([[edge.id_from, edge.id_to] for edge in self.pg.edges])
        pos = self.pg.nodes[:,0:2]
        self.gui.setgraph(pos, adj)


    def checkupdate(self,pre_pose,cur_pose):
        dx = pre_pose[0] - cur_pose[0]
        dy = pre_pose[1] - cur_pose[1]
        da = pre_pose[2] - cur_pose[2]
        trans = sqrt(dx**2 + dy**2)
        update = (trans > d_thresh_) or (fabs(da) > a_thresh_)
        return update

    def run(self):
        self.creategraph(self.data)

    def readdata(self, raw_data):
        data = []
        for scan, odom in raw_data:
            
            rot, trans = getRtFromM(self.scan_base)
            scan = transpose(rot, trans, scan)

            pose = t2v(odom).ravel()
            if len(data) == 0:
                data.append((scan, pose))

            if not self.checkupdate(data[-1][1],pose):
                continue    

            data.append((scan, pose))

            #print pose
        return data

    def creategraph(self, data):
        nodes = []
        edges = []
        for i in range(len(data)):
            scan, pose = data[i]
            nodes.append(pose)
            self.pg.nodes.append(pose)
            if i == 0:
                continue
            infm = np.array([[1, 0, 0],
                            [0,  1, ],
                            [0,  0,  1]])
            edge = [i-1, i, get_motion_vector(nodes[i], nodes[i-1]), infm]
            edges.append(edge)
            self.pg.edges.append(PoseEdge(*edge))

        self.pg.nodes = np.array(self.pg.nodes)        
        #flann = FLANN()

        for i in range(len(data)):
            if i + 20 < len(data):
                for j in range(i + 20, len(data)):
                    scan_i, pose_i = data[i]
                    scan_j, pose_j = data[j]
                    dst = distance.euclidean(pose_i[0:2],pose_j[0:2])
                    if dst < 1:
                        infm = np.array([[1, 0, 0],
                            [0,  1, ],
                            [0,  0,  1]])
                        edge = [i, j, get_motion_vector(nodes[j], nodes[i]), infm]
                        edges.append(edge)
                        self.pg.edges.append(PoseEdge(*edge))                

            
            
            
        pass

    def findloop(self, data):
        nodes = []
        edges = []
        for i in range(len(data)):
            scan, pose = data[i]
            nodes.append(pose)
            self.pg.nodes.append(pose)
            if i == 0:
                continue
            infm = np.array([[1, 0, 0],
                            [0,  1, ],
                            [0,  0,  1]])
            edge = [i-1, i, get_motion_vector(nodes[i], nodes[i-1]), infm]
            edges.append(edge)
            self.pg.edges.append(PoseEdge(*edge))
        self.pg.nodes = np.array(self.pg.nodes)
        pass

        #pg.nodes = np.array(pg.nodes)


        


if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()

    bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    slam = graphSLAM(bagreader.data,gui)
    time.sleep(1000)
    start_time = time.time()
