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

from hough import check_loop_candidate
from costmap import CostMap


##########################
#bagfile = '/home/liu/tokyo_bag/lg_1.bag'
bagfile = '/home/liu/bag_kusatsu/lg_kusatsu_C7_1.bag'

#bagfile = '/home/liu/tokyo_bag/rp_1.bag'
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
d_thresh_ = 0.5
a_thresh_ = np.pi/3
##########################
min_loop_dist_ = 30
max_dist_ = 0.3
weight_trans_ = 1
weight_rot_ = 100
opt_iter_ = 5
##########################

class graphSLAM():
    def __init__(self, raw_data,gui):
        self.gui = gui
        self.costmap = CostMap()
        x = base_x_
        y = base_y_
        yaw = base_yaw_
        self.scan_base = np.array([[np.cos(base_yaw_), -np.sin(base_yaw_),base_x_], 
            [np.sin(base_yaw_),  np.cos(base_yaw_),  base_y_],
            [0.,0.,1.]])
        self.pg = PoseGraph() # LS-SLAM pose graph
        self.node_scan = self.readdata(raw_data)
        print("Generating graph...")
        self.creategraph(self.node_scan)
        print("Graph OK.")
        # Show graph before optimization
        self.show_graph()
        self.show_pointcloud()
        #print("waiting create map ...")
        #self.show_map()

        #self.gui.setpcd(pos, adj)

    def show_graph(self):
        adj = np.array([[edge.id_from, edge.id_to] for edge in self.pg.edges])
        pos = self.pg.nodes[:,0:2]
        self.gui.setgraph(pos, adj)

    def show_pointcloud(self):
        pcd = np.array([[0,0]])
        for scan, pose in self.node_scan:
            scan_trans = transpose_by_pose(pose,scan)
            pcd = np.vstack((pcd,scan_trans))
        self.gui.setpcd(pcd)

    def show_map(self, use_gaussian = False):
        
        self.costmap.map_data.fill(0)
        pcd = np.array([[0,0]])
        for scan, pose in self.node_scan:
            scan_trans = transpose_by_pose(pose,scan)
            pcd = np.vstack((pcd,scan_trans))
        idx = self.costmap.world_map(pcd)
        
        for i in range(idx.shape[0]):
            p = idx[i,:].astype(int)
            if use_gaussian:
                self.costmap.updateCostMap(p, 30, True)
            else:
                self.costmap.updateCostMap(p, 10, False)
        self.gui.setmap(self.costmap.map_data)
        
                
        

    def update(self):
        pcd = np.array([[0,0]])
        #self.node_scan.clear()
        node_scan = []
        #del self.node_scan[:]
        for i in range(len(self.node_scan)):
            scan, pose = self.node_scan[i]
            old_pose = self.pg.nodes[i,:]
            node_scan.append((scan, self.pg.nodes[i,:]))
        self.node_scan = node_scan

    def checkupdate(self,pre_pose,cur_pose):
        dx = pre_pose[0] - cur_pose[0]
        dy = pre_pose[1] - cur_pose[1]
        da = angle_diff(pre_pose[2],cur_pose[2])
        trans = sqrt(dx**2 + dy**2)
        update = (trans > d_thresh_) or (fabs(da) > a_thresh_)
        return update

    def optimize(self):
        print('Waiting optimization...')
        self.pg.optimize(opt_iter_)
        print('Optimization OK.')
        self.update()
        self.show_graph()
        self.show_pointcloud()
        


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
        # Generate basic nodes and edges for graph
        for i in range(len(data)):
            scan, pose = data[i]
            nodes.append(pose)
            self.pg.nodes.append(pose)
            if i == 0:
                continue
            infm = np.array([[weight_trans_, 0, 0],
                            [0,  weight_trans_, 0],
                            [0,  0,  weight_rot_]])
            edge = [i-1, i, get_motion_vector(nodes[i], nodes[i-1]), infm]
            edges.append(edge)
            self.pg.edges.append(PoseEdge(*edge))
        self.pg.nodes = np.array(self.pg.nodes)        
        
        # Detecte the loop-closing
        for i in range(len(data)):
            if i + min_loop_dist_ >= len(data):
                continue
            scan_i, pose_i = data[i]
            if not check_loop_candidate(scan_i):
                continue
            for j in range(i + min_loop_dist_, len(data)):
                scan_j, pose_j = data[j]
                dst = distance.euclidean(pose_i[0:2],pose_j[0:2])
                if dst > max_dist_:
                    continue

                delta_odom_init = getDeltaM(v2t(pose_i), v2t(pose_j))
                delta_rot_init, delta_trans_init = getRtFromM(delta_odom_init)

                #start_time = time.time()
                icp = ICP(scan_i, scan_j)
                delta_rot, delta_trans, cost = icp.calculate(20, delta_rot_init, delta_trans_init, max_dist_)
                t2v(getMFromRt(delta_rot, delta_trans))

                infm = np.array([[weight_trans_, 0, 0],
                                [0,  weight_trans_, 0],
                                [0,  0,  weight_rot_]])
                edge = [i, j, t2v(getMFromRt(delta_rot, delta_trans)), infm]
                edges.append(edge)
                self.pg.edges.append(PoseEdge(*edge))

if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()
    bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    slam = graphSLAM(bagreader.data,gui)
    gui.guiobj.set_slam_core(slam)
    time.sleep(1000)
    start_time = time.time()
