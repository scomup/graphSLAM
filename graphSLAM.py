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
import sys
from scipy.spatial import distance
from hough import check_loop_candidate
from costmap import CostMap
import matplotlib.pyplot as plt

args = sys.argv

##########################
icp_debug_ = False
##########################
#bagfile = '/home/liu/tokyo_bag/lg_1.bag'
bagfile = '/home/liu/bag_kusatsu/lg_kusatsu_C5_1.bag'

if len(args) > 1:
    bagfile = args[1]
#bagfile = '/home/liu/tokyo_bag/rp_2.bag'
#bagfile = '/home/liu/tokyo_bag/ap/wall1.bag'

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
min_loop_dist_ = 20
max_dist_ = 0.5
weight_trans_ = 1
weight_rot_ = 100
opt_iter_ = 5
##########################
first_cost_ = 2
second_cost_ = 5
hough_weight_ = 0.1
use_cost_threshold_ = True
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
        self.gui.setgraph(pos, adj, self.loop_candidates)

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
        # Generate basic nodes and edges for our pose graph
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
        
        # Detect the loop-closing
        loop_count = 0
        self.loop_candidates = []
        for i in range(len(data)):
            if i + min_loop_dist_ >= len(data):
                continue
            scan_i, pose_i = data[i]
            if not check_loop_candidate(scan_i, hough_weight_):
                continue
            self.loop_candidates.append(i)
            for j in range(i + min_loop_dist_, len(data)):
                scan_j, pose_j = data[j]
                dst = distance.euclidean(pose_i[0:2],pose_j[0:2])
                if dst > max_dist_:
                    continue
                #if not check_loop_candidate(scan_j, hough_weight_):
                #    continue
                loop_count += 1
                delta_odom_init = getDeltaM(v2t(pose_i), v2t(pose_j))
                delta_rot_init, delta_trans_init = getRtFromM(delta_odom_init)

                #start_time = time.time()
                icp = ICP(scan_i, scan_j)
                delta_rot, delta_trans, cost = icp.calculate(200, delta_rot_init, delta_trans_init, max_dist_)
                if(cost > first_cost_):
                    #print 'old cost:',cost
                    delta_rot, delta_trans, cost = icp.calculate(200, delta_rot, delta_trans, 0.5)
                    #print 'new cost:',cost
                    #print '-------------'
                if(use_cost_threshold_ and (cost > second_cost_)):
                    continue
                #cost = cost/5

                if icp_debug_:
                    new_scan_j = transpose(delta_rot, delta_trans.ravel(),scan_j)
                    scan_j = transpose(delta_rot_init, delta_trans_init.ravel(),scan_j)
                    show_icp_matching(scan_i,new_scan_j,scan_j, str(i)+'-->'+str(j)+'cost:'+str(cost))
                    #plt.show()

                infm = np.array([[weight_trans_/cost**2, 0, 0],
                                [0,  weight_trans_/cost**2, 0],
                                [0,  0,  weight_rot_/cost**2]])
                edge = [i, j, t2v(getMFromRt(delta_rot, delta_trans)), infm]
                edges.append(edge)
                self.pg.edges.append(PoseEdge(*edge))
        print('{0} loop have been detected!'.format(loop_count))


def show_icp_matching(a,b,c, str):
    fig, ax = plt.subplots()
    plt.xlim(-4,4)
    plt.ylim(-4,4)
    ax.scatter(a[:,0],a[:,1], c='blue',s = 10, label = "previous scan")
    ax.scatter(b[:,0],b[:,1], c='red',s = 10,label = "current scan(matched)")
    ax.scatter(c[:,0],c[:,1], c='green',s = 10,label = "current scan")
    ax.set_title(str)
    ax.legend()

    
if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()
    bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    slam = graphSLAM(bagreader.data,gui)
    gui.guiobj.set_slam_core(slam)
    plt.show()
    input("Press Enter to Exit.\n")
