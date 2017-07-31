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


##########################
#bagfile = '/home/liu/tokyo_bag/lg_2.bag'
bagfile = '/home/liu/tokyo_bag/lg_1.bag'
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


pg = PoseGraph()

data = np.array([[0.,0,0],\
    [1,0,-0.57],\
    [1,-1,-1.14],\
    [0,-1,1.57],\
    [0,-0.2,0],\
    ])
loopedges = np.array([[0,4]])

def creategraph(data):
    nodes = []
    edges = []
    for i in range(len(data)):
        pose = data[i]
        nodes.append(pose)
        pg.nodes.append(pose)
        if i == 0:
            continue
        infm = np.array([[20., 0, 0],
                        [0,  20, 0],
                        [0,  0,  1000]])
        edge = [i-1, i, get_motion_vector(nodes[i],nodes[i-1]), infm]
        edges.append(edge)
        pg.edges.append(PoseEdge(*edge))
    pg.nodes = np.array(pg.nodes)
    for idpair in loopedges:
        infm = np.array([[20., 0, 0],
            [0,  20, 0],
            [0,  0,  1000]])
        edge = [idpair[0], idpair[1], np.array([0.,0, 0]), infm]
        edges.append(edge)
        pg.edges.append(PoseEdge(*edge))

            
def show_graph(gui):
    adj = np.array([[edge.id_from, edge.id_to] for edge in pg.edges])
    pos = pg.nodes[:,0:2]
    gui.setgraph(pos, adj)


if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()
    creategraph(data)
    show_graph(gui)
    while True:
        time.sleep(0.1)
        if gui.guiobj.state != 1:
            continue
        pg.optimize(5)
        show_graph(gui)
        gui.guiobj.state = 0
        
    #bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    #slam = graphSLAM(gui)
    #slam.run()
input("Press Enter to continue...")