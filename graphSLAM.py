#!/usr/bin/python
# coding: UTF-8

import time
import numpy as np
from icp import ICP
from gui import graphSLAM_GUI_Thread
from readbag import BagReader
from common import *

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
d_thresh_ = .1
a_thresh_ = np.pi/6
##########################


class graphSLAM():
    def __init__(self, raw_data, gui):
        self.raw_data = raw_data
        self.gui = gui
        self.idx = 0
        x = base_x_
        y = base_y_
        yaw = base_yaw_
        self.scan_base = np.array([[np.cos(yaw), -np.sin(yaw),x], 
            [np.sin(yaw),  np.cos(yaw),  y],
            [0.,0.,1.]])
        self.size = len(self.raw_data)
        self.dur = time.time() - time.time()
        self.pose_matrix = np.eye(3)
        self.graph_base = []

    def checkupdate(self,pre_pose,cur_pose):
        dx = pre_pose[0] - cur_pose[0]
        dy = pre_pose[1] - cur_pose[1]
        da = pre_pose[2] - cur_pose[2]
        trans = sqrt(dx**2 + dy**2)
        update = (trans > d_thresh_) or (fabs(da) > a_thresh_)
        return update

    def run(self):
        while self.idx < self.size:
            time.sleep(0.1)
            if self.gui.guiobj.state == 1:
                update = False
                while not update:
                    update = self.step()
                    self.idx += 1
            elif self.gui.guiobj.state == 2:
                self.gui.guiobj.state = 0
                update = False
                while not update:
                    update = self.step()
                    self.idx += 1
        print("icp %s seconds" % self.dur)

    def step(self):
        scan, odom = self.raw_data[self.idx]
        rot, trans = getRtFromM(self.scan_base)
        scan = transpose(rot, trans, scan)
        try:
            if not self.checkupdate(matrix_to_pose(self.last_odom),matrix_to_pose(odom)):
                return False    
            last_odom = self.last_odom
            last_scan = self.last_scan
            self.last_odom = odom
            self.last_scan = scan
        except AttributeError:
            self.last_odom = odom
            self.last_scan = scan
            return False

        delta_odom_init = getDeltaM(last_odom, odom)
        delta_rot_init, delta_trans_init = getRtFromM(delta_odom_init)

        #start_time = time.time()
        icp = ICP(last_scan, scan)
        #delta_rot, delta_trans, cost = icp.calculate(20, delta_rot_init, delta_trans_init, 0.01)
        delta_rot = delta_rot_init
        delta_trans = delta_trans_init


        
        delta_odom = getMFromRt(delta_rot, delta_trans)
        

        print 'init yaw:',matrix_to_pose(delta_odom_init)[2]
        print 'icp yaw:',matrix_to_pose(delta_odom)[2]
        self.pose_matrix = np.dot(self.pose_matrix, delta_odom)

        self.graph_base.append((scan, delta_odom, self.pose_matrix))
        #print self.pose_matrix
        rot, trans = getRtFromM(self.pose_matrix)
        scan_t = transpose(rot, trans, scan)
        self.gui.setpcd(scan_t)
        self.gui.setrobot(matrix_to_pose(self.pose_matrix))
        return True
       # print self.idx, cost


if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()

    bagreader = BagReader(bagfile, scan_topic, odom_topic, start_time, end_time)
    start_time = time.time()
    slam = graphSLAM(bagreader.data, gui)
    start_time = time.time()
    slam.run()
    print("--- %s seconds ---" % (time.time() - start_time))