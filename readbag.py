#!/usr/bin/python
# coding: UTF-8

import rosbag
from std_msgs.msg import Int32, String
import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import tf
from matplotlib.widgets import Button
from geometry_msgs.msg import *
from math import *

class BagReader:
    def __init__(self, bagfile, scan_topic, odom_topic, start_time, end_time):
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.start_time = start_time
        self.end_time = end_time
        self.points = []
        self.odoms = []
        self.data = []
        print "Bag file reading..."
        self.bag = rosbag.Bag(bagfile, 'r')
        print "Scan data reading..."
        self.readscan()
        print "Odom data reading..."
        self.readodom()
        print "Data sync..."
        self.sync()
        print "All ready."
        self.bag.close()

    def readscan(self):
        laser_projector = LaserProjection()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.scan_topic]):
            cloud = laser_projector.projectLaser(msg)
            frame_points = np.zeros([0,2])
            for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
                    #theta = math.atan2(p[0] , p[1])
                    #laser_range = np.pi
                    #if theta > np.pi/2 + np.pi/3 + laser_range/2 or theta < np.pi/2 + np.pi/3 - laser_range/2:
                    #    continue
                    #d = math.sqrt(p[0]*p[0] + p[1]*p[1])
                    #if d > 1.5 and theta > np.pi/2:
                    #    continue
                    p2d = np.array([p[0], p[1]])
                    frame_points = np.vstack([frame_points, p2d])
            self.points.append([time_stamp,frame_points])

    def readodom(self):
        laser_projector = LaserProjection()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.odom_topic]):
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            e = tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
            yaw = e[2]
            t = np.array([[np.cos(yaw), -np.sin(yaw),msg.pose.pose.position.x], 
                [np.sin(yaw),  np.cos(yaw),msg.pose.pose.position.y],
                [0.,0.,1.]])
            self.odoms.append([time_stamp,t])
            #R = np.array([[np.cos(yaw), -np.sin(yaw)], 
            #    [np.sin(yaw),  np.cos(yaw)]])
            #t = [msg.pose.pose.position.x,msg.pose.pose.position.y]
            #self.odoms.append([time_stamp,(R,t)])


    def sync(self):
        idx = 0
        start_time =self.points[0][0] + rospy.Duration(self.start_time, 0)
        end_time =self.points[0][0] + rospy.Duration(self.end_time, 0)
        for time_stamp_scan,scan_data in self.points:
            if time_stamp_scan > end_time:
                    break
            if time_stamp_scan < start_time:
                continue
            while idx < len(self.odoms) - 1:
                if self.odoms[idx][0] > time_stamp_scan:
                    break
                idx+=1
            time_stamp_odom_a,odom_data_a = self.odoms[idx]
            time_stamp_odom_b,odom_data_b = self.odoms[idx-1]
            if fabs(time_stamp_scan.to_sec() - time_stamp_odom_a.to_sec()) > fabs(time_stamp_scan.to_sec() - time_stamp_odom_b.to_sec()):
                odom = odom_data_b
            else:
                odom = odom_data_a
            self.data.append((scan_data,odom))

            


class GUI(object):
    def __init__(self, bagreader):
        print "!You can use keyboard(left/right) or botton to play bag file."
        self.idx = 0
        self.data = bagreader.data
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        self.ax.get_xaxis().set_visible(False)
        self.ax.get_yaxis().set_visible(False)
        axprev = plt.axes([0.7, 0.02, 0.1, 0.055])
        axnext = plt.axes([0.81, 0.02, 0.1, 0.055])
        self.a = 0.63
        tmp = tf.transformations.euler_matrix(0.0, 0.0, self.a)
        tmp[0,3] = 0.15
        self.scan_base = np.matrix(tmp)
        self.init_pose = np.matrix([[0],[0],[0],[1]])

        bnext = Button(axnext, 'Next')
        bnext.on_clicked(self.next)
        bprev = Button(axprev, 'Previous')
        bprev.on_clicked(self.prev)
        self.fig.canvas.mpl_connect('key_press_event', self.press)
        
        plt.show()

    def next(self, event):
        self.idx += 1
        self.update()

    def prev(self, event):
        self.idx -= 1
        self.update()

    def press(self, event):
        if event.key == 'left':
            self.idx += 1
            self.update()
            
        elif event.key == 'right':
            self.idx += 1
            self.update()          

    def update(self):
        try:
            self.curscan_show.remove()
            self.odom_show.remove()
        except:
            pass
        print self.idx
        scan = self.data[self.idx][0]
        odom = np.matrix(self.data[self.idx][1])
        scan_size = scan.shape[0]
        scan = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan = np.vstack([scan, tmp])
        tmp.fill(1)
        scan = np.vstack([scan, tmp])
        scan = np.matrix(scan)
        scan = odom*self.scan_base*scan
        scan = np.array(scan)
        curp = odom*self.init_pose 
        v = np.matrix([[1],[0]])
        v= odom[0:2,0:2]*v
        self.odom_show = self.ax.quiver(curp[0,0], curp[1,0], v[0,0], v[1,0], units='width')
        self.scan_show = self.ax.scatter(scan[0,:],scan[1,:],c='g', s=5)
        self.curscan_show = self.ax.scatter(scan[0,:],scan[1,:],c='r', s=10)
        plt.draw()

if __name__ == "__main__":
    bagreader = BagReader('h1.bag', 'scan', 'odom',0,800)
    gui = GUI(bagreader)