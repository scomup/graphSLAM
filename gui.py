#!/usr/bin/python
# coding: UTF-8

from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import signal
import Queue
import sys
import time
import threading
from RobotItem import RobotItem

import matplotlib.pyplot as plt

import warnings
warnings.filterwarnings("ignore")

class graphSLAM_GUI(QtGui.QTabWidget): 
    def __init__(self,parent=None):
        self.q = Queue.Queue()
        super(graphSLAM_GUI,self).__init__(parent)
        self.ceate_tab_pointcloud()
        self.ceate_tab_map()
        pg.setConfigOption('background', 'w')
        self.state = 0
        

    def set_slam_core(self,slam_core):
        self.slam_core = slam_core  

    def handleButton_play(self):
        try:
            self.slam_core.optimize()
        except:
            pass
        self.state = 1 # used by demo

    def handleButton_next(self):
        self.state = 2 

    def handleButton_createmap(self):
        self.slam_core.show_map()

    def handleButton_createmap_g(self):
        self.slam_core.show_map(True)

    def ceate_tab_map(self):
        #  Create tab_map 
        tab = QtGui.QTabWidget()
        self.addTab(tab, "map")
        layout = QtGui.QGridLayout()
        tab.setLayout(layout)
        gv = pg.GraphicsView()
        layout.addWidget(gv, 0, 0, 1, 5)
        # Create a viewBox for 2D image
        self.vb = pg.ViewBox()
        self.vb.setAspectLocked()
        gv.setCentralItem(self.vb)
        #Create ImageItem for map
        self.img = pg.ImageItem(np.zeros((1600,1600)))
        self.vb.addItem(self.img)

        button_createmap = QtGui.QPushButton('Create map')
        button_createmap.setFixedWidth(110)
        button_createmap.clicked.connect(self.handleButton_createmap)
        layout.addWidget(button_createmap, 2, 0)

        button_createmap_g = QtGui.QPushButton('Create map(G)')
        button_createmap_g.setFixedWidth(110)
        button_createmap_g.clicked.connect(self.handleButton_createmap_g)
        layout.addWidget(button_createmap_g, 2, 1)

    def ceate_tab_pointcloud(self):
        #  Create tab_pointcloud 
        tab = QtGui.QTabWidget()
        self.addTab(tab, "point cloud")
        layout = QtGui.QGridLayout()
        tab.setLayout(layout)
        gv = pg.GraphicsView()
        layout.addWidget(gv, 0, 0, 1, 5)
        # Create a viewBox for 2D image
        self.vb = pg.ViewBox()
        self.vb.setAspectLocked()
        gv.setCentralItem(self.vb)
        #Create ImageItem for map

        button_play = QtGui.QPushButton('Graph optimization')
        button_play.setFixedWidth(180)
        button_play.clicked.connect(self.handleButton_play)

        button_next = QtGui.QPushButton('Next')
        button_next.setFixedWidth(110)
        button_next.clicked.connect(self.handleButton_next)

        layout.addWidget(button_play, 2, 0)
        #layout.addWidget(button_next, 2, 1)

        self.pcd = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
        self.vb.addItem(self.pcd)

        self.cur_scan = pg.ScatterPlotItem(pen = pg.mkPen(None), 
            brush = pg.mkBrush("g"), 
            size =5, 
            antialias = False)
        self.vb.addItem(self.cur_scan)

        self.robot = RobotItem('b')
        #no robot show
        #self.robot.setParentItem(self.pcd)
        self.robot.scale(0.03,0.03)

        self.graph = pg.GraphItem(size=10, symbol="o", symbolPen="w", symbolBrush="y")
        self.vb.addItem(self.graph)

class graphSLAM_GUI_Thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.q_map = Queue.Queue()
        self.q_pcd = Queue.Queue()
        self.q_graph = Queue.Queue()
        self.state = 0

    def run(self):
        app=QtGui.QApplication(sys.argv) 
        self.guiobj=graphSLAM_GUI() 
        self.guiobj.setWindowTitle("graphSLAM v0.1")
        self.guiobj.resize(800,600)
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(300)
        self.guiobj.show() 
        app.exec_()

    def update(self):
        try:
            #Check is there any new data in queue
            data = self.q_map.get(block=False)
            self.q_map.queue.clear()
            self.guiobj.img.setImage(data.transpose())
        except Queue.Empty:
            pass
        try:
            data = self.q_pcd.get(block=False)
            self.q_pcd.queue.clear()
            #self.guiobj.cur_scan.clear()
            #self.guiobj.cur_scan.addPoints(data[:,0],data[:,1])

            self.guiobj.pcd.clear()
            self.guiobj.pcd.addPoints(data[:,0],data[:,1])
        except Queue.Empty:
            pass
        try:
            pos,adj, symbol = self.q_graph.get(block=False)
            adj.shape[0]
            lines = np.tile([255,0,0,255,1], (adj.shape[0],1))
            self.guiobj.graph.setData(pos=pos, adj=adj, pen=lines, symbol=symbol)
        except:
            pass

    def setmap(self, data):
        self.q_map.put( data )

    def setpcd(self, data):
        self.q_pcd.put( data )
      
    def setrobot(self, pose):
        self.guiobj.robot.setRotation(180.*pose[2]/np.pi)
        self.guiobj.robot.setPos(pose[0],pose[1])

    def setgraph(self, pos, adj, loop_nodes = None):
        
        if loop_nodes is not None:
            symbol = np.array(["x"]*len(pos))
            symbol[np.array(loop_nodes)] = 'o'
        else:
            symbol = np.array(["x"]*len(pos))
        self.q_graph.put( (pos, adj, symbol) )

if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()
    time.sleep(0.1)
    print 'sample gui test'
    for i in range(2000):
        time.sleep(0.05)
        gui.setmap(np.random.rand(400,400))
        gui.setpcd(np.random.rand(400,2))
        pos = np.random.rand(10,2)*10
        adj = np.array([
            [0,1],
            [1,3],
            [3,2],
            [2,0],
            [1,5],
            [3,5],
            ])
        gui.setgraph(pos, adj,[0])
    


