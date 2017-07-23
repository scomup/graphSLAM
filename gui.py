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

class graphSLAM_GUI(QtGui.QTabWidget): 
    def __init__(self,parent=None):
        self.q = Queue.Queue()
        super(graphSLAM_GUI,self).__init__(parent)
        self.ceate_tab_pointcloud()
        self.ceate_tab_map()

    def ceate_tab_map(self):
        #  Create tab_map 
        tab = QtGui.QTabWidget()
        self.addTab(tab, "map")
        layout = QtGui.QGridLayout()
        tab.setLayout(layout)
        gv = pg.GraphicsView()
        layout.addWidget(gv, 0, 0, 1, 5)
        # Create a viewBox for 2D image
        vb = pg.ViewBox()
        vb.setAspectLocked()
        gv.setCentralItem(vb)
        #Create ImageItem for map
        self.img = pg.ImageItem(np.zeros((400,400)))
        vb.addItem(self.img)
        #self.img.setImage(np.random.rand(400,400))

    def ceate_tab_pointcloud(self):
        #  Create tab_pointcloud 
        tab = QtGui.QTabWidget()
        self.addTab(tab, "point cloud")
        layout = QtGui.QGridLayout()
        tab.setLayout(layout)
        gv = pg.GraphicsView()
        layout.addWidget(gv, 0, 0, 1, 5)
        # Create a viewBox for 2D image
        vb = pg.ViewBox()
        vb.setAspectLocked()
        gv.setCentralItem(vb)
        #Create ImageItem for map
        self.pcd = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
        vb.addItem(self.pcd)
        #Set timer


class graphSLAM_GUI_Thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.q_map = Queue.Queue()
        self.q_pcd = Queue.Queue()

        self.state = 0


        #Set timer
        #timer = pg.QtCore.QTimer()
        #timer.timeout.connect(self.update)
        #timer.start(300)

    def run(self):
        app=QtGui.QApplication(sys.argv) 
        self.gui=graphSLAM_GUI() 
        self.gui.setWindowTitle("Gui")
        self.gui.resize(800,600)
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(300)

        self.gui.show() 
        app.exec_()

    def update(self):
        try:
            #Check is there any new data in queue
            data = self.q_map.get(block=False)
            self.q_map.queue.clear()
            self.gui.img.setImage(data.transpose())
        except Queue.Empty:
            pass
        try:
            data = self.q_pcd.get(block=False)
            self.q_pcd.queue.clear()
            spots = [{'pos': data[i,:] } for i in range(data.shape[0])]
            self.gui.pcd.addPoints(spots)
        except Queue.Empty:
            pass

    def setmap(self, data):
        self.q_map.put( data )
        pass

    def setpcd(self, data):
        self.q_pcd.put( data )
        pass


if __name__ == "__main__":
    gui = graphSLAM_GUI_Thread()
    gui.start()
    
    print 'sample gui test'
    for i in range(2000):
        time.sleep(0.05)
        gui.setmap(np.random.rand(400,400))
        gui.setpcd(np.random.rand(400,2))
    


