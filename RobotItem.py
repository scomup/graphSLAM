#!/usr/bin/python
# coding: UTF-8

from pyqtgraph.Qt import QtCore, QtGui

class RobotItem(QtGui.QGraphicsItem):
    """a sample robot item"""
    def __init__(self, color):
        super(RobotItem, self).__init__()
        #self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
        self.setCacheMode(QtGui.QGraphicsItem.DeviceCoordinateCache)
        self.setZValue(1)
        self.color = color
        
    def boundingRect(self):
        adjust = 2.0
        return QtCore.QRectF(-10 - adjust, -10 - adjust, 20 + adjust,
                20 + adjust)

    def paint(self, painter, option, widget):
        #Draw a sample robot
        pen = QtGui.QPen()
        pen.setWidth(1);
        if self.color =='r':
            pen.setBrush(QtCore.Qt.red)
        elif self.color =='b':
            pen.setBrush(QtCore.Qt.blue)
        else:
            pen.setBrush(QtCore.Qt.green)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(0.0, 0.0), 5, 5)
        painter.drawLine(0, 0, 5, 0)