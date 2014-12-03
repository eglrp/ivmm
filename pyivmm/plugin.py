#!/usr/bin/env python
#-*- coding:utf-8 -*-

from qgis.core import *
from qgis.gui import *
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pyivmm
import dock
from os import path
from shortest_path import ShortestPathController
from ivmm import IVMMComputeController
from gps import GisGpsLoaderController

import re



class GisPointValidator( QValidator ):
    def __init__(self, *argc, ** argv):
        QValidator.__init__(self, *argc, **argv)

    def validate( self, instr, pos ):
        if re.match(r'^\d*(\.\d*)?(,\d*(\.\d*)?)?$', instr):
            return (QValidator.Acceptable, instr, pos)
        return (QValidator.Invalid, instr, pos)

class  MainPlugin:

    def __init__(self, iface ):
        self.iface = iface
        self.mainWindow = iface.mainWindow()
        self.ui = dock.Ui_pyivmm()
        self.mapCanvas = iface.mapCanvas()
        self.roadShpFilePath = None
        self.network = pyivmm.Network()
        self.shortestPathController = None
        self.ivmmComputeController = None
        self.pathLoadController = None

    def initGui(self):
        self.dockWidget = QDockWidget("pyivmm")
        self.ui.setupUi( self.dockWidget )
        self.ui.roadLoadButton.clicked.connect(self.loadRoad)
        self.ui.gotoButton.clicked.connect(self.gotoViewPoint)
        self.ui.goToCrossButton.clicked.connect(self.gotoCross)
        self.ui.centerLineEdit.setValidator(GisPointValidator())
        self.iface.addDockWidget(Qt.BottomDockWidgetArea, self.dockWidget)

    def gotoCross(self):
        crossDBID = self.ui.crossLineEdit.text()
        try:
            crossDBID = str(crossDBID)
        except:
            return
        if self.network.contain_cross(crossDBID):
            cross = self.network.cross(crossDBID)
            x = cross.x
            y = cross.y
            reg = QgsRectangle(x - 10, y - 10, x + 10, y + 10)
            self.mapCanvas.setExtent(reg)
            self.mapCanvas.refresh()

    def gotoViewPoint(self ):
        pointstr = self.ui.centerLineEdit.text()
        xy = pointstr.split(',')
        if len(xy) != 2:
            return

        x = float(xy[0])
        y = float(xy[1])
        reg = QgsRectangle(x - 10, y - 10, x + 10, y + 10)
        self.mapCanvas.setExtent(reg)
        self.mapCanvas.refresh()
        #transform = self.mapCanvas.getCoordinateTransform()
        #point = transform.transform(x, y)
        #print 'after transform', point
        #self.mapCanvas.zoomWithCenter(point.x(), point.y(), False)


    def unload(self):
        self.dockWidget.deleteLater()

    def loadRoad(self ):
        self.roadShpFilePath = QFileDialog.getOpenFileName(self.mainWindow,"road shp",
            "", "SHP (*.shp)")
        if not self.roadShpFilePath:
            return
        layer = self.iface.addVectorLayer(self.roadShpFilePath, path.basename(self.roadShpFilePath), "ogr")
        if layer:
            layer.layerDeleted.connect(self.networkPartDeleted)
            self.ui.roadLineEdit.setText(self.roadShpFilePath)
            self.build_network()
        else:
            self.iface.messageBar().pushMessage("fail to load road", QgsMessageBar.WARNING, 5)

    def build_network(self):
        success = False
        if self.roadShpFilePath:
            if self.network.load(self.roadShpFilePath.encode('utf-8')):
                success = True
        if success:
            self.networkBuildSuccess()
            self.iface.messageBar().pushMessage("success to build network", QgsMessageBar.INFO, 5)
        else:
            self.iface.messageBar().pushMessage("fail to build network", QgsMessageBar.WARNING, 5)

    def networkPartDeleted(self):
        print 'networkPartDeleted'

    def networkBuildSuccess(self):
        self.shortestPathController = ShortestPathController( self.network, self.ui, self.mapCanvas )
        self.ivmmComputeController = IVMMComputeController( self.network, self.ui, self.mapCanvas )
        self.gpsLoadControaller = GisGpsLoaderController( self.network, self.ui, self.mapCanvas)

        self.gpsLoadControaller.gpsLoad.connect(self.ivmmComputeController.onGpsLoad)
