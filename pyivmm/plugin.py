#!/usr/bin/env python
#-*- coding:utf-8 -*-

from qgis.core import *
from qgis.gui import *
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pyivmm
import dock
from os import path

import re



class GisPointValidator( QValidator ):
    def __init__(self, *argc, ** argv):
        QValidator.__init__(self, *argc, **argv)

    def validate( self, instr, pos ):
        if re.match(r'^\d*(\.\d*)?(,\d*(\.\d*)?)?$', instr):
            return (QValidator.Acceptable, instr, pos)
        return (QValidator.Invalid, instr, pos)


class ShortestPathController:
    def __init__(self, network = pyivmm.Network(), ui = dock.Ui_pyivmm(), mapCanvas = QgsMapCanvas()):
        self.network = network
        self.ui = ui
        self.mapCanvas = mapCanvas
        self.pointPickerTool = QgsMapToolEmitPoint(self.mapCanvas)
        self.pointPickerTool.canvasClicked.connect(self.onPickPoint)
        self.ui.pickPointButton.clicked.connect(self.startToPick)
        self.points = []

        self.pointLayer = None
        self.pathLayer = None

        self.ui.shortestPathTab.setEnabled(True)

        print "I am create!"

    def createPointLayer(self):
        self.pointLayer = QgsVectorLayer("Point?crs=epsg:4326&field=id:integer&field=x:double&field=y:double&index=yes"
            , "ShortestPathPoint", "memory")
        provider = self.pointLayer.dataProvider()
        QgsMapLayerRegistry.instance().addMapLayer(self.pointLayer)
        self.pointLayer.layerDeleted.connect(self.pointLayerDeleted)

    def createPathLayer(self):
        self.pathLayer = QgsVectorLayer("LineString?crs=epsg:4326&field=length:double","ShortestPath", "memory")
        provider = self.pathLayer.dataProvider()
        QgsMapLayerRegistry.instance().addMapLayer(self.pathLayer)
        self.pathLayer.layerDeleted.connect(self.pathLayerDeleted)

    def pointLayerDeleted(self):
        self.pointLayer = None

    def pathLayerDeleted(self):
        self.pathLayer = None


    def addPoint(self, point):
        if self.pointLayer is None:
            self.createPointLayer()
        feature = QgsFeature()
        feature.setGeometry( QgsGeometry.fromPoint(point) )
        feature.setAttributes([self.pointLayer.dataProvider().featureCount(), point.x(), point.y()])
        self.pointLayer.dataProvider().addFeatures([feature])
        self.pointLayer.updateExtents()
        self.mapCanvas.refresh()

    def onPickPoint( self, point, button ):
        if button != Qt.LeftButton:
            self.mapCanvas.unsetMapTool(self.pointPickerTool)
            return
        self.points.append( pyivmm.Point( point.x(), point.y() ) )
        if len(self.points) == 1:
            self.ui.beginPointLineEdit.setText("%g,%g"%(point.x(), point.y()))
        elif len(self.points) == 2:
            self.ui.endPointLineEdit.setText("%g,%g"%(point.x(), point.y()))
            self.mapCanvas.unsetMapTool(self.pointPickerTool)
            try:
                self.doShortestPath()
            except e:
                print e

        self.addPoint(point)

    def startToPick( self):
        self.points = []
        if self.pointLayer is None:
            self.createPointLayer()
        iter = self.pointLayer.dataProvider().getFeatures()
        idList = []
        for feature in iter:
            idList.append(feature.id())
        self.pointLayer.dataProvider().deleteFeatures(idList)
        self.pointLayer.updateExtents()
        self.mapCanvas.update()
        self.ui.beginPointLineEdit.clear()
        self.ui.endPointLineEdit.clear()
        self.ui.shortestLengthLabel.clear()
        self.mapCanvas.setMapTool(self.pointPickerTool)

    def doShortestPath(self):
        cp1 = self.network.project(self.points[0])
        cp2 = self.network.project(self.points[1])
        path = self.network.shortest_path(cp1, cp2)
        self.ui.shortestLengthLabel.setText("%f" % path.length)
        if self.pathLayer is None:
            self.createPathLayer()

        points = [ QgsPoint(p.x, p.y) for p in path.points ]

        iter = self.pathLayer.dataProvider().getFeatures()
        idList = []
        for feature in iter:
            idList.append(feature.id())
        self.pathLayer.dataProvider().deleteFeatures(idList)
        feature = QgsFeature()
        feature.setAttributes([ path.length ])
        feature.setGeometry(QgsGeometry.fromPolyline( points ))
        self.pathLayer.dataProvider().addFeatures([feature])
        self.pathLayer.updateExtents()
        self.pathLayer.setCacheImage(None)
        self.mapCanvas.update()


class  MainPlugin:
    
    def __init__(self, iface ):
        self.iface = iface
        self.mainWindow = iface.mainWindow()
        self.ui = dock.Ui_pyivmm()
        self.mapCanvas = iface.mapCanvas()
        self.roadShpFilePath = None
        self.network = pyivmm.Network()
        self.shortestPathController = None
        #self.ui.centerLineEdit.setValidator(QValidator(self.mainWindow))

    def initGui(self):
        self.dockWidget = QDockWidget("pyivmm")
        self.ui.setupUi( self.dockWidget )
        self.ui.roadLoadButton.clicked.connect(self.loadRoad)
        self.ui.gotoButton.clicked.connect(self.gotoViewPoint)
        self.ui.centerLineEdit.setValidator(GisPointValidator())
        self.iface.addDockWidget(Qt.BottomDockWidgetArea, self.dockWidget)


    def gotoViewPoint(self ):
        pointstr = self.ui.centerLineEdit.text()
        xy = pointstr.split(',')
        if len(xy) != 2:
            return

        x = float(xy[0])
        y = float(xy[1])
        reg = QgsRectangle(x - 0.01, y - 0.01, x + 0.01, y + 0.01)
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
            self.shortestPathController = ShortestPathController( self.network, self.ui, self.mapCanvas )
            self.iface.messageBar().pushMessage("success to build network", QgsMessageBar.INFO, 5)
        else:
            self.iface.messageBar().pushMessage("fail to build network", QgsMessageBar.WARNING, 5)
    
    def networkPartDeleted(self):
        print 'networkPartDeleted'
