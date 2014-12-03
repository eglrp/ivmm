#!/usr/bin/env python
#-*- coding:utf-8 -*-

from qgis.core import *
from qgis.gui import *
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import pyivmm

class ShortestPathController:
    #def __init__(self, network = pyivmm.Network(), ui = dock.Ui_pyivmm(), mapCanvas = QgsMapCanvas()):
    def __init__(self, network , ui,  mapCanvas ):
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
        self.pointLayer = QgsVectorLayer("Point?crs=epsg:3785&field=id:integer&field=x:double&field=y:double&index=yes"
            , "ShortestPathPoint", "memory")
        provider = self.pointLayer.dataProvider()
        QgsMapLayerRegistry.instance().addMapLayer(self.pointLayer)
        self.pointLayer.layerDeleted.connect(self.pointLayerDeleted)

    def createPathLayer(self):
        self.pathLayer = QgsVectorLayer("LineString?crs=epsg:3785&field=length:double","ShortestPath", "memory")
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
        self.pointLayer.setCacheImage(None)
        self.mapCanvas.update()

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
