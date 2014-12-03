#!/usr/bin/env python
#-*- coding:utf-8 -*-


import pyivmm
import math
import dock
from qgis.core import  *
from qgis.gui import *
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import time
import os

def listFromTxtFile(filename):
    f = open(filename)
    if not f:
        return None
    gpsList = []
    for line in f.readlines():
        meta = line.split(',')
        if len(meta ) == 9:
            _id = meta[0]
            _dateStr = meta[3]
            x = float(meta[4])
            y = float(meta[5])
            gpsList.append([x, y, long(time.mktime(time.strptime(_dateStr, '%Y%m%d%H%M%S'))) ] )
        elif len(meta) == 6:
            _id = meta[0]
            _dateStr = meta[1]
            x = float(meta[4])
            y = float(meta[5])
            gpsList.append([x, y, long(time.mktime(time.strptime(_dateStr, '%Y-%m-%d %H:%M:%S'))) ] )
        else:
            _id = meta[0]
            _dateStr = meta[1]
            x = float(meta[2])
            y = float(meta[3])
            gpsList.append([x, y, long(time.mktime(time.strptime(_dateStr, '%Y-%m-%d %H:%M:%S'))) ] )

    return gpsList

class GisGpsLoaderController(QObject):
    gpsLoad = pyqtSignal(list)
    def __init__(self, network , ui , mapCanvas):
        QObject.__init__(self)
        self.network = network
        self.ui = ui
        self.mapCanvas = mapCanvas
        self.ui.gpsTab.setEnabled(True)
        self.ui.gpsLoadButton.clicked.connect( self.loadNewGps)
        self.ui.candidateCreateButton.clicked.connect( self.createCandidate )
        self.ui.trajLoadButton.clicked.connect ( self.loadNewTraj)
        self.ui.rawTrajButton.clicked.connect( self.createRawTraj )
        self.candidateLayer = None
        self.gpsList = None
        self.gpsLogName = None

        self.gpsLastOpenDir = ""

    def getGpsLayer(self , name):
        gpsLayer = QgsVectorLayer('Point?crs=epsg:3785&field=id:integer&field=data:string(20)&index=yes',
            'Gps-'+name, 'memory')
        QgsMapLayerRegistry.instance().addMapLayer(gpsLayer)
        return gpsLayer
    def getRawTrajLayer(self, name):
        #rawTrajLayer = QgsVectorLayer('LineString?crs=epsg:3785&field=id:integer&field=vx:double&field=vy:double&field=speed:double&field=length:double&field=time:integer&index=yes'
        #    'rawTraj-' + name, 'memory')
        rawTrajLayer = QgsVectorLayer('LineString?crs=epsg:3785&field=id:integer&field=vx:double&field=vy:double&field=speed:double&index=yes', 'rawTraj-' +name, 'memory')
        QgsMapLayerRegistry().instance().addMapLayer(rawTrajLayer)
        print 'trajLayer', rawTrajLayer.isValid()
        return rawTrajLayer

    def createRawTraj(self ):
        if not self.gpsList:
            return

        rawTrajLayer = self.getRawTrajLayer(self.gpsLogName)
        provider = rawTrajLayer.dataProvider()
        print 'provider', provider
        features = []
        for i in range(len(self.gpsList) - 1):
            p1 = self.gpsList[i]
            p2 = self.gpsList[i+1]
            time = p2[2] - p1[2] * 1.0
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            vx = dx / time if time != 0 else float('inf') if dx == 0.0 else float('nan')
            vx = float('%.2f'%vx)
            vy = dy / time if time != 0 else float('inf') if dy == 0.0 else float('nan')
            vy = float('%.2f'%vy)
            length = math.sqrt(dx*dx + dy*dy)
            length = float('%.2f'%length)
            speed = length / time if time != 0 else float('inf') if length == 0.0 else float('nan')
            speed = float('%.2f'%speed)
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPolyline([QgsPoint(p1[0],p1[1]),QgsPoint(p2[0],p2[1])]));
            feature.setAttributes([i,vx,vy,speed, length, time ])
            features.append(feature)
        provider.addFeatures(features)
        provider.updateExtents()
        self.mapCanvas.update()

    def loadNewGps(self ):
        gpsFileName = QFileDialog.getOpenFileName(self.mapCanvas, 'gps log', self.gpsLastOpenDir, 'txt(*.txt)')
        if gpsFileName == '':
            return
        self.gpsLastOpenDir = os.path.dirname(gpsFileName)
        self.ui.gpsFileLineEdit.setText(gpsFileName)
        self.gpsList = listFromTxtFile(gpsFileName)
        if not self.gpsList:
            return

        self.gpsLogName = os.path.basename(gpsFileName)
        layer = self.getGpsLayer( self.gpsLogName )#self.safeGetGpsLayer()
        provider = layer.dataProvider()
        fetList = []
        for i,gps in enumerate(self.gpsList):
            fet = QgsFeature()
            fet.setGeometry(QgsGeometry.fromPoint(QgsPoint(gps[0], gps[1])))
            fet.setAttributes([i, time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(gps[2]))])
            fetList.append(fet)
        ids = []
        for oldFet in provider.getFeatures():
            ids.append(oldFet.id())
        provider.deleteFeatures(ids)
        provider.addFeatures(fetList)
        provider.updateExtents()
        self.mapCanvas.update()

    def createCandidate(self ):
        pass

    def loadNewTraj( self ):
        trajFileName = QFileDialog.getOpenFileName(self.mapCanvas, "select traj", "", "txt(*.txt)")
        if trajFileName == '':
            return

        f = open(trajFileName)
        l = []
        for line in f.readlines():
            items = line.split(',')
            r = self.network.road(items[0])
            c1 = self.network.cross(items[1])
            c2 = self.network.cross(items[2])
            enterTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(long(items[3])))
            leaveTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(long(items[4])))
            cost = long(items[5])
            l.append( ( r, c1, c2, enterTime, leaveTime, cost))

        trajLayer = QgsVectorLayer(
            "LineString?crs=epsg:3785&field=id:integer&field=road:string&field=enter:string&field=leave:string&field=cost:integer&index=yes",
            "traj-"+os.path.basename( trajFileName),
            "memory")
        dp = trajLayer.dataProvider()
        feats= []
        for i, (r, c1, c2, enterTime, leaveTime, cost) in enumerate(l):
            feat = QgsFeature()
            points = []
            for p in r.points:
                points.append(QgsPoint(p.x, p.y))
            if r.begin == c2:
                points.reverse()
            feat.setGeometry(QgsGeometry.fromPolyline(points))
            feat.setAttributes([i, r.dbID, enterTime, leaveTime, cost])
            feats.append(feat)

        dp.addFeatures(feats)
        dp.updateExtents()
        QgsMapLayerRegistry.instance().addMapLayer(trajLayer)
