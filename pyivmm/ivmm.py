#!/usr/bin/env python
#-*- coding:utf-8 -*-

import pyivmm
import dock
import simplejson
class IVMMComputeController:
    def __init__(self, network, ui , mapCanvas):
        self.network = network
        self.ui = ui
        self.mapCanvas = mapCanvas
        self.ui.ivmmTab.setEnabled(True)
        self.ui.ivmmComputeButton.clicked.connect(self.ivmmCompute)
        self.ivmm = pyivmm.IVMM(self.network)

    def ivmmCompute(self ):
        param = pyivmm.IVMMParam()
        param.limit = self.ui.candidateLimitSpine.value()
        param.beta = float(self.ui.betaLineEdit.text())
        param.mu = float(self.ui.projectDistMuLineEdit.text())
        param.sigma = float(self.ui.projectDistStddevLineEdit.text())
        param.radious = float(self.ui.queryRadiousLineEdit.text())

    def onGpsLoad(self, gpsList):
        pass
