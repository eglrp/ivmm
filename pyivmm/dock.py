# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dock.ui'
#
# Created: Sat Aug  2 14:41:41 2014
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_pyivmm(object):
    def setupUi(self, pyivmm):
        pyivmm.setObjectName(_fromUtf8("pyivmm"))
        pyivmm.resize(485, 300)
        pyivmm.setFloating(False)
        self.dockWidgetContents = QtGui.QWidget()
        self.dockWidgetContents.setObjectName(_fromUtf8("dockWidgetContents"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.dockWidgetContents)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.tabWidget = QtGui.QTabWidget(self.dockWidgetContents)
        self.tabWidget.setEnabled(True)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.mapLoadTab = QtGui.QWidget()
        self.mapLoadTab.setObjectName(_fromUtf8("mapLoadTab"))
        self.verticalLayout = QtGui.QVBoxLayout(self.mapLoadTab)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_4 = QtGui.QLabel(self.mapLoadTab)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_2.addWidget(self.label_4)
        self.roadLineEdit = QtGui.QLineEdit(self.mapLoadTab)
        self.roadLineEdit.setReadOnly(True)
        self.roadLineEdit.setObjectName(_fromUtf8("roadLineEdit"))
        self.horizontalLayout_2.addWidget(self.roadLineEdit)
        self.roadLoadButton = QtGui.QPushButton(self.mapLoadTab)
        self.roadLoadButton.setObjectName(_fromUtf8("roadLoadButton"))
        self.horizontalLayout_2.addWidget(self.roadLoadButton)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_5 = QtGui.QLabel(self.mapLoadTab)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_3.addWidget(self.label_5)
        self.centerLineEdit = QtGui.QLineEdit(self.mapLoadTab)
        self.centerLineEdit.setReadOnly(False)
        self.centerLineEdit.setObjectName(_fromUtf8("centerLineEdit"))
        self.horizontalLayout_3.addWidget(self.centerLineEdit)
        self.gotoButton = QtGui.QPushButton(self.mapLoadTab)
        self.gotoButton.setObjectName(_fromUtf8("gotoButton"))
        self.horizontalLayout_3.addWidget(self.gotoButton)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem = QtGui.QSpacerItem(20, 127, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.tabWidget.addTab(self.mapLoadTab, _fromUtf8(""))
        self.shortestPathTab = QtGui.QWidget()
        self.shortestPathTab.setEnabled(False)
        self.shortestPathTab.setObjectName(_fromUtf8("shortestPathTab"))
        self.gridLayout = QtGui.QGridLayout(self.shortestPathTab)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label = QtGui.QLabel(self.shortestPathTab)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.beginPointLineEdit = QtGui.QLineEdit(self.shortestPathTab)
        self.beginPointLineEdit.setReadOnly(True)
        self.beginPointLineEdit.setObjectName(_fromUtf8("beginPointLineEdit"))
        self.gridLayout.addWidget(self.beginPointLineEdit, 0, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.shortestPathTab)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 0, 2, 1, 1)
        self.endPointLineEdit = QtGui.QLineEdit(self.shortestPathTab)
        self.endPointLineEdit.setReadOnly(True)
        self.endPointLineEdit.setObjectName(_fromUtf8("endPointLineEdit"))
        self.gridLayout.addWidget(self.endPointLineEdit, 0, 3, 1, 1)
        self.pickPointButton = QtGui.QPushButton(self.shortestPathTab)
        self.pickPointButton.setObjectName(_fromUtf8("pickPointButton"))
        self.gridLayout.addWidget(self.pickPointButton, 0, 4, 1, 1)
        self.label_3 = QtGui.QLabel(self.shortestPathTab)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout.addWidget(self.label_3, 1, 0, 1, 1)
        self.shortestLengthLabel = QtGui.QLabel(self.shortestPathTab)
        self.shortestLengthLabel.setText(_fromUtf8(""))
        self.shortestLengthLabel.setObjectName(_fromUtf8("shortestLengthLabel"))
        self.gridLayout.addWidget(self.shortestLengthLabel, 1, 1, 1, 4)
        spacerItem1 = QtGui.QSpacerItem(20, 139, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem1, 2, 2, 1, 1)
        self.tabWidget.addTab(self.shortestPathTab, _fromUtf8(""))
        self.gps_loader = QtGui.QWidget()
        self.gps_loader.setEnabled(False)
        self.gps_loader.setObjectName(_fromUtf8("gps_loader"))
        self.gridLayout_2 = QtGui.QGridLayout(self.gps_loader)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.gps_log_loader = QtGui.QPushButton(self.gps_loader)
        self.gps_log_loader.setObjectName(_fromUtf8("gps_log_loader"))
        self.gridLayout_2.addWidget(self.gps_log_loader, 0, 4, 1, 1)
        self.candidate_main_end = QtGui.QLineEdit(self.gps_loader)
        self.candidate_main_end.setObjectName(_fromUtf8("candidate_main_end"))
        self.gridLayout_2.addWidget(self.candidate_main_end, 3, 1, 1, 1)
        self.candidate_sub_start = QtGui.QLineEdit(self.gps_loader)
        self.candidate_sub_start.setObjectName(_fromUtf8("candidate_sub_start"))
        self.gridLayout_2.addWidget(self.candidate_sub_start, 2, 3, 1, 1)
        self.label_10 = QtGui.QLabel(self.gps_loader)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_2.addWidget(self.label_10, 3, 0, 1, 1)
        spacerItem2 = QtGui.QSpacerItem(20, 163, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem2, 4, 2, 1, 1)
        self.label_7 = QtGui.QLabel(self.gps_loader)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gridLayout_2.addWidget(self.label_7, 2, 0, 1, 1)
        self.label_8 = QtGui.QLabel(self.gps_loader)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout_2.addWidget(self.label_8, 2, 2, 1, 1)
        self.candidate_main_start = QtGui.QLineEdit(self.gps_loader)
        self.candidate_main_start.setObjectName(_fromUtf8("candidate_main_start"))
        self.gridLayout_2.addWidget(self.candidate_main_start, 2, 1, 1, 1)
        self.label_9 = QtGui.QLabel(self.gps_loader)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_2.addWidget(self.label_9, 3, 2, 1, 1)
        self.candidate_sub_end = QtGui.QLineEdit(self.gps_loader)
        self.candidate_sub_end.setObjectName(_fromUtf8("candidate_sub_end"))
        self.gridLayout_2.addWidget(self.candidate_sub_end, 3, 3, 1, 1)
        self.candidate_to_map = QtGui.QPushButton(self.gps_loader)
        self.candidate_to_map.setObjectName(_fromUtf8("candidate_to_map"))
        self.gridLayout_2.addWidget(self.candidate_to_map, 1, 4, 1, 1)
        self.compute_candidate_path = QtGui.QPushButton(self.gps_loader)
        self.compute_candidate_path.setObjectName(_fromUtf8("compute_candidate_path"))
        self.gridLayout_2.addWidget(self.compute_candidate_path, 2, 4, 1, 1)
        self.candidate_path_length_show = QtGui.QLabel(self.gps_loader)
        self.candidate_path_length_show.setText(_fromUtf8(""))
        self.candidate_path_length_show.setObjectName(_fromUtf8("candidate_path_length_show"))
        self.gridLayout_2.addWidget(self.candidate_path_length_show, 3, 4, 1, 1)
        self.topk = QtGui.QSpinBox(self.gps_loader)
        self.topk.setMaximum(1000)
        self.topk.setObjectName(_fromUtf8("topk"))
        self.gridLayout_2.addWidget(self.topk, 0, 3, 1, 1)
        self.gps_log_show = QtGui.QLineEdit(self.gps_loader)
        self.gps_log_show.setReadOnly(True)
        self.gps_log_show.setObjectName(_fromUtf8("gps_log_show"))
        self.gridLayout_2.addWidget(self.gps_log_show, 0, 0, 1, 2)
        self.label_15 = QtGui.QLabel(self.gps_loader)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_2.addWidget(self.label_15, 0, 2, 1, 1)
        self.label_6 = QtGui.QLabel(self.gps_loader)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout_2.addWidget(self.label_6, 1, 0, 1, 1)
        self.radiou_query_candidate = QtGui.QDoubleSpinBox(self.gps_loader)
        self.radiou_query_candidate.setMaximum(500.0)
        self.radiou_query_candidate.setObjectName(_fromUtf8("radiou_query_candidate"))
        self.gridLayout_2.addWidget(self.radiou_query_candidate, 1, 1, 1, 1)
        self.label_17 = QtGui.QLabel(self.gps_loader)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.gridLayout_2.addWidget(self.label_17, 1, 2, 1, 1)
        self.candidate_limit = QtGui.QSpinBox(self.gps_loader)
        self.candidate_limit.setMinimum(1)
        self.candidate_limit.setObjectName(_fromUtf8("candidate_limit"))
        self.gridLayout_2.addWidget(self.candidate_limit, 1, 3, 1, 1)
        self.tabWidget.addTab(self.gps_loader, _fromUtf8(""))
        self.ivmm = QtGui.QWidget()
        self.ivmm.setEnabled(False)
        self.ivmm.setObjectName(_fromUtf8("ivmm"))
        self.gridLayout_3 = QtGui.QGridLayout(self.ivmm)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        spacerItem3 = QtGui.QSpacerItem(20, 108, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gridLayout_3.addItem(spacerItem3, 5, 1, 1, 1)
        self.compute_best_candidate_button = QtGui.QPushButton(self.ivmm)
        self.compute_best_candidate_button.setObjectName(_fromUtf8("compute_best_candidate_button"))
        self.gridLayout_3.addWidget(self.compute_best_candidate_button, 0, 2, 1, 1)
        self.label_11 = QtGui.QLabel(self.ivmm)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_3.addWidget(self.label_11, 0, 0, 1, 1)
        self.param_mu = QtGui.QLineEdit(self.ivmm)
        self.param_mu.setObjectName(_fromUtf8("param_mu"))
        self.gridLayout_3.addWidget(self.param_mu, 0, 1, 1, 1)
        self.label_12 = QtGui.QLabel(self.ivmm)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_3.addWidget(self.label_12, 1, 0, 1, 1)
        self.param_sigma = QtGui.QLineEdit(self.ivmm)
        self.param_sigma.setObjectName(_fromUtf8("param_sigma"))
        self.gridLayout_3.addWidget(self.param_sigma, 1, 1, 1, 1)
        self.label_13 = QtGui.QLabel(self.ivmm)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_3.addWidget(self.label_13, 2, 0, 1, 1)
        self.param_beta = QtGui.QLineEdit(self.ivmm)
        self.param_beta.setObjectName(_fromUtf8("param_beta"))
        self.gridLayout_3.addWidget(self.param_beta, 2, 1, 1, 1)
        self.label_14 = QtGui.QLabel(self.ivmm)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.gridLayout_3.addWidget(self.label_14, 3, 0, 1, 1)
        self.param_radious = QtGui.QLineEdit(self.ivmm)
        self.param_radious.setObjectName(_fromUtf8("param_radious"))
        self.gridLayout_3.addWidget(self.param_radious, 3, 1, 1, 1)
        self.label_16 = QtGui.QLabel(self.ivmm)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_3.addWidget(self.label_16, 4, 0, 1, 1)
        self.ivmm_candidate_limit = QtGui.QSpinBox(self.ivmm)
        self.ivmm_candidate_limit.setMinimum(1)
        self.ivmm_candidate_limit.setObjectName(_fromUtf8("ivmm_candidate_limit"))
        self.gridLayout_3.addWidget(self.ivmm_candidate_limit, 4, 1, 1, 1)
        self.tabWidget.addTab(self.ivmm, _fromUtf8(""))
        self.horizontalLayout.addWidget(self.tabWidget)
        pyivmm.setWidget(self.dockWidgetContents)

        self.retranslateUi(pyivmm)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(pyivmm)
        pyivmm.setTabOrder(self.roadLineEdit, self.beginPointLineEdit)
        pyivmm.setTabOrder(self.beginPointLineEdit, self.endPointLineEdit)
        pyivmm.setTabOrder(self.endPointLineEdit, self.pickPointButton)
        pyivmm.setTabOrder(self.pickPointButton, self.gps_log_show)
        pyivmm.setTabOrder(self.gps_log_show, self.topk)
        pyivmm.setTabOrder(self.topk, self.candidate_main_start)
        pyivmm.setTabOrder(self.candidate_main_start, self.candidate_sub_start)
        pyivmm.setTabOrder(self.candidate_sub_start, self.candidate_main_end)
        pyivmm.setTabOrder(self.candidate_main_end, self.candidate_sub_end)
        pyivmm.setTabOrder(self.candidate_sub_end, self.gps_log_loader)
        pyivmm.setTabOrder(self.gps_log_loader, self.candidate_to_map)
        pyivmm.setTabOrder(self.candidate_to_map, self.compute_candidate_path)
        pyivmm.setTabOrder(self.compute_candidate_path, self.param_mu)
        pyivmm.setTabOrder(self.param_mu, self.param_sigma)
        pyivmm.setTabOrder(self.param_sigma, self.param_beta)
        pyivmm.setTabOrder(self.param_beta, self.param_radious)
        pyivmm.setTabOrder(self.param_radious, self.ivmm_candidate_limit)
        pyivmm.setTabOrder(self.ivmm_candidate_limit, self.compute_best_candidate_button)

    def retranslateUi(self, pyivmm):
        pyivmm.setWindowTitle(_translate("pyivmm", "pyivmm", None))
        self.label_4.setText(_translate("pyivmm", "道路", None))
        self.roadLoadButton.setText(_translate("pyivmm", "载入道路", None))
        self.label_5.setText(_translate("pyivmm", "视角", None))
        self.gotoButton.setText(_translate("pyivmm", "前往", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.mapLoadTab), _translate("pyivmm", "载入地图", None))
        self.label.setText(_translate("pyivmm", "起点", None))
        self.label_2.setText(_translate("pyivmm", "终点", None))
        self.pickPointButton.setText(_translate("pyivmm", "拾取", None))
        self.label_3.setText(_translate("pyivmm", "路径长度", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.shortestPathTab), _translate("pyivmm", "最短路径", None))
        self.gps_log_loader.setText(_translate("pyivmm", "加载GPS数据", None))
        self.label_10.setText(_translate("pyivmm", "结束采样点编号", None))
        self.label_7.setText(_translate("pyivmm", "起始采样点编号", None))
        self.label_8.setText(_translate("pyivmm", "候选编号", None))
        self.label_9.setText(_translate("pyivmm", "候选编号", None))
        self.candidate_to_map.setText(_translate("pyivmm", "生成候选点", None))
        self.compute_candidate_path.setText(_translate("pyivmm", "候选点最短路径", None))
        self.label_15.setText(_translate("pyivmm", "top-k", None))
        self.label_6.setText(_translate("pyivmm", "半径", None))
        self.label_17.setText(_translate("pyivmm", "限制", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.gps_loader), _translate("pyivmm", "GPS加载", None))
        self.compute_best_candidate_button.setText(_translate("pyivmm", "计算最佳候选点", None))
        self.label_11.setText(_translate("pyivmm", "候选点偏差距离均值", None))
        self.label_12.setText(_translate("pyivmm", "偏差距离标准差", None))
        self.label_13.setText(_translate("pyivmm", "距离加权参数", None))
        self.label_14.setText(_translate("pyivmm", "候选半径", None))
        self.label_16.setText(_translate("pyivmm", "候选点数量限制", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.ivmm), _translate("pyivmm", "IVMM", None))
