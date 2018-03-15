#!/usr/bin/env python

import os
import rospkg
import rospy
import vtk

from numpy import random
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QFrame, QVBoxLayout, QWidget
from sensor_msgs.msg import PointCloud
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


class PclData(object):
    def __init__(self, topic, topic_type):
        self.name = topic
        self.points = None
        self.do_update = False

        topic_name = '/rosarnl_node/sim_lms1xx_1_pointcloud'
        self.sub = rospy.Subscriber(topic_name, PointCloud, self._ros_cb)

    def close(self):
        self.sub.unregister()

    def _ros_cb(self, data):
        self.points = data.points
        self.do_update = True


class PclVisualizer(object):
    def __init__(self, zmin=-10.0, zmax=10.0, maxpoints=10e6):
        self.maxpoints = maxpoints

        self.polydata = vtk.vtkPolyData()
        self._clearPoints()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.polydata)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zmin, zmax)
        mapper.SetScalarVisibility(1)

        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def _addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxpoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxpoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def _clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')

        self.polydata.SetPoints(self.vtkPoints)
        self.polydata.SetVerts(self.vtkCells)
        self.polydata.GetPointData().SetScalars(self.vtkDepth)
        self.polydata.GetPointData().SetActiveScalars('DepthArray')

    def update(self, points):
        self._clearPoints()
        for point in points:
            self._addPoint([point.x, point.y, point.z])
        self.polydata.Modified()


class PclWidget(QWidget):
    def __init__(self, topic=None):
        super(PclWidget, self).__init__()
        self.setObjectName('PclWidget')

        # Create subscriber
        if topic:
            self._pclData = PclData(topic, PointCloud)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_pcl'), 'resource', 'RosPcl.ui')
        loadUi(ui_file, self)

        self.vtkWidget = QVTKRenderWindowInteractor(self)
        self.data_layout.addWidget(self.vtkWidget)

        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        self.pointcloud = PclVisualizer()
        self.ren.AddActor(self.pointcloud.vtkActor)
        self.ren.ResetCamera()

        # Init and start update timer
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self.update_plot)
        self._update_timer.start(100)

        self.iren.Initialize()

    def update_plot(self):
        if self._pclData.do_update:
            self.pointcloud.update(self._pclData.points)
            self._pclData.do_update = False

            self.ren.ResetCamera()
            self.iren.Render()
