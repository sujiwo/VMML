#!/usr/bin/python

import vtk
from vtk.util.numpy_support import numpy_to_vtk
import pcl
import sys


def loadPcd (pcdfile):
    pcd = pcl.PointCloud()
    pcd.from_file(pcdfile)
    array = numpy_to_vtk(pcd.to_array(), 1, vtk.VTK_FLOAT)
    return array


    
if __name__ == '__main__':

    pcdfile = loadPcd(sys.argv[1])
    pointsrc = vtk.vtkPoints()
    pointsrc.SetData(pcdfile)

    colors = vtk.vtkNamedColors()

    # create a rendering window and renderer
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    # create a renderwindowinteractor
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # create source
    src = vtk.vtkPointSource()
    src2 = vtk.vtkPoints()
    src.SetCenter(0, 0, 0)
    src.SetNumberOfPoints(50)
    src.SetRadius(5)
    src.Update()

    # mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(src.GetOutputPort())

    # actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(colors.GetColor3d("Tomato"))
    actor.GetProperty().SetPointSize(4)

    # assign actor to the renderer
    ren.AddActor(actor)
    ren.SetBackground(colors.GetColor3d("DarkGreen"))

    # enable user interface interactor
    iren.Initialize()
    renWin.Render()
    iren.Start()
