# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 10:15:21 2017

@author: ivan
"""
import numpy
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import Axes3D

from scipy.interpolate import griddata
from matplotlib import cm

from helperComputationFunctions import testEllipse


def initializeFigurePlot():
    

    

    
    fig3d_anim = plt.figure()
    
    mngr = plt.get_current_fig_manager()
    # to put it into the upper left corner for example:
    geom = mngr.window.geometry()
    x,y,dx,dy = geom.getRect()
    mngr.window.setGeometry(50,50,dx, dy)
    
    ax = Axes3D(fig3d_anim)
    
    return ax
    
    
def drawBlade(ax, bladeChunk, radius1, radius2, orientationAngle, percentageChange):
    #    Test visualization of blade 
#    with open("testBlade.csv", "r") as f:
#        reader = csv.reader(f)
#        csvList = list(reader)
#    
#    
#    bladeList = [i[0:2] for i in csvList]

    _, ellipseBladePoints = testEllipse(radius1,radius2, orientationAngle, [0,0])
    

    fullblade = [0,0,0]
    fullblade = numpy.array([fullblade])
    counter = 1
    for p in range(0, bladeChunk, 100):
        
        bladeArray = numpy.array(ellipseBladePoints)
        bladeArray = bladeArray.astype(float)
        bladeArray = bladeArray[0::10]
        zBlade = numpy.ones([len(bladeArray),1])*p
        bladeArray = numpy.append(bladeArray,zBlade,axis=1)
        bladeArray[:,0:2] = bladeArray[:,0:2] + bladeArray[:,0:2]*counter*percentageChange
        
        fullblade = numpy.concatenate([fullblade,bladeArray], axis=0) 
        counter +=1
        
    graph2, = ax.plot([0], [0], [0], '-r.')
    
    graph2.set_data (fullblade[:,0], fullblade[:,1])
    graph2.set_3d_properties(fullblade[:,2])    

    plt.pause(0.0001) 
    
    

def plot3D_static(ax,x,y,z, lims, plotType, title = "Movement Pattern"):
    
  
#    fig3d = plt.figure()
#
#
#    ax = Axes3D(fig3d)

    
    ax.plot(x, y, z, plotType)
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title(title)
    
    
    ax.set_xlim([-lims[0],lims[0]])
    ax.set_ylim([-lims[1],lims[1]])
    ax.set_zlim([-lims[2],lims[2]])
    

     

    
def plot3D_animation(ax,x,y,z, lim_x,lim_y, lim_z):
        
   
    
    graph, = ax.plot([x[0]], [y[0]], [z[0]], '-ro')
    title = ax.set_title("Movement Pattern")
    
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    
    newDatX = []
    newDatY = []
    newDatZ = []
    
    ax.set_xlim([-lim_x,lim_x])
    ax.set_ylim([-lim_y,lim_y])
    ax.set_zlim([-lim_z,lim_z])
    
#    ax.auto_scale_xyz([-3000,3000], [-3000,3000], [-15000,15000])
    
    for j in range(0,len(x),1):
        
        newDatX.append(x[j])
        newDatY.append(y[j])
        newDatZ.append(z[j])
        
        graph.set_data (newDatX, newDatY)
        graph.set_3d_properties(newDatZ)
        title.set_text('Movement Pattern, point={}'.format(j))
        
        
        
#        max_range = [max(newDatX),max(newDatY),max(newDatZ)]
#    
#        mid_x = (max(newDatX)+min(newDatX)) * 0.5
#        mid_y = (max(newDatY)+min(newDatY)) * 0.5
#        mid_z = (max(newDatZ)+min(newDatZ)) * 0.5
#        ax.set_xlim(mid_x - max_range[0], mid_x + max_range[0])
#        ax.set_ylim(mid_y - max_range[1], mid_y + max_range[1])
#        ax.set_zlim(mid_z - max_range[2], mid_z + max_range[2])
        
        plt.pause(0.0001)    

    
     

#x_center = 0
#y_center = 0
#
#
#r = 2000
#rotationAngle = 0
#range_start = 0
#range_stop = 190
#range_delta = 10
#
#
#
#height = 0
#height_delta = 400


# LEFT TO RIGHT pattern
#for h in range(0,3):
#    
#    for n in range(rotationAngle+ range_start,rotationAngle+ range_stop,range_delta):
#    
#        x = numpy.append(x,r*math.cos(math.radians(n)) + x_center)
#        y = numpy.append(y,r*math.sin(math.radians(n)) + y_center)
#        z = numpy.append(z,height)
#    height += 400
#    range_delta = -range_delta
#    temp = range_start + range_delta
#    range_start = range_stop + range_delta
#    range_stop = temp
    
    

#  UP TO DOWN pattern   
#for n in range(rotationAngle+ range_start,rotationAngle+ range_stop,range_delta):
#    
#    for h in range(0,3):
#        x = numpy.append(x,r*math.cos(math.radians(n)) + x_center)
#        y = numpy.append(y,r*math.sin(math.radians(n)) + y_center)
#        z = numpy.append(z,height)
#        height += height_delta
#    height_delta = -height_delta
#    height +=height_delta

# Initial Scan pattern
#heightTop = 10000
#heightBottom = 1000
#heightScanDelta = 500
#
#startingX = 2000
#startingY = 0
#for u in range(heightBottom, heightTop, heightScanDelta):
#    x = numpy.append(x, startingX)
#    y = numpy.append(y, startingY)
#    z = numpy.append(z,height)
#    height+=heightScanDelta
#
#startingAngle = int(math.degrees(math.acos((startingX - x_center)/r)))
#centerNewX = startingX - r * math.cos(math.radians(startingAngle))
#centerNewY = startingY - r * math.sin(math.radians(startingAngle))
#
#for n in range(startingAngle,190, range_delta):
#    x = numpy.append(x,r*math.cos(math.radians(n)) + centerNewX)
#    y = numpy.append(y,r*math.sin(math.radians(n)) + centerNewY)
#    z = numpy.append(z,height)
#
#endingX = x[-1]
#endingY = y[-1]
#    
#for d in range(heightBottom, heightTop, heightScanDelta):
#    x = numpy.append(x, endingX)
#    y = numpy.append(y, endingY)
#    z = numpy.append(z,height)
#    height-=heightScanDelta    
        
def scanningPattern_leftToRight(rotationAngle, circleCent, radius, startAngle, endAngle, startingHeight, scanDiffHeight, deltaScannedPoints):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]
    y_center = circleCent[1]
    
    for h in range(0,1):
        
        for n in numpy.arange(startAngle,endAngle,deltaScannedPoints):
        
            x = numpy.append(x,radius*math.cos(math.radians(n)))
            y = numpy.append(y,radius*math.sin(math.radians(n)))
            z = numpy.append(z,startingHeight)
        startingHeight += scanDiffHeight
        deltaScannedPoints = -deltaScannedPoints
        temp = startAngle + deltaScannedPoints
        startAngle = endAngle + deltaScannedPoints
        endAngle = temp
    rotationAngle = numpy.radians(rotationAngle + 180)  
    

    
    rotation_mat = numpy.matrix([[numpy.cos(rotationAngle), numpy.sin(rotationAngle)],
                      [-numpy.sin(rotationAngle), numpy.cos(rotationAngle)]])
    
    coords = numpy.vstack([x, y])
    transformed_mat = rotation_mat * coords
    
    x, y = transformed_mat.A
    
    x += x_center
    y += y_center
    
    return x,y,z


def scanningPattern_downToUp(rotationAngle,circleCent, radius, startAngle, endAngle, startingHeight, scanDiffHeight, deltaScannedPoints):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]
    y_center = circleCent[1]
    
    for n in range(rotationAngle+ startAngle,rotationAngle+ endAngle,deltaScannedPoints):
        
        for h in range(0,3):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + x_center)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + y_center)
            z = numpy.append(z,startingHeight)
            startingHeight += scanDiffHeight
        scanDiffHeight = -scanDiffHeight
        startingHeight +=scanDiffHeight
    
    
        
    return x,y,z
    

def scanningPattern_fullBladeScan(rotationDir, circleCent, radius, endAngle,  startingX, startingY, heightTop, heightBottom, heightScanDelta, deltaScannedPoints):

    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = circleCent[0]



#    heightTop = 10000
#    heightBottom = 1000
#    heightScanDelta = 500
    
#    startingX = 2000
#    startingY = 0
    
    for u in range(heightBottom, heightTop, heightScanDelta):
        x = numpy.append(x, startingX)
        y = numpy.append(y, startingY)
        z = numpy.append(z,u)
#        startingHeight+=heightScanDelta
    
    startingAngle = int(math.degrees(math.acos((startingX - x_center)/radius)))
    print(startingAngle)
    centerNewX = startingX - radius * math.cos(math.radians(startingAngle))
    centerNewY = startingY - radius * math.sin(math.radians(startingAngle))
    
    if rotationDir is 'r':
        for n in range(startingAngle,endAngle + startingAngle+10, deltaScannedPoints):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + centerNewX)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + centerNewY)
            z = numpy.append(z,u)
    elif rotationDir is 'l':
        for n in range(startingAngle + 360,endAngle + startingAngle-10, -deltaScannedPoints):
            x = numpy.append(x,radius*math.cos(math.radians(n)) + centerNewX)
            y = numpy.append(y,radius*math.sin(math.radians(n)) + centerNewY)
            z = numpy.append(z,u)
    
    endingX = x[-1]
    endingY = y[-1]
        
    for d in range( heightTop - heightScanDelta, heightBottom - heightScanDelta, -heightScanDelta):
        x = numpy.append(x, endingX)
        y = numpy.append(y, endingY)
        z = numpy.append(z,d)
#        startingHeight-=heightScanDelta   
    
       

    return x,y,z
    
    
def scanningPattern_startPos(centerXY, radius, startingAngle, rotationAngle):
    x = centerXY[0] + radius*math.cos(math.radians(startingAngle))
    y = centerXY[1] + radius*math.sin(math.radians(startingAngle))
    
    x,y = scanningPattern_rotateAtAngle(x,y, rotationAngle)
    
    return x,y
    
    
def scanningPattern_rotateAtAngle(x,y, rotationAngle):
    
    rotationAngle = numpy.radians(rotationAngle)  
    
    rotation_mat = numpy.matrix([[numpy.cos(rotationAngle), numpy.sin(rotationAngle)],
                      [-numpy.sin(rotationAngle), numpy.cos(rotationAngle)]])
    
    coords = numpy.vstack([x, y])
    transformed_mat = rotation_mat * coords
    
    x, y = transformed_mat.A
    
    return x, y
    
    
def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    theta = math.radians(theta)
    axis = numpy.asarray(axis)
    axis = axis/math.sqrt(numpy.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return numpy.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])
    
    
    
def scanningPattern_goHorizontal(centerXY, radius, height, rotationAngle, startAngle, endAngle, deltaScannedPoints):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    x_center = centerXY[0]
    y_center = centerXY[1]



    if startAngle > endAngle:
        deltaScannedPoints = -deltaScannedPoints

    endAngle+= deltaScannedPoints

    for n in numpy.arange(startAngle,endAngle,deltaScannedPoints):
        x = numpy.append(x,x_center + radius*math.cos(math.radians(n)))
        y = numpy.append(y,y_center + radius*math.sin(math.radians(n)))
        z = numpy.append(z,height)
        
        
    x,y = scanningPattern_rotateAtAngle(x - x_center,y - y_center, rotationAngle)
    
    x = x + x_center
    y = y + y_center
        
    return x, y, z
    
    
    
    
def scanningPattern_goVertically(startingX, startingY,startingZ, heightToGo, heightScanDelta):
    x_list = []
    y_list = []
    z_list = []
    x = numpy.array(x_list)
    y = numpy.array(y_list)
    z = numpy.array(z_list)
    
    startingZ = int(startingZ)
    
    if heightToGo < 0:
        heightScanDelta = - heightScanDelta
    
    for u in range(startingZ, startingZ + heightToGo, heightScanDelta):
        x = numpy.append(x, startingX)
        y = numpy.append(y, startingY)
        z = numpy.append(z,u)
        
    return x,y,z
    

def appendPath(x,y,z, pathTempX, pathTempY, pathTempZ):
        
    pathTempX = numpy.append(pathTempX, x)
    pathTempY = numpy.append(pathTempY, y)
    pathTempZ = numpy.append(pathTempZ, z)
    
    return pathTempX, pathTempY, pathTempZ

            
if __name__ == '__main__': 
    
    
    pathX = numpy.array([])
    pathY = numpy.array([])
    pathZ = numpy.array([])
    
    positionBeforeScan = [-2000, 2000, 300]
    
#    pathX, pathY, pathZ = appendPath(positionBeforeScan[0], positionBeforeScan[1],positionBeforeScan[2], pathX,pathY, pathZ )
    

    
    
    
    startingHeight = 500
    
    bladeLE = [1000,380]
    
    rotationAngle = 180 
    
    rotationRadius = 2000
    
    heightChange = 1500
    
    
    deltaPath_horizontal = 10
    deltaPath_vertical = 50
    

    startingAngle = 0
    endAngle = 180
    
    #    Check which side is the proper side
    
#    startingAngle_temp = 0
#    x_0,y_0 = scanningPattern_startPos(bladeLE,rotationRadius, startingAngle_temp, rotationAngle)
#    
#    startingAngle_temp = 180
#    x_180,y_180 = scanningPattern_startPos(bladeLE,rotationRadius, startingAngle_temp, rotationAngle)
#    
#    
#    
#    dist_posBefore_angle0 = math.sqrt( (x_0 - positionBeforeScan[0])**2 + (y_0 - positionBeforeScan[1])**2 )
#    dist_posBefore_angle180 = math.sqrt( (x_180 - positionBeforeScan[0])**2 + (y_180 - positionBeforeScan[1])**2 )
#    
#    dist_posBefore_bladeLE = math.sqrt( (bladeLE[0] - positionBeforeScan[0])**2 + (bladeLE[1] - positionBeforeScan[1])**2 )
#    
#    
#    if dist_posBefore_angle0 < dist_posBefore_bladeLE:
#        startingAngle = 0
#        
#        
#    elif dist_posBefore_angle180 < dist_posBefore_bladeLE:
#        startingAngle = 180
#        endAngle = 0
    
    
    
    
    
#    Scan pattern formation
#    x,y = scanningPattern_startPos(bladeLE,rotationRadius, startingAngle, rotationAngle)
    
#    pathX, pathY, pathZ = appendPath(x, y,startingHeight, pathX,pathY, pathZ )
    
#    x,y,z = scanningPattern_goVertically(pathX[-1], pathY[-1], startingHeight, heightChange, deltaPath_vertical)
    
#    pathX, pathY, pathZ = appendPath(x,y,z, pathX,pathY, pathZ )
    
    x,y,z = scanningPattern_goHorizontal(bladeLE, rotationRadius, startingHeight, rotationAngle, startingAngle, endAngle, deltaPath_horizontal)
    
    pathX, pathY, pathZ = appendPath(x,y,z, pathX,pathY, pathZ )
    
#    x,y,z = scanningPattern_goVertically(pathX[-1], pathY[-1], pathZ[-1], -heightChange, deltaPath_vertical)
    
#    pathX, pathY, pathZ = appendPath(x,y,z, pathX,pathY, pathZ )
    
#    x,y,z = scanningPattern_leftToRight(50, [0,0], 1500, 0, 180, 0, 800, 10)
    
#    x,y,z = scanningPattern_leftToRight(180, [0,0], 2000, 0, 190, 0, 400, 10)
#    plot3D_animation(x,y,z,2000,2000,2000)
#    x,y,z = scanningPattern_fullBladeScan('l',[0,0], 2000, 180,  2000, 0, 10000, 1000, 500, 10)
    
#    drawBlade(ax, 3000, 110, 660, 0, 0)
    
#    for j in range(1,10):
        
#    x,y,z = scanningPattern_downToUp(180, [0,0], 2000, 180, -10, 0, 400, -10)
    ax = initializeFigurePlot()
    plot3D_animation(ax,pathX,pathY,pathZ,2000,2000,3000)
#    plot3D_static(x,y,z)
#    plot3D_animation(x,y,z,2000,2000,10000)
    

