# -*- coding: utf-8 -*-
"""
Created on Tue Jul  4 11:39:39 2017

@author: ivan nikolov
"""
import csv
import numpy
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist
import math

import time
import timeit

from testLineFilter import sunFilter_testForWeights, sunFilterV2


with open("testBlade.csv", "r") as f:
    reader = csv.reader(f)
    csvList = list(reader)
    
    
bladeList = [i[0:2] for i in csvList]

bladeArray = numpy.array(bladeList)
bladeArray = bladeArray.astype(float)

isSelected = False

def testEllipse(distance1,distance2, orientation, center):
#    ellipseInfo = collections.namedtuple('ellipseInfo', ['radiusAnglesEllipse', 'ellipsePointsPos'])
    numberOfPoints = 360
    centerX = center[0]
    centerY = center[1]
    orientation = -orientation
    theta = numpy.linspace(0, 2*math.pi, numberOfPoints)
    orientation=orientation*math.pi/180
    
    radiusDist = []
    xx2 = []
    yy2 = []

    for i in range(0,numberOfPoints):
        xx = -(distance1/2) * math.sin(theta[i]) + centerX
        yy = -(distance2/2) * math.cos(theta[i]) + centerY
    
        xx2_temp = (xx-centerX)*math.cos(orientation) - (yy-centerY)*math.sin(orientation) + centerX
        yy2_temp = (xx-centerX)*math.sin(orientation) + (yy-centerY)*math.cos(orientation) + centerY

        xx2.append(xx2_temp)
        yy2.append(yy2_temp)
    
        radiusDist.append(math.sqrt(xx2_temp**2 + yy2_temp**2))


    degrees = range(0,numberOfPoints)
    
    radiusAngles = numpy.column_stack((radiusDist,degrees))
    ellipsePos = numpy.column_stack((xx2,yy2))

    return radiusAngles, ellipsePos

def intersectionLineCurve(lineFirst, lineSecond, curvePoints, searchRadius):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = numpy.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = numpy.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    
    if cross_points.size is 0:
        return [-1,-1], -1
    else:
        
        
#        print(a)
        
        distToLidar_1 = numpy.array([ lineSecond,  cross_points[0,:] ])
        distToLidar_1 = pdist(distToLidar_1,'euclidean')
        
        distToLidar_2 = numpy.array([ lineSecond,  cross_points[1,:] ])
        distToLidar_2 = pdist(distToLidar_2,'euclidean')
        
        
        
        distToLidarReal = 0
        if (distToLidar_1 > distToLidar_2):
            outputInters = cross_points[1,:]
            distToLidarReal = distToLidar_2
        else:
            outputInters = cross_points[0,:]
            distToLidarReal = distToLidar_1

        
        distEndToCross = numpy.array([ lineFirst,  outputInters ])
        distEndToCross = pdist(distEndToCross,'euclidean')
            
        if (distEndToCross < searchRadius and distToLidarReal < searchRadius):
            return outputInters, distToLidarReal
        else:
            return [-1,-1], -1
        
        

def intersectionLineCurveOLD(lineFirst, lineSecond, curvePoints):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = numpy.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = numpy.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    
    distToLidar_1 = numpy.array([ lineSecond,  cross_points[0,:] ])
    distToLidar_1 = pdist(distToLidar_1,'euclidean')
    
    distToLidar_2 = numpy.array([ lineSecond,  cross_points[1,:] ])
    distToLidar_2 = pdist(distToLidar_2,'euclidean')
    
    if (distToLidar_1 > distToLidar_2):
        outputInters = cross_points[1,:]
    else:
        outputInters = cross_points[0,:]
    
#    print(cross_points)
    return outputInters
        
    #    print(cross_points)
def circularMean(angles,weight):
    
    sinAngles = []
    cosAngles = []
    
    for i in range(0,len(weight)):
        sinAngles.append( math.sin(math.radians(angles[i])))
        cosAngles.append( math.cos(math.radians(angles[i])))
    
#    for i in range(0,len(weight)):
#    meanSin = weight@sinAngles
#    meanCos = weight@cosAngles
    meanSin = numpy.dot(weight,sinAngles)
    
    meanCos = numpy.dot(weight,cosAngles)
    
    if (meanSin > 0 and meanCos > 0):
        meanAngle = math.atan(meanSin/meanCos)
        
    elif (meanCos < 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(180)
    elif (meanSin <0 and meanCos> 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(360)
    
    return math.degrees(meanAngle)        

    
def circle2cart_drone(center,angle, distance):
   
    xC = center[0] - distance * math.sin(math.radians(angle))
    yC = center[1] - distance * math.cos(math.radians(angle))

    return [xC,yC]




def circle2cart_points(circleCenter,measurement,addDegree):

    xC = []
    yC = []
    for i in range(0,len(measurement)):
        xC.append( circleCenter[0] + measurement[i,1] * math.sin(math.radians(measurement[i,0] + addDegree)) )
        yC.append( circleCenter[1] + measurement[i,1] * math.cos(math.radians(measurement[i,0] + addDegree)) )
    
    return numpy.column_stack((xC,yC))

def eq( a, b, eps=0.0001 ):
    return abs(a - b) <= eps


def pointForPloting(angle, length):
    endy = length * math.sin(math.radians(angle))
    endx = length * math.cos(math.radians(angle))
    
    return endx, endy
    
    
def recursiveSearchCenter(averagePointPos, bladePointsPos, recursiveCounter):
    
    distFromCenter = (bladePointsPos-averagePointPos)**2
    distFromCenter = distFromCenter.sum(axis=-1)
    distFromCenter = numpy.sqrt(distFromCenter)
    
    occurancesBool = distFromCenter < numpy.average(distFromCenter)
    occurancesCloser = distFromCenter[occurancesBool]
    try:
        averageIndex = 9999
        
        occurances_distFromStart = int(numpy.argmin(occurancesCloser)) + 1
        occurances_distToEnd = len(occurancesCloser) - int(numpy.argmin(occurancesCloser))
    
    
        
        if occurances_distFromStart > occurances_distToEnd and recursiveCounter < 10:
            averagePointPos = bladePointsPos[numpy.where(distFromCenter == occurancesCloser[numpy.argmin(occurancesCloser)+1])]
            recursiveCounter += 1
            return recursiveSearchCenter(averagePointPos, bladePointsPos, recursiveCounter)
        elif occurances_distFromStart < occurances_distToEnd and recursiveCounter < 10:
            averagePointPos = bladePointsPos[numpy.where(distFromCenter == occurancesCloser[numpy.argmin(occurancesCloser)-1])]
            recursiveCounter += 1
            return recursiveSearchCenter(averagePointPos, bladePointsPos, recursiveCounter)
        
        elif occurances_distFromStart == occurances_distToEnd and recursiveCounter == 0:
            averageIndex = -1
        else:
            averageIndex = numpy.where(bladePointsPos == averagePointPos)
            averageIndex  = int(averageIndex[0][0])
    except (ValueError, IndexError):
        averageIndex = -1
        
    return averageIndex, averagePointPos
    

fig = plt.figure()

ax = fig.add_subplot(111)
plt.plot(bladeArray[:,0],bladeArray[:,1]) 
plt.axis([-1500, 1500, -1500, 1500])

searchRad = 900


#Click VISUALIZE

angle = 0



plotHandle, = plt.plot([0],[0], 'go')

plotHandle2, = plt.plot([0,0],[0,0])

plotHandle3, = plt.plot([0],[0], 'ro')

plotHandle_coord, = plt.plot([0,0],[0,0])

plotHandle_calculated, = plt.plot([0],[0], 'ko')
#plt.axes().set_aspect('equal', 'datalim')

plotHandle_averages_old, = plt.plot([0],[0], 'rx')
plotHandle_averages_new, = plt.plot([0],[0], 'bx')

coords = []

detectedPoints = numpy.array([[0,0]])

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
#    print (ix, iy)

    global coords, isSelected
    coords = []
    coords.append((ix, iy))
    
    isSelected = True
    

        


#    return coords
cid = fig.canvas.mpl_connect('button_press_event', onclick)
start_time = 0
distList = []
angleList = []
try:
    while True:
        plt.pause(0.0000000001)
        
        if isSelected is True:
            plotHandle3.set_data(coords[0][0],coords[0][1])
#            
            detectedPoints = numpy.array([[0,0]])
            isSelected = False
            plotHandle_coord.set_data([0,0],[0,0])
        angle += 2    
        if len(coords) > 0:
            if angle == 2:
                start_time = timeit.default_timer()
                
            
            coordinates = [coords[0][0],coords[0][1]]
            
            circlePos = circle2cart_drone(numpy.array(coordinates).T,angle, searchRad)
            plotHandle2.set_data([circlePos[0],coordinates[0]],[circlePos[1], coordinates[1]])
            
            intersectionPoint, dist = intersectionLineCurve( circlePos, numpy.array(coordinates), bladeArray,searchRad)
            
            if intersectionPoint[0] != -1 and not numpy.any(eq(detectedPoints[:, 0], intersectionPoint[0])) and not numpy.any(eq(detectedPoints[:, 1], intersectionPoint[1])):

                detectedPoints = numpy.concatenate((detectedPoints,[intersectionPoint]))
                
                angleList.append(angle)
                distList.append(dist[0])
                plotHandle.set_data(detectedPoints[:,0],detectedPoints[:,1])
            
            
            if angle > 360:
                
                if len(distList) is not 0:
                    
                    meanDist = sum(distList)/ float(len(distList))
                    
                    
                    angleArray = numpy.array([angleList]) + 180
                    distArray = numpy.array([distList])

                    
                    angleDistance = numpy.concatenate([angleArray,distArray],axis =0).T                             
                                                     
                    weight = numpy.ones(len(angleDistance))/len(angleDistance)
                    angleArrayForMen = numpy.array(angleList)+ 180
       
                    meanAngle = circularMean(angleArrayForMen,weight)                              
                               
                    dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
#                    plotHandle_calculated.set_data(dronePos[0], dronePos[1])        
                    bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
                    
                    
                    averagePointPos=[sum(bladePointsPos[:,0])/len(bladePointsPos[:,0]),sum(bladePointsPos[:,1])/len(bladePointsPos[:,1])]
                    
                    indexAverage, averagePointNew =  recursiveSearchCenter(averagePointPos, bladePointsPos, 0)
                    
                    
                    distCentToPoint = numpy.array([ [0,0],  [averagePointPos[0],averagePointPos[1]] ])
                    distAveragePointToZero = pdist(distCentToPoint,'euclidean');
                    
                    intersectPointOnCurve = intersectionLineCurveOLD([0,0], dronePos, bladeArray)
                    correctionDist = math.sqrt(intersectPointOnCurve[0]**2 + intersectPointOnCurve[1]**2)

                    newCenterPos = circle2cart_drone([0,0],meanAngle, correctionDist -distAveragePointToZero)
                    
                    
                    newDronePos = circle2cart_drone(newCenterPos,meanAngle, meanDist)
                    
                    
                    plotHandle_calculated.set_data(newDronePos[0], newDronePos[1])
                    

                    
                    
                    angleBetween = math.atan2(-coordinates[1] + bladeArray[0,1], -coordinates[0] + bladeArray[0,0])
                    angleBetween = math.degrees(angleBetween)

                    plotHandle_coord.set_data([coordinates[0], bladeArray[1022,0]],[coordinates[1], bladeArray[1022,1]])
                    
                angleList = []    
                distList = []
                angle = 0
                detectedPoints = numpy.delete(detectedPoints,0,0)
                elapsed = timeit.default_timer() - start_time

                
                detectedPoints = numpy.array([[0,0]])
            
#        time.sleep(0.1)
except KeyboardInterrupt:
    fig.canvas.mpl_disconnect(cid)
    print("Ended")
        
    