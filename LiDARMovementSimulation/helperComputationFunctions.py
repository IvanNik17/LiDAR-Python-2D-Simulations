# -*- coding: utf-8 -*-
"""
Created on Tue Sep 19 11:32:19 2017

@author: ivan
"""

import math
import numpy as np
from scipy.spatial.distance import pdist

def testEllipse(distance1,distance2, orientation, center):
#    ellipseInfo = collections.namedtuple('ellipseInfo', ['radiusAnglesEllipse', 'ellipsePointsPos'])
    numberOfPoints = 360
    centerX = center[0]
    centerY = center[1]
    orientation = -orientation
    theta = np.linspace(0, 2*math.pi, numberOfPoints)
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
#    theta_deg = numpy.degrees(theta)
    
    
    radiusAngles = np.column_stack((radiusDist,degrees))
    ellipsePos = np.column_stack((xx2,yy2))

    return radiusAngles, ellipsePos  




#def testEllipse(distance1,distance2, orientation, center):
##    ellipseInfo = collections.namedtuple('ellipseInfo', ['radiusAnglesEllipse', 'ellipsePointsPos'])
#    numberOfPoints = 360
#    centerX = center[0]
#    centerY = center[1]
#    orientation = -orientation
#    theta = np.linspace(0, 2*math.pi, numberOfPoints)
#    orientation=orientation*math.pi/180
#    
#    radiusDist = []
#    xx2 = []
#    yy2 = []
#
#    for i in range(0,numberOfPoints):
#        xx = -(distance1/2) * math.sin(theta[i]) + centerX
#        yy = -(distance2/2) * math.cos(theta[i]) + centerY
#    
#        xx2_temp = (xx-centerX)*math.cos(orientation) - (yy-centerY)*math.sin(orientation) + centerX
#        yy2_temp = (xx-centerX)*math.sin(orientation) + (yy-centerY)*math.cos(orientation) + centerY
#
#        xx2.append(xx2_temp)
#        yy2.append(yy2_temp)
#    
#        radiusDist.append(math.sqrt(xx2_temp**2 + yy2_temp**2))
#
#
#    degrees = range(0,numberOfPoints)
#    
#    radiusAngles = np.column_stack((radiusDist,degrees))
#    ellipsePos = np.column_stack((xx2,yy2))
#
#    return radiusAngles, ellipsePos
    
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(x, y)
    return(rho, phi)
    
def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return idx      

    
def circularMean(angles,weight):
    sinAngles = []
    cosAngles = []
    
    for i in range(0,len(weight)):
        sinAngles.append( math.sin(math.radians(angles[i])))
        cosAngles.append( math.cos(math.radians(angles[i])))
    
    meanSin = np.dot(weight,sinAngles)
    meanCos = np.dot(weight,cosAngles)
    
    if (meanSin > 0 and meanCos > 0):
        meanAngle = math.atan(meanSin/meanCos)
        
    elif (meanCos < 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(180)
    elif (meanSin <0 and meanCos> 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(360)
    
    return math.degrees(meanAngle)
    
def circle2cart_drone(center,angle, distance):
    xC = center[0] - distance * np.sin(np.radians(angle))
    yC = center[1] - distance * np.cos(np.radians(angle))
    return [xC,yC]

def circle2cart_points(circleCenter,measurement,addDegree):

    xC = []
    yC = []
    for i in range(0,len(measurement)):
        xC.append( circleCenter[0] + measurement[i,1] * math.sin(math.radians(measurement[i,0] + addDegree)) )
        yC.append( circleCenter[1] + measurement[i,1] * math.cos(math.radians(measurement[i,0] + addDegree)) )
    
    return np.column_stack((xC,yC))
    
    
def intersectionLineCurve(lineFirst, lineSecond, curvePoints, searchRadius):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = np.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = np.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    
    if cross_points.size is 0:
        return [-1,-1], -1
    else:
        
        
#        print(a)
        
        distToLidar_1 = np.array([ lineSecond,  cross_points[0,:] ])
        distToLidar_1 = pdist(distToLidar_1,'euclidean')
        
        distToLidar_2 = np.array([ lineSecond,  cross_points[1,:] ])
        distToLidar_2 = pdist(distToLidar_2,'euclidean')
        
        
        
        distToLidarReal = 0
        if (distToLidar_1 > distToLidar_2):
            outputInters = cross_points[1,:]
            distToLidarReal = distToLidar_2
        else:
            outputInters = cross_points[0,:]
            distToLidarReal = distToLidar_1

        
        distEndToCross = np.array([ lineFirst,  outputInters ])
        distEndToCross = pdist(distEndToCross,'euclidean')
            
        if (distEndToCross < searchRadius and distToLidarReal < searchRadius):
            return outputInters, distToLidarReal
        else:
            return [-1,-1], -1
        
        

def intersectionLineCurveOLD(lineFirst, lineSecond, curvePoints):

    b = (lineSecond[1] - lineFirst[1]) / (lineSecond[0] - lineFirst[0]) # gradient
    a = lineFirst[1] - b * lineFirst[0] # intercept
    B = (a + curvePoints[:,0] * b) - curvePoints[:,1] # distance of y value from line
    ix = np.where(B[1:] * B[:-1] < 0)[0] # index of points where the next point is on the other side of the line
    
    
    d_ratio = B[ix] / (B[ix] - B[ix + 1]) # similar triangles work out crossing points
    cross_points = np.zeros((len(ix), 2)) # empty array for crossing points
    cross_points[:,0] = curvePoints[ix,0] + d_ratio * (curvePoints[ix+1,0] - curvePoints[ix,0]) # x crossings
    cross_points[:,1] = curvePoints[ix,1] + d_ratio * (curvePoints[ix+1,1] - curvePoints[ix,1]) # y crossings
    
    distToLidar_1 = np.array([ lineSecond,  cross_points[0,:] ])
    distToLidar_1 = pdist(distToLidar_1,'euclidean')
    
    distToLidar_2 = np.array([ lineSecond,  cross_points[1,:] ])
    distToLidar_2 = pdist(distToLidar_2,'euclidean')
    
    if (distToLidar_1 > distToLidar_2):
        outputInters = cross_points[1,:]
    else:
        outputInters = cross_points[0,:]
    
#    print(cross_points)
    return outputInters
        

def eq( a, b, eps=0.0001 ):
    return abs(a - b) <= eps