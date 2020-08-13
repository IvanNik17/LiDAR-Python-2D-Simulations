# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:48:07 2018

@author: ivan


Helper functions for the LiDAR_animate and Lidar_visualizer. only the pdist from scipy is used, but this can be easily substituted with
numpy implementation. Some of the functions are not fully vectorised, but in their final implementation will be. Some of the functions
are still in development and current versions are still in testing. List functions:
    -calculateMeans - a blanket function for calculating the mean angle and mean distance from the Lidar readings, calls circularMean
    -circle2cart_drone - function for calculating the carthesian coordinates of the lidar
    -circle2cart_points - function for calculating the reprojected blade point coordinates
    -intersectionLineCurve - function that calculates the intersection between a curve an a line made from two points. If it's a closed curve
    it calculates the distances from the start point to the intersections and takes the closest one
    -loadBlade - function to load one of the provided NACA4 blades contained as a X,Y txt file
    -rotateTranslateBlade - function to rotate and translate loaded blade to demonstrate different orientations and positions
    -droneOrientation - change the initial orientation of the drone
    -lidarRotation_change - change the rotation start angle of the LiDAR
    -calculateDronePos - change polar to carthesian coordinates of the LiDAR and reproject the LiDAR's readings.
    
    
    

"""

import math
import numpy as np
from scipy.spatial.distance import pdist



from os import walk, getcwd


def calculateMeans(angleDistList):
    # It calculates the means of the distance and angle, it introduces weights, so if needed a weighted average an be easily implemented   
    dists = angleDistList[:,1]
    weight = dists/sum(dists)
    
    meanAngleCalc = circularMean(angleDistList[:,0],weight)    

    meanDistCalc = np.average(angleDistList[:,1],weights= weight)

    return meanAngleCalc, meanDistCalc



def circularMean(angles,weight):
    
    #For calculating the circular mean, the going from 0 to 360 needs to be observed, so the cos and sin angles are calculated
    # as well as the mean sin and cos, which are then used to determine if additional degrees need to be added
    sinAngles = np.sin(np.radians(angles))
    cosAngles = np.cos(np.radians(angles))
    

    meanSin = np.dot(weight,sinAngles)
    meanCos = np.dot(weight,cosAngles)

    
    if (meanSin > 0 and meanCos > 0):
        meanAngle = math.atan(meanSin/meanCos)
        
    elif (meanCos < 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(180)
    elif (meanSin <0 and meanCos> 0):
        meanAngle = math.atan(meanSin/meanCos) + math.radians(360)
    
    return math.degrees(meanAngle)

# The difference between the two circle2cart functions is the direction of projection of  the data. The points program is currently not
#properly vectoried and it can take an additional angle to rotate it which is not presently used.    
def circle2cart_drone(center,angle, distance):
    xC = center[0] - distance * math.sin(math.radians(angle))
    yC = center[1] - distance * math.cos(math.radians(angle))
    return [xC,yC]

def circle2cart_lidarLine(center,angle, distance):
    xC = center[0] + distance * math.sin(math.radians(angle))
    yC = center[1] + distance * math.cos(math.radians(angle))
    return [xC,yC]




def circle2cart_points(center,angle,distance):
    
    
    xC = center[0] + distance * np.sin(np.radians(angle))
    yC = center[1] + distance * np.cos(np.radians(angle))
    
    xC = np.expand_dims(xC, axis=1)
    yC = np.expand_dims(yC, axis=1)
    
    pointCoords = np.concatenate((xC, yC), axis=1) 
    
    return pointCoords
    
    
def circle2cart_points_op(center,angle,distance):
    
    
    xC = center[0] - distance * np.sin(np.radians(angle))
    yC = center[1] - distance * np.cos(np.radians(angle))
    
    xC = np.expand_dims(xC, axis=1)
    yC = np.expand_dims(yC, axis=1)
    
    pointCoords = np.concatenate((xC, yC), axis=1) 
    
    return pointCoords
    
    
def rotatePoints(center, pointsX, pointsY, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = center
    angle= math.radians(angle)

    qx = ox + math.cos(angle) * (pointsX - ox) - math.sin(angle) * (pointsY - oy)
    qy = oy + math.sin(angle) * (pointsX - ox) + math.cos(angle) * (pointsY - oy)
    
    
    qx = np.expand_dims(qx, axis=1)
    qy = np.expand_dims(qy, axis=1)
    
    pointCoords = np.concatenate((qx, qy), axis=1)
    
    return pointCoords
    
    
def movePoints(points, angle, distance):
    
    points[:,0] =distance * np.sin(np.radians(angle))
    points[:,1] = - distance * np.cos(np.radians(angle))
    
#    xC = np.expand_dims(xC, axis=1)
#    yC = np.expand_dims(yC, axis=1)
#    
#    pointCoords = np.concatenate((xC, yC), axis=1) 
    
    return points
  
    
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


# Loads blade from NACA4 folder, checks if the blade exists in the folder, scales it, repositions it to the center of the coordinate system
def loadBlade(bladeName, scale):
    stringpath = getcwd() + "\\NACA4"
    
    whichBlade = ""
    f = []
    for (dirpath, dirnames, filenames) in walk(stringpath):
        f.extend(filenames)
        
        break
    
    for i in range(0, len(filenames)):
        if filenames[i] == bladeName + '.txt':
            whichBlade = filenames[i]
    
    if len(whichBlade) == 0:
        return -1
        
    
    airfoilDir = dirpath + "\\" +whichBlade
    airfoil = np.loadtxt(airfoilDir)
    bladeArray = np.array(airfoil)
    
    bladeArray[:,0] -= np.mean(bladeArray[:,0])
    
    bladeArray = bladeArray * scale

    return bladeArray
    
    

def calculateDronePos( distListL, angleListL):
       
#   List to Numpy array                        
    angleArray = np.array([angleListL])
    
    distArray = np.array([distListL])
    
#   Concatenate the two arrays 
    angleDistance = np.concatenate([angleArray,distArray],axis =0).T

 
    meanAngle,meanDist = calculateMeans(angleDistance)  # calculate the mean angle and distance from LiDAR datta
    dronePos = circle2cart_drone([0,0],meanAngle, meanDist) # transform drone position from the mean angle and distance to a 0,0 center
    bladePointsPos = circle2cart_points(dronePos,angleDistance[:,0], angleDistance[:,1]) # reproject the captured data from the newly calculated drone position  


    return np.array(dronePos), bladePointsPos, meanDist, meanAngle
        
        

        
#Check if two flats are equal to a certain value
def eq( a, b, eps=0.0001 ):
    return abs(a - b) <= eps


#   Rotate the blade points and translate them
def rotateTranslateBlade(bladeArray, angle, translate):
        
    angle = -angle
    qx = 0 + math.cos(math.radians(angle)) * (bladeArray[:,0] - translate[0]) - math.sin(math.radians(angle)) * (bladeArray[:,1] - translate[0])
    qy = 0 + math.sin(math.radians(angle)) * (bladeArray[:,0] - translate[1]) + math.cos(math.radians(angle)) * (bladeArray[:,1] - translate[1])
    
    
    
    newCoord = np.array([qx,qy]).T

    return newCoord
    
#   change the initial orientation of the LiDAR/drone    
def droneOrientation(x_temp,y_temp, angle):
#    size of the line representing the orientation
    arrowSize = 500 
    
    distToEndX = arrowSize*math.sin(math.radians(angle))
    distToEndY = arrowSize*math.cos(math.radians(angle))
   

    positionFrontX = x_temp + distToEndX
    positionFrontY = y_temp + distToEndY
    
    return [positionFrontX,positionFrontY]


def lidarRotation_change(angle, offset):
    return angle + offset
    
    
def calculateMinMax(pointCloud2D):
    # get all distances between the blade points
    distAll = np.linalg.norm(pointCloud2D - pointCloud2D[:,None], axis=-1)
    
    # remove duplicates
    distAll_noDup = np.triu(distAll)
    # get the maximum distances, minimum distances, as well as the points which have the minimum and maximum distances
    minDist = np.min(distAll_noDup[np.nonzero(distAll_noDup)])
    maxDist = np.max(distAll_noDup[np.nonzero(distAll_noDup)])
    minDist_index = np.where(distAll_noDup==minDist)
    minDist_points = np.array([pointCloud2D[minDist_index[0][0],:], pointCloud2D[minDist_index[1][0],:]])
    maxDist_index = np.where(distAll_noDup==maxDist)
    
#    Remove if you want both points visible
    
#    maxDist_points = np.array([pointCloud2D[maxDist_index[0][0],:], pointCloud2D[maxDist_index[1][0],:]])
    
    twoMaxIndex = [maxDist_index[0][0], maxDist_index[1][0]]
    biggerInd = np.max(twoMaxIndex)
    maxDist_points = np.array([pointCloud2D[biggerInd,:]])

    return maxDist, maxDist_points, twoMaxIndex
    
def calculateAnglesPCA(bladePointsPos):
    
    # calculate covariance matrix of the detected blade points

    cov_mat = np.cov(bladePointsPos.T)
    # calculate the eigen values and eigen vectors from the covariance matrix
    eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
    
    # find the largest eigenvalue and the eigenvector that coresponds to it
    maxIndEigenval = np.argmax(eig_val_cov)
    evec1 = eig_vec_cov[:,maxIndEigenval]
    
    # calculate the angle from the eigenvector
    angleOffsetMeasured = math.degrees(np.arctan2( evec1[0],evec1[1]   )) 
    
    # currently the algorithm does not detect angles larger than 180 degrees, and it's up to the user to specify if it's suction of pressure side
    if angleOffsetMeasured < 0:
        angleOffsetMeasured =  angleOffsetMeasured + 180
        
    return angleOffsetMeasured