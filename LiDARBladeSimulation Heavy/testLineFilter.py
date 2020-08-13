# -*- coding: utf-8 -*-
"""
Created on Thu Jun 29 12:41:41 2017

@author: ivan
"""

import numpy

def difference_matrix(a):
    x = numpy.reshape(a, (len(a), 1))
    return x - x.transpose()
    

    
    
#angles =numpy.array( [[275.0781,
# 275.4063,
# 275.7188,
# 276.0938,
# 282.7813,
# 283.9063,
# 285.0000,
# 286.0625,
# 287.2031,
# 288.3125,
# 289.3906,
# 290.4375,
# 291.5469,
# 292.6563,
# 313.8438,
# 314.2969,
# 314.6719]])
#
#
#distances = numpy.array([[
#    508.2,
#    676.0,
#    1036.0,
#    2188.8,
#    1793.3,
#    1793.8,
#    1793.0,
#    1795.0,
#    1795.0,
#    1803.8,
#    1815.5,
#    1830.2,
#    1869.0,
#    1885.0,
#    645.8,
#    971.5,
#    1842.2]])

def sunFilterV2(angles,distances, distThreshA, distThreshD):
    
    x = angles.T
    
    sinAngles = numpy.sin(numpy.radians(x))
    cosAngles = numpy.cos(numpy.radians(x)) 
#    NEEDS TO BE CHECKED
#    distances = (a-b)**2
#distances = distances.sum(axis=-1)
#distances = np.sqrt(distances)
    
    diffMatSin = difference_matrix(sinAngles.T)
    diffMatSinAbs = numpy.absolute(diffMatSin)
    
    diffMatCos = difference_matrix(cosAngles.T)
    diffMatCosAbs = numpy.absolute(diffMatCos)
    
    diffMatSinAbs[numpy.logical_and(diffMatSinAbs > distThreshA, diffMatSinAbs != 0)] = -1
    diffMatCosAbs[numpy.logical_and(diffMatCosAbs > distThreshA, diffMatCosAbs != 0)] = -1
                
                
    occurancesInSin = (diffMatSinAbs == -1).sum(axis=1)
    occurancesInSin[occurancesInSin > numpy.average(occurancesInSin)] = -1
                     
    occurancesInCos = (diffMatCosAbs == -1).sum(axis=1)
    occurancesInCos[occurancesInCos > numpy.average(occurancesInCos)] = -1
                    
    diffMatDist = difference_matrix(distances.T)
    diffMatDistAbs = numpy.absolute(diffMatDist)       
    diffMatDistAbs[numpy.logical_and(diffMatDistAbs > distThreshD, diffMatDistAbs != 0)] = -1
    occurancesInDist = (diffMatDistAbs == -1).sum(axis=1)
    occurancesInDist[occurancesInDist > numpy.average(occurancesInDist)] = -1
                    
                  
    compareMat = numpy.zeros([numpy.size(occurancesInSin)])
    compareMat[numpy.logical_and(occurancesInSin == -1, occurancesInCos == -1, occurancesInDist == -1)] = 1
    
    problemIndices = numpy.where(compareMat==1)
    
    return problemIndices
    

def sunFilter_testForWeights(angles,distances, distThreshA, distThreshD):
    
    x = angles.T
    
    sinAngles = numpy.sin(numpy.radians(x))
    cosAngles = numpy.cos(numpy.radians(x)) 
    
    
    
    diffMatSin = difference_matrix(sinAngles.T)
    diffMatSinAbs =  numpy.absolute(diffMatSin)
    
    diffMatCos = difference_matrix(cosAngles.T)
    diffMatCosAbs = numpy.absolute(diffMatCos)
    
    diffMatSinAbs[numpy.logical_and(diffMatSinAbs > distThreshA, diffMatSinAbs != 0)] = -1
    diffMatCosAbs[numpy.logical_and(diffMatCosAbs > distThreshA, diffMatCosAbs != 0)] = -1
                
                
    occurancesInSin = (diffMatSinAbs == -1).sum(axis=1)
    occurancesInSin[occurancesInSin > numpy.average(occurancesInSin)] = -1
                     
    occurancesInCos = (diffMatCosAbs == -1).sum(axis=1)
    occurancesInCos[occurancesInCos > numpy.average(occurancesInCos)] = -1
                    
    diffMatDist = difference_matrix(distances.T)
    diffMatDistAbs = numpy.absolute(diffMatDist)       
    diffMatDistAbs[numpy.logical_and(diffMatDistAbs > distThreshD, diffMatDistAbs != 0)] = -1
    occurancesInDist = (diffMatDistAbs == -1).sum(axis=1)
    occurancesInDist[occurancesInDist > numpy.average(occurancesInDist)] = -1
                    
                  
    compareMat = numpy.zeros([numpy.size(occurancesInSin)])
    compareMat[numpy.logical_and(occurancesInSin == -1, occurancesInCos == -1, occurancesInDist == -1)] = 1
    
    problemIndices = numpy.where(compareMat==1)
    
    return diffMatSinAbs, diffMatCosAbs, diffMatDistAbs
    

def lineFilter(angles,distances, distThreshA, distThreshD):

    diffMatA = difference_matrix(angles.T)
    
    diffMatAAbs = numpy.absolute(numpy.triu(diffMatA))
    
    #distThreshA = 2
    
    diffMatAAbs[numpy.logical_and(diffMatAAbs < distThreshA, diffMatAAbs != 0)] = -1
    
    
    diffMatD = difference_matrix(distances.T)
    
    diffMatDAbs = numpy.absolute(numpy.triu(diffMatD))
    
    #distThreshD = 50
    
    diffMatDAbs[numpy.logical_and(diffMatDAbs > distThreshD, diffMatDAbs != 0)] = -1
    
    compareMat = numpy.zeros([numpy.size(diffMatDAbs,1), numpy.size(diffMatDAbs,1)])
    compareMat[numpy.logical_and(diffMatDAbs == -1, diffMatAAbs == -1)] = 1
    
    indecesArr = numpy.where(compareMat==1)
    print(indecesArr)
    problemIndices = numpy.unique( indecesArr )
    
#    distances = numpy.delete(distances,problemIndices)
#    angles = numpy.delete(angles,problemIndices)
    
    return problemIndices


#problemPairsA = numpy.array([angles[0,indecesArr[0]], angles[0,indecesArr[1]]]) 
#problemPairsD = numpy.array([distances[0,indecesArr[0]], distances[0,indecesArr[1]]]) 
#
#
#problemCoord = circle2cart_drone([0,0],problemPairsA[0,:], problemPairsD[0,:])