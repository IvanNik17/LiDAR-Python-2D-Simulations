# -*- coding: utf-8 -*-
"""
Created on Wed Sep 27 13:34:14 2017

@author: ivan
"""


import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import pdist

import sys
import random
import time
import math
from os import walk

import csv

from CreateMovementPattern import scanningPattern_leftToRight, scanningPattern_downToUp

from helperComputationFunctions import testEllipse, circularMean, circle2cart_drone, circle2cart_points, intersectionLineCurve, intersectionLineCurveOLD,eq, cart2pol, find_nearest


class AnimatedScatter(object):

    
    def __init__(self,patternType = 'LtoR',  maxRange = 30000, axisRange = 10000):
        self.maxRange = maxRange
        self.axisRange = axisRange
        self.timeStamp = 0

        self.startP = []
        self.patternType = patternType 
        
        self.angle = 0
        self.searchRad= 6000
        
        self.scanningDistance = 3000
        
        self.detectedPoints = np.array([[-99999,-99999]])
        
        
        self.distList = []
        self.angleList = []

        self.newPos = []

        self.droneOrientationAngle = 0
        
        self.droneArmedAngle = 0
        
        self.bladeOrientationAngle = 0
        
        self.bladeCalcOrientationAngle = 0
        
        self.calcBladeAngle = False
        
        self.startEllipseAlg = False
        
        self.startMovementAlg = False
        
        self.bladeSway = 0
        self.swayDir = 1
        
        self.adjustToSwap = False
        self.v_xM = 0
        self.v_yM = 0
        
        self.ellipseStart = [0,0]

        self.allDetectedPoints = np.array([[0,0,0]])
        
        self.intersectionPoints = np.array([[0,0,0]])
        
        self.rotationCounter = 0
        
        self.allDetectedPoints_moved = np.array([[0,0]])
        
        self.bladeSaveName = ""
        
        self.scat_intersect = []
        self.scat_movedIntersect=[]
        
        
#        self.prevMeanDist = 
        
#        self.deltax = self.endP[0] - self.startP[0]
#        self.deltay = self.endP[1] - self.startP[1]


        
        
        
        
#        self.angle_rad = math.atan2(self.deltay,self.deltax)
#        angle_deg = angle_rad*180.0/math.pi
        
#        self.dist = math.hypot(self.endP[0] - self.startP[0], self.endP[1] - self.startP[1])
        
        

        # Setup the figure and axes...
        self.fig, self.ax = plt.subplots()
        # Then setup FuncAnimation.
        self.ani = animation.FuncAnimation(self.fig, self.update2D, interval=1./40, 
                                       init_func=self.setup_plot2D, blit=True)
        

            
        mngr = plt.get_current_fig_manager()
        # to put it into the upper left corner for example:
        geom = mngr.window.geometry()
        x,y,dx,dy = geom.getRect()
        mngr.window.setGeometry(50,1080 - 50 - dy,dx, dy)    
                
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        
        self.onClickEv = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_press_key)
#    def __enter__(self):
#        return self
    
    def handle_close(self,evt):
        print('Closed Figure!')
        self.fig.canvas.mpl_disconnect(self.onClickEv)
        
        # saveStrAirfoil = 'F:\\Ivan\\Airframe_testing_forPaper\\airfoil_' + self.bladeSaveName + '.txt'
        # saveStrIntersections = 'F:\\Ivan\\Airframe_testing_forPaper\\intersections\\airfoil_intersections_'+ self.bladeSaveName +'.txt'
        
        # np.savetxt(saveStrAirfoil, self.allDetectedPoints, delimiter=',', fmt='%1.3f') 
        # np.savetxt(saveStrIntersections, self.intersectionPoints, delimiter=',', fmt='%1.3f') 
        
        
        
    
    def on_click(self, evt):
        
        ix, iy = evt.xdata, evt.ydata
        self.startP = []
        self.startP.append((ix, iy))   
        
        
        
        self.x_temp = self.startP[0][0]
        self.y_temp = self.startP[0][1]
#        self.x_temp = -600
#        self.y_temp = -3000
        
        
       
        
        
    def on_press_key(self, evt):
        print('press', evt.key)
        sys.stdout.flush()
        
        if evt.key == 'c':
            self.calcBladeAngle = True
        if evt.key == 'v':
            self.startEllipseAlg = True
            self.droneArmedAngle = self.droneOrientationAngle
             
        if evt.key == 'b':
            self.startMovementAlg = True
            
            self.count = 0
            
#self.ellipseStart
            print(self.ellipseStart)
            x_p,y_p,z_p = scanningPattern_leftToRight(int(self.bladeCalcOrientationAngle), [self.ellipseStart[0],self.ellipseStart[1]], self.scanningDistance, 180, -10, 0, 400, -10)
#            x_s,y_s,z_s = scanningPattern_leftToRight(int(self.bladeCalcOrientationAngle), [0,0], 2000, 0, 190, 0, 400, 10)
#            
#            dist_p = math.hypot(x_p[0] - self.x_temp, y_p[0] - self.y_temp)
#            
#            dist_s = math.hypot(x_s[0] - self.x_temp, y_s[0] - self.y_temp)
#            
#            if dist_p < dist_s:
#                self.positionsToGo = np.array([x_p,y_p]).T 
#            else:
#                self.positionsToGo = np.array([x_s,y_s]).T 
##        
            self.positionsToGo = np.array([x_p,y_p]).T 
            self.endP = AnimatedScatter.getNextPos(self, self.count)
            
            self.dist_traveled = math.sqrt((self.endP[0] - self.x_temp)**2 + (self.endP[1] - self.y_temp)**2)
            
        
        
    def setup_plot2D(self):
        """Initial drawing of the scatter plot."""
        # Generate static point
#        lidarPoint = [0,0]
#        frontPoint = [[0.0,0.0], [0,self.maxRange]]
#
#        """Setup static markings."""
#        # Draw lidar position
#        self.ax.scatter(lidarPoint[0],lidarPoint[1],marker='o', c='r', s=10)
#        # Draw lidar front direction
#        self.ax.plot(frontPoint[:][0], frontPoint[:][1], 'g-')
        # Draw Max distance circle
        
#        with open("testBlade.csv", "r") as f:
#            reader = csv.reader(f)
#            csvList = list(reader)
#            
#            
#        bladeList = [i[0:2] for i in csvList]
#        
#        bladeArray = np.array(bladeList)
#        self.bladeArray = bladeArray.astype(float)*2


        stringpath = r"NACA4"

        f = []
        for (dirpath, dirnames, filenames) in walk(stringpath):
            f.extend(filenames)
            break
        
        whichBlade = filenames[8]
        
        self.bladeSaveName = whichBlade[5:9]
        
        airfoilDir = dirpath + "\\" +whichBlade
        airfoil = np.loadtxt(airfoilDir)
        self.bladeArray = np.array(airfoil)
        
        self.bladeArray[:,0] -= np.mean(self.bladeArray[:,0])
        
        self.bladeArray = self.bladeArray * 3000



        self.bladeArray = AnimatedScatter.rotateBlade(self,self.bladeArray, self.bladeOrientationAngle)
        

        
#        self.ax.scatter(self.bladeArray[:,0],self.bladeArray[:,1],marker='.', c='r', s=15)
        
#        _, ellipseBladePoints = testEllipse(260, 1480, 0, [0,0])
#        
#        self.ax.scatter(ellipseBladePoints[:,0],ellipseBladePoints[:,1],marker='x', c='r', s=10)
#        circle2 = plt.Circle((0,0), self.maxRange, color='r', fill=False)
#        plt.gca().add_artist(circle2)
        # Setup axis limits
        self.ax.axis([-self.axisRange, self.axisRange, -self.axisRange, self.axisRange])

        # Create updating scatter plot and variable for later use
        self.scat = self.ax.scatter([], [], c='b', s=18, animated=True, alpha=0.5)
        
        self.scat_end = self.ax.scatter([], [], c='b', s=25, animated=True, alpha=0.8)
        
        self.scat_detected = self.ax.scatter([], [], c='r', s=20, animated=True, alpha=1)
        
        self.scat_calcPos = self.ax.scatter([], [], c='r', s=20, animated=True, alpha=1)
        
        
        self.scat_blade = self.ax.scatter(self.bladeArray[:,0],self.bladeArray[:,1],c='r',s=10, animated = True, alpha = 0.1)
        
        self.scat_circle, = self.ax.plot([], [], linestyle="-")
        
        self.scat_ellipse = self.ax.scatter([],[],c='g',s=10, animated = True, alpha = 0.1)
        
        self.scat_traject = self.ax.scatter([],[],c='g',s=10, animated = True, alpha = 0.3)
        
        self.scat_droneHead, = self.ax.plot([], [], linestyle="-", c = "b")
        
        self.scat_pointCheck, = self.ax.plot([], [], linestyle="-")
        
        self.eigenVec, = self.ax.plot([], [], linestyle="-", c="g")
        
        self.scat_foundPoints =self.ax.scatter([],[],c='g',s=30, animated = True, alpha = 1)
        
        
        self.scat_intersect = self.ax.scatter([],[],c='g',s=30, animated = True, alpha = 1)
        self.scat_movedIntersect = self.ax.scatter([],[],c='b',s=30, animated = True, alpha = 1)
#        self.circle_droneSpan = plt.Circle((5, -5), 0.75, fc='y')
#        
#        self.circle_droneSpan.center = (5, 5)
#        
#        self.ax.add_patch(self.circle_droneSpan)

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat, self.scat_end, self.scat_circle, self.scat_detected, self.scat_calcPos, self.scat_pointCheck, self.scat_blade, self.scat_ellipse, self.scat_traject, self.scat_droneHead, self.eigenVec, self.scat_foundPoints,self.scat_intersect,self.scat_movedIntersect,

    def update2D(self, i):
        """Update the scatter plot."""
        #===============================================================#
        #======= Change this part depending on the sensor used!! =======#
        #===============================================================#
        
   #        real life  speed = 1m/s and 1000 mm movement each direction
#        self.bladeSway += self.swayDir*0.5
#        
#        self.bladeArray[:,0] += self.swayDir*0.5
#        
#        if self.bladeSway > 300:
#            self.swayDir = -1
#            
#        elif self.bladeSway < -300:
#            self.swayDir = 1
#            
        
        if len(self.startP) is not 0:
            
            
#            self.droneOrientationAngle += random.uniform(-0.5,0.5) % 360
            
          
            
            
#            self.droneOrientationAngle = self.droneOrientationAngle % 360
            
            
            self.angle += 1
#            print(self.angle)
            
            circlePos = circle2cart_drone(np.array([self.x_temp,self.y_temp]).T,self.angle, self.searchRad)
#            self.scat_circle.set_offsets([circlePos[0],circlePos[1]])
            
            self.scat_circle.set_data([circlePos[0],self.x_temp],[circlePos[1], self.y_temp])
            
            intersectionPoint, dist = intersectionLineCurve( circlePos, np.array([self.x_temp,self.y_temp]), self.bladeArray,self.searchRad)
            
            
            if intersectionPoint[0] != -1 and not np.any(eq(self.detectedPoints[:, 0], intersectionPoint[0])) and not np.any(eq(self.detectedPoints[:, 1], intersectionPoint[1])):
                self.detectedPoints = np.concatenate((self.detectedPoints,[intersectionPoint]))
                
                self.angleList.append(self.angle)
                self.distList.append(dist[0])
                self.scat_detected.set_offsets(self.detectedPoints)
                

                
            if self.angle > 360:
                
#                print(self.angleList)
                
                
                
                if len(self.distList) is not 0:
                    
                    
                   self.newPos, self.meanDist, self.meanAngle = AnimatedScatter.calculateDronePos(self, self.distList, self.angleList)
                   self.scat_calcPos.set_offsets(self.newPos.T)

                   
                   
                   self.diffDist = math.fabs(self.scanningDistance - self.meanDist)
                   
                   if self.diffDist > 100:
                       
                       self.adjustToSwap = True
                       

                           
                   else:
                       self.adjustToSwap = False
                          
                       
                      
                   

                
                self.angle = 0
                
                self.detectedPoints = np.delete(self.detectedPoints,0,0)
                self.detectedPoints = np.array([[-99999,-99999]])
                self.angleList = []    
                self.distList = []
                self.newPos = []
            

            if self.adjustToSwap == True:
                
                if np.sign(self.scanningDistance - self.meanDist) == 1:
                       
                   opositeAngle = (self.meanAngle+180)%360
#                   print("Correction needed: ", self.diffDist, " at angle ", self.meanAngle, " oposite ", opositeAngle)
                   self.changeEndX = self.diffDist*math.sin(math.radians(opositeAngle)) # 100 can be diffDist for instant teleport
                   self.changeEndY = self.diffDist*math.cos(math.radians(opositeAngle))
                   
    
                   pointCheckX = self.x_temp + self.changeEndX
                   pointCheckY = self.y_temp + self.changeEndY
                   
                   
                elif np.sign(self.scanningDistance - self.meanDist) == -1:
                   
                   
#                   print("Correction needed: ", self.diffDist, " at angle ", self.meanAngle)
                   self.changeEndX = self.diffDist*math.sin(math.radians(self.meanAngle)) # 100 can be diffDist for instant teleport
                   self.changeEndY = self.diffDist*math.cos(math.radians(self.meanAngle))
                   
    
                   pointCheckX = self.x_temp + self.changeEndX
                   pointCheckY = self.y_temp + self.changeEndY

                speed = 0.5
                self.scat_pointCheck.set_data([ self.x_temp,pointCheckX],[self.y_temp ,pointCheckY ])
                self.v_xM = speed/self.diffDist * self.changeEndX
                self.v_yM = speed/self.diffDist * self.changeEndY
                
                
            else:
                self.v_xM = 0
                self.v_yM = 0
                self.scat_pointCheck.set_data([ ],[])
                # new point
#                if self.startMovementAlg == True:
#                    self.endP[0] = self.endP[0] + v_xM
#                    self.endP[1] = self.endP[1] + v_xM
#
#
#                self.x_temp = self.x_temp + v_xM
#                self.y_temp = self.y_temp + v_yM

                
                
        
            if self.startMovementAlg == True:
                
                self.scat_traject.set_offsets(self.positionsToGo)
                
                if (math.fabs(self.x_temp - self.endP[0]) > 10 or math.fabs(self.y_temp - self.endP[1]) > 10):
                    
                    speed = 0.5
                    
                    # new point using speed and delta distance
        #            self.x_temp = self.x_temp + speed*self.deltax
        #            self.y_temp = self.y_temp + speed*self.deltay
                    
                    
                    # Recalculate dist traveled for a constant velocity            
                    self.dist_traveled = math.sqrt((self.endP[0] - self.x_temp)**2 + (self.endP[1] - self.y_temp)**2)
                    
                    
                    v_x = speed/self.dist_traveled * (self.endP[0] - self.x_temp)
                    v_y = speed/self.dist_traveled * (self.endP[1] - self.y_temp)
        
                    # new point using velocity
                    self.x_temp = self.x_temp + v_x + self.v_xM
                    self.y_temp = self.y_temp + v_y + self.v_yM
                    
                    self.endP[0] = self.endP[0] + self.v_xM
                    self.endP[1] = self.endP[1] + self.v_yM
                    
                else:
                    
                    self.count +=1
                    self.endP = AnimatedScatter.getNextPos(self, self.count)
                    
                    
                    self.scat_end.set_offsets([self.endP[0],self.endP[1]])  
                    self.scat_end.set_edgecolors((1,0,0,1))
                    
                    # Recalculate dist traveled only once per new point for a easying in velocity 
    #                self.dist_traveled = math.sqrt((self.endP[0] - self.x_temp)**2 + (self.endP[1] - self.y_temp)**2)

            else:
                self.x_temp = self.x_temp + self.v_xM
                self.y_temp = self.y_temp + self.v_yM
            frontPoint = AnimatedScatter.droneOrientation(self,self.droneOrientationAngle)
            


            self.scat_droneHead.set_data([ self.x_temp,frontPoint[0]],[self.y_temp ,frontPoint[1]])
        
        


        #===============================================================#
        #========================== BLOCK END ==========================#
        #===============================================================#

        # Set x and y data...
        self.scat.set_offsets([self.x_temp,self.y_temp])
        

        
        self.scat_blade.set_offsets(self.bladeArray)
        self.scat_blade.set_edgecolors((0,0,1,0.1))
        
#        self.circle_droneSpan.center(5,5)
        
        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_blade,self.scat, self.scat_end, self.scat_circle, self.scat_detected, self.scat_calcPos, self.scat_pointCheck, self.scat_ellipse, self.scat_traject, self.scat_droneHead,self.eigenVec, self.scat_foundPoints,self.scat_intersect,self.scat_movedIntersect,

    def show(self):
        plt.show()
        
    def close(self):
        plt.close()
        
        
        
    def getNextPos(self, counter):
        return self.positionsToGo[counter,:]

    def rotateBlade(self,bladeArray, angle):
        
        angle = -angle
        qx = 0 + math.cos(math.radians(angle)) * (bladeArray[:,0] - 0) - math.sin(math.radians(angle)) * (bladeArray[:,1] - 0)
        qy = 0 + math.sin(math.radians(angle)) * (bladeArray[:,0] - 0) + math.cos(math.radians(angle)) * (bladeArray[:,1] - 0)
        
        

        
        newCoord = np.array([qx,qy]).T

        return newCoord

    def droneOrientation(self,angle):
        
        arrowSize = 500
        
        distToEndX = arrowSize*math.sin(math.radians(angle))
        distToEndY = arrowSize*math.cos(math.radians(angle))
       

        positionFrontX = self.x_temp + distToEndX
        positionFrontY = self.y_temp + distToEndY
        
        return [positionFrontX,positionFrontY]



    def calculateAnglesPCA(self,bladePointsPos):
        
#        bladePointsPos[:,0] = bladePointsPos[:,0] - np.mean(bladePointsPos[:,0])
#        bladePointsPos[:,1] = bladePointsPos[:,1] - np.mean(bladePointsPos[:,1])
#        
#        coords = np.vstack([bladePointsPos[:,0], bladePointsPos[:,1]])
#        cov_mat = np.cov(coords)
#        
#        eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
#        
#        sort_indices = np.argsort(eig_val_cov)[::-1]
#        evec1, evec2 = eig_vec_cov[:, sort_indices]
#        x_v1, y_v1 = evec1  # Eigenvector with largest eigenvalue
#        x_v2, y_v2 = evec2
#
##        angleOffsetMeasured = np.tanh((x_v1)/(y_v1))  
#
##        print(math.degrees(np.arctan2( eig_vec_cov[0,1],eig_vec_cov[0,0])))
##        
##        print(math.degrees(np.arctan2( eig_vec_cov[1,1],eig_vec_cov[1,0])))
#        
##        print(eig_vec_cov)
##        
##        maxIndEigenval = np.argmax(eig_val_cov)
##        maxEigenvec = eig_vec_cov[:,maxIndEigenval]
#        self.eigenVec.set_data([ x_v1*-200*2,x_v1*200*2],[ y_v1*-200*2,y_v1*200*2 ])
#        
#        angleOffsetMeasured = math.degrees(np.arctan2( y_v1,x_v1   )) + 90
#        
#        print(angleOffsetMeasured)
#        print(math.degrees(np.arctan2( evec1[1],evec1[0])))
#        
##        print(math.degrees(np.arctan2( evec2[1],evec2[0])))
##        angleOffsetMeasured = math.degrees(np.arctan2( evec1[1],evec1[0]))
##        print("Max ", maxEigenvec)
##        print("Second max ", evec1)
##    ##    normally this is 90 not  270
#        angleOffsetMeasured =angleOffsetMeasured
#       
#        return angleOffsetMeasured

        cov_mat = np.cov(bladePointsPos.T)
    
        eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
                
        maxIndEigenval = np.argmax(eig_val_cov)
        evec1 = eig_vec_cov[:,maxIndEigenval]
        
        
        angleOffsetMeasured = math.degrees(np.arctan2( evec1[0],evec1[1]   )) 
        
        #print(math.degrees(numpy.arctan2( eig_vec_cov[0,0],eig_vec_cov[1,0]   )))
        #
        #print(math.degrees(numpy.arctan2( eig_vec_cov[0,1],eig_vec_cov[1,1]   )))
        
        if angleOffsetMeasured < 0:
            angleOffsetMeasured =  angleOffsetMeasured + 180
            
        return angleOffsetMeasured
        
   
    def closest_node(self,node, nodes):
        nodes = np.asarray(nodes)
        dist_2 = np.sum((nodes - node)**2, axis=1)
        return np.argmin(dist_2)
        

    
#    def movePointToHigherDensity_newest(self, angleDist, meanDist, meanAngle ):
#        
#        indexClosest = int(AnimatedScatter.closest_node(self,currIntersection, ellipsePointsPos))
#        
#        numBins = int(math.sqrt(len(angleDist)))
#        numMaxes = math.floor(numBins/2)
#        
#        if len(angleDist) > 10:
#            indicesToGet = len(angleDist)/4
#        else:
#            indicesToGet = len(angleDist)/2
#
#        indicesToGet = round(indicesToGet)
#        
#        closeIndValues = angleDist[indexClosest-indicesToGet:indexClosest+indicesToGet,:]
        
        
#    def movePointToHigherDensity_new(self,currIntersection, ellipsePointsPos  ):
#    
#        indexClosest = int(AnimatedScatter.closest_node(self,currIntersection, ellipsePointsPos))
#        print("number of points ",len(ellipsePointsPos))
#        print("index closest ",indexClosest)
#        
#        numBins = int(math.sqrt(len(ellipsePointsPos)))
#        numMaxes = math.floor(numBins/2)
#        
#        print("num bins ",numBins)
#        print("num maxes ",numMaxes)
#        
#        numIndices_before = indexClosest
#        numIndices_after = len(ellipsePointsPos) - indexClosest
#    
#        if len(ellipsePointsPos) > 10:
#            indicesToGet = numIndices_before/2 if numIndices_before<numIndices_after else numIndices_after/2
#        else:
#            indicesToGet = numIndices_before if numIndices_before<numIndices_after else numIndices_after
#        indicesToGet = round(indicesToGet)
#        
#        closeIndPoints = ellipsePointsPos[indexClosest-indicesToGet:indexClosest+indicesToGet,:]
#        
#        print("indices to get ",indicesToGet)
#        print("-----------")
#    
#        
#        numberX, binsX = np.histogram(closeIndPoints[:,0],bins=numBins)
#        numberY, binsY = np.histogram(closeIndPoints[:,1],bins=numBins)
#        
#        indLargestBinsX= np.argpartition(numberX, -numMaxes)[-numMaxes:]
#    
#        indLargestBinsY= np.argpartition(numberY, -numMaxes)[-numMaxes:]
#        
#        binsDistX = binsX[indLargestBinsX]
#        binsDistY = binsY[indLargestBinsY]
#    
#        newIntersectionX = np.mean(binsDistX)
#        
#        newIntersectionY = np.mean(binsDistY)
#        
#        indexClosest_new = int(AnimatedScatter.closest_node(self,np.array([[newIntersectionX,newIntersectionY]]), ellipsePointsPos))
#        
#        return np.array([[newIntersectionX,newIntersectionY]]), closeIndPoints, indexClosest_new

                         
    def movePointToHigherDensity_new(self,currIntersection, ellipsePointsPos  ):
        
        indexClosest = int(AnimatedScatter.closest_node(self,currIntersection, ellipsePointsPos))
        print("number of points ",len(ellipsePointsPos))
        print("index closest ",indexClosest)
        
        numBins = int(math.sqrt(len(ellipsePointsPos)))
        numMaxes = max(math.floor(numBins/2),2)
        
        print("num bins ",numBins)
        print("num maxes ",numMaxes)
        
        numIndices_before = indexClosest
        numIndices_after = len(ellipsePointsPos) - indexClosest
        indicesToGet = -1
        if len(ellipsePointsPos) > 10:
            indicesToGet = numIndices_before/2 if numIndices_before<numIndices_after else numIndices_after/2
            indicesToGet = round(indicesToGet)
        
            closeIndPoints = ellipsePointsPos[indexClosest-indicesToGet:indexClosest+indicesToGet,:]
            
        else:
            indicesToGet = len(ellipsePointsPos)
            
            closeIndPoints = ellipsePointsPos
        
        
        print("indices to get ",indicesToGet)
        print("-----------")
    
        
        numberX, binsX = np.histogram(closeIndPoints[:,0],bins=numBins)
        numberY, binsY = np.histogram(closeIndPoints[:,1],bins=numBins)
        
        indLargestBinsX= np.argpartition(numberX, -numMaxes)[-numMaxes:]
    
        indLargestBinsY= np.argpartition(numberY, -numMaxes)[-numMaxes:]
    
        
        binsDistX = binsX[indLargestBinsX]
        binsDistY = binsY[indLargestBinsY]
    
        newIntersectionX = np.mean(binsDistX)
        
        newIntersectionY = np.mean(binsDistY)
        
        indexClosest_new = int(AnimatedScatter.closest_node(self,np.array([newIntersectionX,newIntersectionY]), ellipsePointsPos))
        
        return np.array([[newIntersectionX,newIntersectionY]]), closeIndPoints, indexClosest_new
                        
                        
    
    def movePointToHigherDensity(self,currIntersection, ellipsePointsPos, indicesToGet, numBins, numMaxes  ):
        
        indexClosest = int(AnimatedScatter.closest_node(self,currIntersection, ellipsePointsPos))
    
        
        
        
        if indexClosest-indicesToGet < 0:
            remainderInd = int(math.fabs(indexClosest-indicesToGet))
            remainderPointsNeg = ellipsePointsPos[-remainderInd:-1,:]
            remainderPointsAfter = ellipsePointsPos[0:indexClosest+indicesToGet,:]
        
            closeIndPoints = np.concatenate((remainderPointsNeg,remainderPointsAfter))
        
        elif indexClosest+indicesToGet > len(ellipsePointsPos): 
            remainderInd = int((indexClosest+indicesToGet) - len(ellipsePointsPos) )
            remainderAfterZero = ellipsePointsPos[0:remainderInd,:]
            remainderBeforeZero = ellipsePointsPos[indexClosest-indicesToGet:-1,:]
        
            closeIndPoints = np.concatenate((remainderAfterZero,remainderBeforeZero))
        else:
            closeIndPoints = ellipsePointsPos[indexClosest-indicesToGet:indexClosest+indicesToGet,:]
    
        
        numberX, binsX = np.histogram(closeIndPoints[:,0],bins=numBins)
        numberY, binsY = np.histogram(closeIndPoints[:,1],bins=numBins)
        
        indLargestBinsX= np.argpartition(numberX, -numMaxes)[-numMaxes:]
    
        indLargestBinsY= np.argpartition(numberY, -numMaxes)[-numMaxes:]
        
        binsDistX = binsX[indLargestBinsX]
        binsDistY = binsY[indLargestBinsY]
    
        newIntersectionX = np.mean(binsDistX)
        newIntersectionY = np.mean(binsDistY)
        
        return np.array([[newIntersectionX,newIntersectionY]])        
        
        
        
    def CalcAngleBetweenCenterAndPoints(self,point1, point2, center):
        #create vectors
#        point1 = [60, 0]
#center = [0, 0]
#point2 = [60, -60]
#
#
#point1 = np.array([point1].T)
#center = np.array([center].T)
#point2 = np.array([point2].T)
        
        
        
        
        ba = point1 - center
        bc = point2- center
        # calculate angle
        cosine_angle = np.dot(ba,bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        
        angle = np.arccos(cosine_angle) 
        pAngle = np.degrees(angle)
        
        return pAngle
        
        
    
        

    def calculateDronePos(self, distListL, angleListL):
        
        
        
        meanDist = sum(distListL)/ float(len(distListL))
                    
        angleArray = (np.array([angleListL]) + 180 )%360

        compensateYaw = self.droneOrientationAngle - self.droneArmedAngle

        angleArray[:,0] = angleArray[:,0] + compensateYaw
        
        distArray = np.array([distListL])
        
        angleDistance = np.concatenate([angleArray,distArray],axis =0).T

        weight = np.ones(len(angleDistance))/len(angleDistance)
        angleArrayForMen = angleArray.T
       

#        meanAngle = circularMean(angleArrayForMen,weight)    
        meanAngle = np.degrees(math.atan2(np.sum(np.sin(np.radians(angleDistance[:,0]))),np.sum(np.cos(np.radians(angleDistance[:,0])))))
        
#        newMeanAngleDist, _, indexToGet = AnimatedScatter.movePointToHigherDensity_new(self,np.array([meanAngle, meanDist]), angleDistance)    
#        print("New means",newMeanAngleDist)
#        print("Old means" ,meanAngle, " ", meanDist)
        
#        meanAngle = newMeanAngleDist[0][0]
#        meanDist = newMeanAngleDist[0][1]

        
                   
        dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
        

#        print(angleDistance)
  
        bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
        
#        averagePointPos=[sum(bladePointsPos[:,0])/len(bladePointsPos[:,0]),sum(bladePointsPos[:,1])/len(bladePointsPos[:,1])]
#        averagePointPos_new,closeIndPoints, indexClosest_new = AnimatedScatter.movePointToHigherDensity_new(self,averagePointPos, bladePointsPos  )
        
        

        
#        print(meanAngle)
#        meanAngle = angleDistance[indexClosest_new,0]
#        meanDist = angleDistance[indexClosest_new,1]
#        print(meanAngle_2)

#        print(angleDistance)
#        meanAngle = np.degrees(math.atan2(np.sum(np.sin(np.radians(angleDistance[:,0]))),np.sum(np.cos(np.radians(angleDistance[:,0])))))
#        meanDist = angleDistance[indexClosest_new,1]

        dronePos = circle2cart_drone([0,0],meanAngle, meanDist)
        bladePointsPos = circle2cart_points(dronePos,angleDistance, 0)
        
#        print("Old dist ", math.hypot(self.x_temp - dronePos[0], self.y_temp - dronePos[1]))
#        print("New dist ",math.hypot(self.x_temp - dronePos_2[0], self.y_temp - dronePos_2[1]))
        
#        print(math.sqrt( (dronePos_2[0] - dronePos[0])**2 + (dronePos_2[1]  - dronePos[1])**2 ))
        
        if self.calcBladeAngle == True:
            
            
            averagePointPos_ang=[sum(bladePointsPos[:,0])/len(bladePointsPos[:,0]),sum(bladePointsPos[:,1])/len(bladePointsPos[:,1])]
            bladePointsPos_moved = bladePointsPos
            bladePointsPos_moved[:,0] -=  averagePointPos_ang[0]
            bladePointsPos_moved[:,1] -=  averagePointPos_ang[1]

            self.bladeCalcOrientationAngle = AnimatedScatter.calculateAnglesPCA(self,bladePointsPos_moved)
            print(self.bladeCalcOrientationAngle)
            self.calcBladeAngle = False;
            
            distAll = pdist(bladePointsPos)
            
            
            
            self.maxDist = distAll.max()
            print(self.maxDist)
            
        
        if self.startEllipseAlg == True:
            
            averagePointPos=[sum(bladePointsPos[:,0])/len(bladePointsPos[:,0]),sum(bladePointsPos[:,1])/len(bladePointsPos[:,1])]
            
            distCentToPoint = np.array([ [0,0],  [averagePointPos[0],averagePointPos[1]] ])
            distAveragePointToZero = pdist(distCentToPoint,'euclidean');
            
#            print(self.maxDist)
    
            ellipseRadAngle, ellipseBladePoints = testEllipse(int(self.maxDist/6), int(self.maxDist), self.bladeCalcOrientationAngle, [0,0])
            
            self.ellipseStart[0] = ellipseBladePoints[0,0]
            self.ellipseStart[1] = ellipseBladePoints[0,1]
            
            self.scat_ellipse.set_offsets(ellipseBladePoints)
            
            intersectPointOnCurve = intersectionLineCurveOLD([0,0], dronePos, ellipseBladePoints)
            correctionDist = math.sqrt(intersectPointOnCurve[0]**2 + intersectPointOnCurve[1]**2)
            
#            print(intersectPointOnCurve)    

        
#            BETTER POSITIONING ALGO USING ELLIPSE
#            newIntersection = AnimatedScatter.movePointToHigherDensity(self,intersectPointOnCurve, ellipseBladePoints,20, 10, 4)
#
#            newIntersection_less = np.squeeze(newIntersection,0)
#            
##            correctionDist = math.sqrt(newIntersection_less[0]**2 + newIntersection_less[1]**2)
#            distX = intersectPointOnCurve[0] - newIntersection_less[0]
#            distY = intersectPointOnCurve[1] - newIntersection_less[1]



#            print("Distance change X: ", distX, " Distance change Y: ", distY)
#            print([intersectPointOnCurve[0],intersectPointOnCurve[1]])
            
#            print([float(bla[0]),float(bla[1])])
            
#            angleBetweenTwo = AnimatedScatter.CalcAngleBetweenCenterAndPoints(self,intersectPointOnCurve, newIntersection_less, [0,0])
#            print(angleBetweenTwo)
#            meanAngle += angleBetweenTwo

#            radiusIndex =np.where(ellipseRadAngle[:,1] == meanAngle)
#     Correction using angle correspondance       
#            radEll, anglesEll = cart2pol(ellipseBladePoints[:,0], ellipseBladePoints[:,1])
#            anglesEll = np.degrees(anglesEll)
#
##            anglesEll_test = anglesEll %360
#            anglesEll_test = anglesEll %360 #(anglesEll + 90)%360
#    
#            anglesEll_test = (anglesEll_test + 180)%360
#
##            anglesEll_test = np.flipud(anglesEll_test)
#            indexClosest =  find_nearest(anglesEll_test,meanAngle )
#            distanceToEll = ellipseRadAngle[indexClosest,0]
            
#            print("From Ellipse: ", distanceToEll, " From intersection ", correctionDist)
#            print(distAveragePointToZero)
            
            newCenterPos = circle2cart_drone([0,0],meanAngle, correctionDist-distAveragePointToZero) # -distAveragePointToZero                              
                                             
                 
            newDronePos = circle2cart_drone(newCenterPos,meanAngle, meanDist)
            newDronePos = np.array(newDronePos)
            
            
            
            newBladePointsPos = circle2cart_points(newDronePos,angleDistance, 0)
            
#            newIntersection, closeIndPoints, indexToGet = AnimatedScatter.movePointToHigherDensity_new(self,intersectPointOnCurve, newBladePointsPos)
#            newIntersection_less = np.squeeze(newIntersection,0)
#            
#
#            distX = intersectPointOnCurve[0] - newIntersection_less[0]
#            distY = intersectPointOnCurve[1] - newIntersection_less[1]
#            

#           THIS WORKS - UNCOMMENT FOR ADDITIONAL POSITIONING ACCURACY
#            newDronePos[0] -= (averagePointPos[0] - averagePointPos_new[0][0])
#            newDronePos[1] -= (averagePointPos[1] - averagePointPos_new[0][1])

#            print(math.hypot(self.x_temp - newDronePos[0], self.y_temp - newDronePos[1]))
 
    
            
            
            rotationCounterArray = np.ones([len(newBladePointsPos),1]) * self.rotationCounter

            
            
            newBladePointsPos =np.concatenate ((newBladePointsPos,rotationCounterArray),1)
            
            self.rotationCounter +=1
            
            self.allDetectedPoints = np.concatenate((self.allDetectedPoints,newBladePointsPos))
            
            
#            intersectPointOnCurve_iter =np.concatenate ((intersectPointOnCurve,self.rotationCounter),1)
            intersectPointOnCurve_iter = np.array([[intersectPointOnCurve[0],intersectPointOnCurve[1],self.rotationCounter]])
            
            
            self.intersectionPoints = np.concatenate((self.intersectionPoints,intersectPointOnCurve_iter))
                
            
#            self.scat_intersect.set_offsets(closeIndPoints)
            self.scat_movedIntersect.set_offsets(intersectPointOnCurve)
#            for j in range(0,len(angleDistance[:,0])):
#                indexClosest_temp =  find_nearest(anglesEll_test,angleDistance[j,0] )
#                distanceToEll_temp = ellipseRadAngle[indexClosest_temp,0]
#                
#                
##                angleDist_temp = np.array([angleDistance[j,0], angleDistance[j,1]])
#               
#                newPos_moved = circle2cart_drone([0,0],angleDistance[j,0],distanceToEll_temp)
#                
##                print(type(newPos_moved))
##                
#                newPos_moved = np.array([newPos_moved])
##                
#                self.allDetectedPoints_moved = np.concatenate((self.allDetectedPoints_moved,newPos_moved))
            
            
#            self.scat_foundPoints.set_offsets(self.allDetectedPoints)
            
            
            return newDronePos, meanDist, meanAngle
        else:
            return np.array(dronePos), meanDist, meanAngle
        
        

if __name__ == '__main__':
#    with AnimatedScatter(30000,5000) as a:
    
    a = AnimatedScatter('LtoR',2000,5000)
    a.show()