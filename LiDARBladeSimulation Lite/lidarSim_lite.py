# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:48:07 2018

@author: ivan
"""



import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation


import sys



from helperComputationFunctions_lite import  intersectionLineCurve,eq, circle2cart_lidarLine, calculateDronePos, rotateTranslateBlade, droneOrientation, lidarRotation_change, loadBlade, calculateMinMax,calculateAnglesPCA


class AnimatedScatter(object):

    
    def __init__(self, bladeName,bladeScale = 3000, axisRange = 5000, searchRad = 6000, droneOrientationAngle = 0, bladeOrientationAngle = 0, translateBlade = [0,0]):
        
        self.axisRange = axisRange


        self.startP = []

        self.lidarOffset = 0
        self.angle = 0
        self.searchRad= searchRad

        
        self.detectedPoints = np.array([[-99999,-99999]])
        
        
        self.distList = []
        self.angleList = []

        self.distList_prev = []
        self.angleList_prev = []

        self.newPos = []

        self.droneOrientationAngle = self.lidarOffset
        
        self.bladeOrientationAngle = bladeOrientationAngle
        
        self.translateBlade = translateBlade
        
        self.bladeName = bladeName
        
        self.bladeScale = bladeScale
        
        
        
        self.fig = plt.figure(figsize=(25, 10), dpi=50)
        
        # Setup the figure and axes...
        
        self.ax_sim = self.fig.add_subplot(121)
        self.ax_sim.set_title('LiDAR simulator Window')

        self.ax_calc = self.fig.add_subplot(122)
        self.ax_calc.set_title('Calculated positions Window')
        
        
        self.droneRot = 0


        # Then setup FuncAnimation - for simulation window
        self.ani_sim = animation.FuncAnimation(self.fig, self.update2D_sim, interval=1./40, 
                                       init_func=self.setup_plot2D_sim, blit=True)
        

        # Then setup FuncAnimation. - for calculated positions window
        self.ani_calc = animation.FuncAnimation(self.fig, self.update2D_calc, interval=1./40, 
                                       init_func=self.setup_plot2D_calc, blit=True)
        

         
        
           
        # Initialize the different listeners - close_event, button press on the mouse and key press on the keyboard              
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        
        self.onClickEv = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.fig.canvas.mpl_connect('key_press_event', self.on_press_key)

#   Three  listener functions
    def handle_close(self,evt):
        print('Closed Figure!')
        self.fig.canvas.mpl_disconnect(self.onClickEv)
        
        self.close()
        
        

#   Select the LiDAR position from the mouse position    
    def on_click(self, evt):
        
        ix, iy = evt.xdata, evt.ydata
        self.startP = []
        self.startP.append((ix, iy))   
        
        
        self.x_temp = self.startP[0][0]
        self.y_temp = self.startP[0][1]



 
    def on_press_key(self, evt):
        print('press', evt.key)
        sys.stdout.flush()
        

            
        
#   Setup function for the simulation animation    
    def setup_plot2D_sim(self):
        """Initial drawing of the scatter plot."""


        self.bladeArray = loadBlade(self.bladeName, self.bladeScale)
        
        if np.isscalar(self.bladeArray):
            print("No such blade found. Stopping Program")
            self.close()
            

        self.bladeArray = rotateTranslateBlade(self.bladeArray, self.bladeOrientationAngle, self.translateBlade)
 
        # Setup axis limits
        self.ax_sim.axis([-self.axisRange, self.axisRange, -self.axisRange, self.axisRange])

        # Create updating scatter plot and variable for later use
        self.scat = self.ax_sim.scatter([], [], c='b', s=18, animated=True, alpha=0.5)
        
        self.scat_detected = self.ax_sim.scatter([], [], c='r', s=20, animated=True, alpha=1)
        
        self.scat_calcPos = self.ax_sim.scatter([], [], c='r', s=20,marker = 'x', animated=True, alpha=1)
        
        
        self.scat_blade = self.ax_sim.scatter(self.bladeArray[:,0],self.bladeArray[:,1],c='r',s=10, animated = True, alpha = 0.1)
        
        self.scat_circle, = self.ax_sim.plot([], [], linestyle="-")
        

        
        self.scat_droneHead, = self.ax_sim.plot([], [], linestyle="-", c = "b")
        

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat, self.scat_circle, self.scat_detected, self.scat_calcPos, self.scat_blade, self.scat_droneHead, 
        
        
#   Setup function for the calculated position animation          
    def setup_plot2D_calc(self):
        """Initial drawing of the scatter plot."""


        
        # Setup axis limits
        self.ax_calc.axis([-self.axisRange, self.axisRange, -self.axisRange, self.axisRange])

        # Create updating scatter plot and variable for later use

        
        self.scat_calcPos_2 = self.ax_calc.scatter([], [], c='r', s=20,marker = 'x', animated=True, alpha=1)
        
        self.scat_calcBladePos_2 = self.ax_calc.scatter([], [], c='g', s=20, marker = 'x', animated=True, alpha=1)
        
        self.scat_detectedEndPoints = self.ax_calc.scatter([], [], c='r', s=25, marker = 'o', animated=True, alpha=1)
        


        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_calcPos_2, self.scat_calcBladePos_2, self.scat_detectedEndPoints, 
        
#   Update function for the calculated position animation          
    def update2D_calc(self, i):
        

        if len(self.distList) is not 0:
                  
           self.newPos, self.newBladePos , self.meanDist, self.meanAngle = calculateDronePos(self.distList_prev, self.angleList_prev)
           self.scat_calcPos_2.set_offsets(self.newPos.T)
           
           

           self.scat_calcBladePos_2.set_offsets(self.newBladePos)
           
           
           maxDist,maxDist_points, twoMaxIndeces = calculateMinMax(self.newBladePos)
           
           angle = calculateAnglesPCA(self.newBladePos)
           print(angle)
           
           self.scat_detectedEndPoints.set_offsets(maxDist_points)
           
           
        
        return self.scat_calcPos_2, self.scat_calcBladePos_2, self.scat_detectedEndPoints,


           
#   Update function for the simulation animation  
    def update2D_sim(self, i):
        """Update the scatter plot."""
       
#       Check if a position is selected with mouse
        if len(self.startP) is not 0:
            
#           update the angle 
            self.angle += 1
            
#           update the LiDAR laser orientation 
            circlePos = circle2cart_lidarLine(np.array([self.x_temp,self.y_temp]).T,self.angle, self.searchRad)

            
            self.scat_circle.set_data([circlePos[0],self.x_temp],[circlePos[1], self.y_temp])
#           Check for intersection of the current LiDAR laser orientation with the blade
            intersectionPoint, dist = intersectionLineCurve( circlePos, np.array([self.x_temp,self.y_temp]), self.bladeArray,self.searchRad)
            
#           Check if there is any intersection. And the point of intersection is a new one 
            if intersectionPoint[0] != -1 and not np.any(eq(self.detectedPoints[:, 0], intersectionPoint[0])) and not np.any(eq(self.detectedPoints[:, 1], intersectionPoint[1])):
                self.detectedPoints = np.concatenate((self.detectedPoints,[intersectionPoint]))
                
                
                self.angleList.append(self.angle+ self.lidarOffset)
                self.distList.append(dist[0])
                self.scat_detected.set_offsets(self.detectedPoints)
                

#           If the LiDAR laser has made a full 360 degree rotation                
            if self.angle >= 360:
                
                self.distList_prev = self.distList
                self.angleList_prev = self.angleList
                
                
#                if len(self.distList) is not 0:
#                    self.scat_calcPos.set_offsets(self.newPos.T)
                


                self.angle = 0
#               Clear all 
                self.detectedPoints = np.delete(self.detectedPoints,0,0)
                self.detectedPoints = np.array([[-99999,-99999]])
                self.angleList = []    
                self.distList = []
                self.newPos = []
            


            frontPoint = droneOrientation(self.x_temp, self.y_temp, self.droneOrientationAngle)
            


            self.scat_droneHead.set_data([ self.x_temp,frontPoint[0]],[self.y_temp ,frontPoint[1]])
        


        # Set x and y data...
        self.scat.set_offsets([self.x_temp,self.y_temp])
        

        
        self.scat_blade.set_offsets(self.bladeArray)
        self.scat_blade.set_edgecolors((0,0,1,0.1))
        

        
        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_blade,self.scat, self.scat_circle, self.scat_detected, self.scat_droneHead, self.scat_calcPos,
    
        
        
    def show(self):
        plt.show()
        
    def close(self):
        plt.close()
        

if __name__ == '__main__':

#   bladeName - name of the blade file, bladeScale = 3000 - rescale the blade, axisRange = 5000 - size of the axis of the figure,
#   searchRad = 6000 - radius of the LiDAR beam, droneOrientationAngle = 0 - initial drone orientation angle,
#   bladeOrientationAngle = 0 - orientation of the blade, translateBlade = [0,0] - translation of the blade
    
    a = AnimatedScatter("NACA 0024", 1000, 2000)
    a.show()