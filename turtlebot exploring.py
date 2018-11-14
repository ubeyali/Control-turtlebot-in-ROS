#!/usr/bin/env python

#  Created by Ubey Ali on 21.10.2018.
#  Copyright © 2018 Ubey Ali. All rights reserved.
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## to check if the returned value is nan or not
from math import isnan
## Common ROS headers.
import rospy
## Required for some printing options
import sys

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid

global start
start=False
## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##
def laser_callback(data):
    ## Lets fill a twist message for motor command
    motor_command = Twist()
    global start
    if not start:
	    motor_command.linear.x = 1
   	    motor_command.angular.z = 0
	    global start
            start=True
##to go forward if there is no obstacle
    elif data.ranges[0]>1 and  data.ranges[-1]>1 and data.ranges[len(data.ranges)/2]>1 :
		    print'1 forward'
		    motor_command.linear.x = 1
	   	    motor_command.angular.z = 0	
##turn completely     
    elif data.ranges[0]<1 and data.ranges[-1]<1 and data.ranges[len(data.ranges)/2]<1:
		if data.ranges[0]> data.ranges[-1]<1:
		    print'2 turn comp left'
		    motor_command.linear.x = 0
	   	    motor_command.angular.z = 7	 
		else :
		    print'3 turn comp right'
		    motor_command.linear.x = 0
	   	    motor_command.angular.z = -7	
##turn left if rightmost scanned point is less than 1 	 	    
    elif data.ranges[0]<1:
		    print'4 left'
		    motor_command.linear.x = 0
	   	    motor_command.angular.z = 1	    
##turn right if lefttmost scanned point is less than 1 	 	
    elif data.ranges[-1]<1:
		    print'5 right'
		    motor_command.linear.x = 0
	   	    motor_command.angular.z = -1
    elif data.ranges[len(data.ranges)/2]>data.ranges[-1] and data.ranges[len(data.ranges)/2]>data.ranges[0]:
		    print'6'
		    motor_command.linear.x = 1
	   	    motor_command.angular.z = 0.2
    elif isnan(data.ranges[len(data.ranges)/2]):
		    print'7 turn'
		    motor_command.linear.x = 1
 		    motor_command.angular.z = 0.2   	
    else:
	    motor_command.linear.x = 1
            print'0'
   	    motor_command.angular.z = -1
	
    global motor_command_publisher
    motor_command_publisher.publish(motor_command)
    
    ## Alternatively we could have looked at the laser scan BEFORE we made this decision
    ## Well Lets see how we might use a laser scan
    ## Laser scan is an array of distances
    print 'Number of points in laser scan is: ', len(data.ranges)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
    print 'The minimum angle scanned by the laser is: ', data.angle_min
    print 'The maximum angle scanned by the laser is: ', data.angle_max
    print 'The increment in the angles scanned by the laser is: ', data.angle_increment
    ## angle_max = angle_min+angle_increment*len(data.ranges)
    print 'The minimum range (distance) the laser can perceive is: ', data.range_min
    print 'The maximum range (distance) the laser can perceive is: ', data.range_max
    
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()
