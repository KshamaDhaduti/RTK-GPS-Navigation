#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
#---RTK_GPS - EECE5554-Assignemnet 2---
#---Created by Kshama Dhaduti---

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64
import time
import utm
from gps_driver.msg import gns_msg

if __name__ == '__main__':
    SENSOR_NAME = "gps_labone"
    
    # Initialize the ROS node
    rospy.init_node('ini_gps')
    
    # Get serial port and baudrate parameters from ROS parameter server
    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',57600)
    
    # Get sampling rate parameter from ROS parameter server
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    
    # Initialize the serial port for communication with the GPS sensor
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    # Log debug information
    rospy.logdebug("Using gps sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    # Calculate the number of samples to be taken based on the sampling rate
    sampling_count = int(round(1/(sampling_rate*0.007913)))
    
    # Variable to keep track of sequence number
    i = 1
    
    # Sleep for a short duration to allow the sensor to initialize
    rospy.sleep(0.2)        
    
    # Create a publisher for the gns_msg message
    pub = rospy.Publisher('full_gps', gns_msg, queue_size=10)
    
    # Increment sequence number
    i =  i+1
    
    # Log debug information
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing longitude and latitude.")
    
    # Create an instance of the gns_msg message
    msg = gns_msg()
   
    try:
        while not rospy.is_shutdown():
            # Set sequence number in message header
            msg.header.seq = i
            
            # Read a line of data from the serial port
            line = port.readline()
            
            if line == '':
                # If no data is received, log a warning
                rospy.logwarn("gngga: No data")
            else:
                if line.startswith(b'$GNGGA') :
                    # If the line starts with "$GNGGA", it contains GPS data
                    
                    # Split the line into comma-separated values
                    s = line.split(b",")
                    
                    # Extract latitude, longitude, direction, fix type, UTC time, and altitude from the data
                    lat = s[2].decode('utf-8')
                    lon = s[4].decode('utf-8')
                    lat_dir = s[3].decode('utf-8')
                    lon_dir = s[5].decode('utf-8')
                    gns_fix = int(s[6].decode('utf-8'))
                    utc_time = s[1].decode('utf-8')
                    alt = float(s[9].decode('utf-8'))
                     
                    # Convert latitude and longitude values to decimal degrees
                    degrees_lat = int(float(lat)/100)
                    minutes_lat = float(lat)-(degrees_lat*100)
                    degrees_lon = int(float(lon)/100)
                    minutes_lon = float(lon)-(degrees_lon*100)
                    dd_lat = float(degrees_lat) + float(minutes_lat)/60
                    dd_lon = float(degrees_lon) + float(minutes_lon)/60 
                    
                    # Adjust sign of latitude and longitude based on direction
                    if lon_dir == 'W':
                        dd_lon *= -1
                    if lat_dir == 'S':
                        dd_lat *= -1
                    
                    # Print the latitude and longitude in decimal degrees
                    print("\n"+str(dd_lat)+" "+str(dd_lon))
    
                    # Convert latitude and longitude to UTM coordinates
                    utm_data3 = utm.from_latlon(dd_lat, dd_lon)
                    print(utm_data3)
                    
                    # Extract easting, northing, zone number, and zone letter from UTM coordinates
                    easting = utm_data3[0]
                    northing = utm_data3[1]
                    zone_float = utm_data3[2]
                    zone_letter = utm_data3[3]
                    
                    # Fill in the message fields with the GPS data
                    msg.header.stamp = rospy.get_rostime()
                    msg.header.frame_id = "GPS_Data"
                    msg.latitude = dd_lat
                    msg.longitude = dd_lon
                    msg.altitude = alt
                    msg.gps_quality = gns_fix
                    msg.utm_easting = easting
                    msg.utm_northing = northing
                    msg.zone_number = zone_float
                    msg.zone_letter = zone_letter
                    
                    # Publish the message
                    pub.publish(msg)
                
    except rospy.ROSInterruptException:
        # Close the serial port if ROS is interrupted
        port.close()
