#!/usr/bin/env python3

from sensor_msgs.msg import NavSatFix
import math
import rospy
import csv
import pymap3d as pm
import pandas as pd
from std_msgs.msg import Float32

class Robot:
    def __init__(self):
        rospy.Subscriber('/gps/fix', NavSatFix, self.callback)
        
        self.left_pub = rospy.Publisher('Left_vel', Float32, queue_size=10)
        self.right_pub = rospy.Publisher('Right_vel', Float32, queue_size=10)

        
    def callback(self,data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

    def get_enu(self):
        self.enu = pm.geodetic2enu(self.latitude,self.longitude,self.altitude, 58.3428685594, 25.5692475361, 91.357)
        return self.enu
    
    def get_nearest_point(self, markers):
        Iout = 0
        x , y, z = self.get_enu()    
        Inx = markers[0][0] - x 
        Iny = markers[0][1] - y
        In = math.sqrt(Inx**2+Iny**2)
        lastIn = In
        
        for p in range(len(markers)):
            Inx = markers[p][0] - x 
            Iny = markers[p][1] - y
            In = math.sqrt(Inx**2+Iny**2)
            
            if In < lastIn:
                lastIn = In
                Iout = p
        return Iout

    def send_vel(self, left, right):
        left_msg = Float32()
        left_msg.data = left
        self.left_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = right
        self.right_pub.publish(right_msg)
    

#Heading 
def calculate_heading(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    bearing = math.atan2(delta_y, delta_x)
    bearing = math.degrees(bearing)
    heading = (bearing + 360) % 360
    return heading

#Error angle
def calculate_error_angle(current_heading, target_bearing):
    error_angle = (target_bearing - current_heading + 360) % 360
    if error_angle > 180:
       error_angle -= 360
    return error_angle

def calculate_speed(x1, y1, x2, y2, dt):                            #kiirus km/h
    speed = (math.sqrt((x2 - x1)**2 + (y2 - y1)**2) / dt) * 3.6
    return speed

points = [] 
with open('/home/roisten/catkin_ws/recordings/13-06-2024-13-04.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, quotechar='|')
        for row in spamreader:                         
            x = float(row[0])                                        
            y = float(row[1])
            z = float(row[2])
            points.append([x, y, z])

robot = Robot()
rospy.init_node('Robot', anonymous=True)

#salvesta roboti asukoht käivitamisel
prevX, prevY, prevZ = robot.get_enu()

lasttime = rospy.get_time() - 1e-6
i = 0 #punkti index
i = robot.get_nearest_point(points)
#PID 
Kp= 0.275 #0.275
Ki = 0.00 #0.0
Kd = 0.0000015 #0.00000

integral = 0.0
previous_error = 0.0

while not rospy.is_shutdown():
    try:
        #current_time = rospy.get_time()
        #dt = (current_time-lasttime)
        #lasttime = current_time
        x, y, z = robot.get_enu()

        #PID#############################
        if x != prevX or y != prevY:
            current_time =rospy.get_time()
            dt = (current_time-lasttime)
            lasttime = current_time 
            heading =calculate_heading(prevX, prevY, x, y)
            marker_heading = calculate_heading(x, y ,points[i][0],points[i][1])
            error = calculate_error_angle(heading,marker_heading)
            
            speed = calculate_speed(prevX, prevY, x, y, dt)
            prevX, prevY, prevZ = x, y, z
        
            base_speed = 75.0  #Algkiirus
            #PID 
            integral += error * dt
            integral = max(min(integral, 100), -100)
            derivative = (error - previous_error) / dt
            previous_error = error
            
            control_signal = Kp * error + Ki * integral + Kd * derivative
            control_signal = max(min(control_signal, 100), -100)
            
            left_speed = base_speed - control_signal
            right_speed = base_speed + control_signal
    
            max_speed = 90.00
            min_speed = 1.0
            left_speed = max(min(left_speed, max_speed), min_speed)
            right_speed = max(min(right_speed, max_speed), min_speed)
            if speed > 1.5:
                left_speed, right_speed = left_speed*0.8, right_speed*0.8
          
   
            robot.send_vel(left_speed, right_speed)
            print("punkti Index:",i)

            #############
            #Threshold
            Inx = points[i][0] - x 
            Iny = points[i][1] - y
            In = math.sqrt(Inx**2+Iny**2)

            print("kaugus meetrites", In)
            if In<=0.5:
                    i = i + 25

    except KeyboardInterrupt:
        print("Shutting down")
        break

