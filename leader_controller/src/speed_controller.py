#! /usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0
y = 0
theta = 0
xo = 0.0
yo = 0.0
theta = 0.0
theta_degree = 0.0
xd = 5
yd = 5
t0 = 0

def Leader_odom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



rospy.init_node("speed_controller")

sub = rospy.Subscriber("/robot_Leader_odom", Odometry, Leader_odom)
pub = rospy.Publisher("/robot_leader/mobile_robot_controller/cmd_vel",Twist, queue_size=1)

speed = Twist()

error_x_follower1_before = 0
error_z_follower1_before = 0

Kp_lin = 0.1 #0.5 60cm,-90 #0.2 20cm,0
Ki_lin = 0.0
Kd_lin = 0.0 #0.2

Kp_ang = 0.05 #0.08 #0.2 60cm,-90 #0.1 20cm,0 #1.1
Ki_ang = 0.0
Kd_ang = 1

proportional_x = 0
integral_x = 0
derivative_x = 0

proportional_z = 0
integral_z = 0
derivative_z = 0
error_x_before = 0
error_z_before = 0


rate = rospy.Rate(4)

setpoint = Point()


while not rospy.is_shutdown():
    t1 = rospy.Time.now().to_sec()
    dt = t1 - t0

    setpoint.x = xd
    setpoint.y = yd
            
    error_x = setpoint.x - x
    error_y = setpoint.y - y
    theta_degree = math.degrees(theta)
    angle_to_setpoint = math.degrees(atan2 (error_y, error_x))
    error_z = theta_degree - angle_to_setpoint
    delta_error_x = error_x - error_x_before
    delta_error_z = (error_z) - error_z_before
            
    proportional_x = error_x
    integral_x += (error_x*dt)
    derivative_x = delta_error_x/dt
    speed_x = Kp_lin*proportional_x + Ki_lin*integral_x + Kd_lin*derivative_x
            
    proportional_z = abs(error_z)
    integral_z += abs(error_z*dt)
    derivative_z = delta_error_z/dt
    speed_z = Kp_ang*proportional_z + Ki_ang*integral_z + Kd_ang*derivative_z

            
    if(abs(error_x) > 0.2 or abs(error_z) > 0.2):
        if error_z > 0.2:
            speed.linear.x = speed_x
            speed.angular.z = speed_z
        elif error_z < -0.2:
            speed.linear.x = speed_x
            speed.angular.z = -speed_z
        else:
            speed.linear.x = speed_x
            speed.angular.z = 0
                    
                
    pub.publish(speed)
    print "--------------MOVE CONDITION-----------------------"
    print "error_x = " + str(error_x)
    print "error_z = " + str(error_z)
    print "setpoint.x = " + str(setpoint.x)
    print "setpoint.y = " + str(setpoint.y)    
    print "speed_x = " + str(speed_x)
    print "speed_z = " + str(speed_z)            
    #print "theta = " + str(theta_degree)
    error_x_before = error_x
    error_z_before = abs(error_z)            
    t1 = t0
    rate.sleep()    
        

