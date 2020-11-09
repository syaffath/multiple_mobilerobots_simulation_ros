#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

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

rate = rospy.Rate(4)

setpoint = Point()
setpoint.x = 10
setpoint.y = 0

while not rospy.is_shutdown():
    inc_x = setpoint.x - x
    inc_y = setpoint.y - y

    angle_to_setpoint = atan2 (inc_y, inc_x)
    delta_teta = angle_to_setpoint - theta

  while(pose_x < D_sawtooth)/*for(pose_x = init_pose_x; pose_x = D_sawtooth; pose_x++)*/{
    if(fmod(pose_x, 20) == 0){
      if (pose_teta < D_angle){
        do {
          spin_left()
        } while(pose_teta <= D_angle)
      }
      else{
        do {
          feedforward()
        } while(pose_x < pose_x+10)
      }
      delayMicroseconds(100)
    }
    else{
      delayMicroseconds(100)
      if(pose_teta > -(D_angle)){
        do {
          spin_right()
        } while(pose_teta >= -(D_angle))
      }

      else{
        do {
          feedforward()
        } while(pose_x <pose_x+10)
      }
    }
  }





    pub.publish(speed)
    rate.sleep()


