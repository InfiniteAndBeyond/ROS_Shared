#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
  global g_range_ahead
  g_range_ahead = min(msg.ranges)

g_range_ahead = 1 # anything to start

# 订阅topic
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
# 发布topic控制机器人运动
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  if driving_forward:
    # BEGIN FORWARD  前方0.8米有障碍物
    if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
      driving_forward = False
      state_change_time = rospy.Time.now() + rospy.Duration(5)
    # END FORWARD
  else: # we're not driving_forward
    # BEGIN TURNING 原地旋转
    if rospy.Time.now() > state_change_time:
      driving_forward = True # we're done spinning,  time to go forwards!
      state_change_time = rospy.Time.now() + rospy.Duration(30)
    # END TURNING
  twist = Twist()
#改变运动速度
  if driving_forward:
    twist.linear.x = 1
  else:
#不直行的话改变角速度
    twist.angular.z = 1
  cmd_vel_pub.publish(twist)

  rate.sleep()
# END ALL
