#coding=UTF-8
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

# 初始化节点
rospy.init_node('red_light_green_light')

# 初始化greenlight状态
red_light_twist = Twist() #<2>
green_light_twist = Twist()
green_light_twist.linear.x = 0.5 #<3>

driving_forward = False
# 当前时间
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
#直行状态
  if driving_forward:
    cmd_vel_pub.publish(green_light_twist) #<4>
  else:
    cmd_vel_pub.publish(red_light_twist)
  # BEGIN PART_1
  if rospy.Time.now() > light_change_time: #<5>
    driving_forward = not driving_forward
# 3s改变状态
    light_change_time = rospy.Time.now() + rospy.Duration(3)
  # END PART_1
  rate.sleep() #<6>
# END ALL
