#coding=UTF-8
#!/usr/bin/env python
# BEGIN ALL
import rospy
from sensor_msgs.msg import LaserScan

# 获取车正前方的障碍物距离
def scan_callback(msg):
  range_ahead = msg.ranges[len(msg.ranges)/2]
  print "range ahead: %0.1f" % range_ahead

# 初始化节点
rospy.init_node('range_ahead')
# 订阅激光topic
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()
