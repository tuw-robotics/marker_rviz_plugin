#!/usr/bin/env python

import roslib; roslib.load_manifest( 'marker_rviz_plugin' )
from math import cos, sin, pi
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from marker_msgs.msg import Marker, MarkerDetection, MarkerWithCovariance, MarkerWithCovarianceArray, MarkerWithCovarianceStamped
import rospy
import tf
import tf_conversions

publisher_detection = rospy.Publisher( 'marker_detection', MarkerDetection, queue_size=5 )
publisher_cov = rospy.Publisher( 'marker_with_cov', MarkerWithCovarianceStamped, queue_size=5 )

rospy.init_node( 'test_covariance' )

br = tf.TransformBroadcaster()


rate = rospy.Rate(100)
angle = 0

r = pi/2 # 90 deg
p = pi/3 # 60 deg
y = 0


while not rospy.is_shutdown():
    stamp = rospy.Time.now()

    # MarkerDetection
    marker_detection = MarkerDetection()
    marker_detection.header.frame_id = "/base_link"
    marker_detection.header.stamp = stamp
    marker_detection.type = "bch"

    marker = Marker()
    marker.ids.append(1)
    marker.ids_confidence.append(1)
    marker.pose = Pose()
    marker.pose.position.x = 0.4
    marker.pose.position.y = 0.3
    marker.pose.position.z = 0.5
    ori = marker.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r, p, y + cos(10 * angle))
    #ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(0, 0, 0)

    marker_detection.markers.append(marker)

    publisher_detection.publish( marker_detection )

    # MarkerWithCovarianceStamped
    marker_with_cov_s = MarkerWithCovarianceStamped()
    marker_with_cov_s.header.frame_id = "/base_link"
    marker_with_cov_s.header.stamp = stamp

    marker_with_cov_s.marker = MarkerWithCovariance()
    marker_with_cov_s.marker.marker = marker
    marker_with_cov_s.marker.covariance = [0.9919999999999933, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2741556778080377]

    publisher_cov.publish( marker_with_cov_s )

    # Send base_link transform
    br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z),
                     (ori.x, ori.y, ori.z, ori.w),
                     stamp,
                     "pose",
                     "base_link")

    angle += .0005
    rate.sleep()


'''
---
header: 
  seq: 419
  stamp: 
    secs: 1472036917
    nsecs: 157325029
  frame_id: /base_link
child_frame_id: /odom_msg
pose: 
  pose: 
    position: 
      x: 0.0
      y: 1.98
      z: 0.0
    orientation: 
      x: -0.0
      y: 0.0
      z: 0.99695882639
      w: -0.0779300871451
  covariance: [0.9919999999999933, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2741556778080377]

'''

