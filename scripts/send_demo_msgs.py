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

rospy.init_node( 'test_marker_covariance' )

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
    marker.pose.position.x = 4.5
    marker.pose.position.y = 4.0
    marker.pose.position.z = 0.0
    ori = marker.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r, p, y + cos(10 * angle))
    #ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(0, 0, 0)

    marker_detection.markers.append(marker)

    publisher_detection.publish( marker_detection )

    # MarkerWithCovarianceStamped
    markerc = Marker()
    markerc.ids.append(1)
    markerc.ids_confidence.append(1)
    markerc.pose = Pose()
    markerc.pose.position.x = 6.03287422834
    markerc.pose.position.y = 4.93379116194
    markerc.pose.position.z = 0.0
    ori = markerc.pose.orientation
    ori.x = 0.0
    ori.y = 0.0
    ori.z = 0.16781934864
    ori.w = 0.985817765219

    marker_with_cov_s = MarkerWithCovarianceStamped()
    marker_with_cov_s.header.frame_id = "/base_link"
    marker_with_cov_s.header.stamp = stamp

    marker_with_cov_s.marker = MarkerWithCovariance()
    marker_with_cov_s.marker.marker = markerc
    marker_with_cov_s.marker.covariance = [0.011241161991539792, -0.011615365449532622, 0.0, 0.0, 0.0, -0.002759911553295745, -0.011615365449532622, 0.01903682230368378, 0.0, 0.0, 0.0, 0.00390837573140233, 0.0, 0.0, 0.00203078279, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00203078279, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00203078279, 0.0, -0.002759911553295745, 0.00390837573140233, 0.0, 0.0, 0.0, 0.01827704518]

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


'''
header: 
  seq: 2917
  stamp: 
    secs: 292
    nsecs: 800000000
  frame_id: map
markers: 
  - 
    marker: 
      ids: [1]
      ids_confidence: [1.0]
      pose: 
        position: 
          x: 2.99259999116
          y: 1.99296511443
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.642837187793
          w: 0.766002839414
    covariance: [5.145585936373751e-06, 3.5465074498143265e-06, 0.0, 0.0, 0.0, 7.763297496781882e-09, 3.5465074498143265e-06, 2.7218408354024943e-06, 0.0, 0.0, 0.0, 6.08589337627961e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.763297496781882e-09, 6.08589337627961e-09, 0.0, 0.0, 0.0, 2.1112777137113703e-08]
  - 
    marker: 
      ids: [2]
      ids_confidence: [1.0]
      pose: 
        position: 
          x: -2.97632781247
          y: 2.00375800444
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: -0.000658593183246
          w: 0.999999783127
    covariance: [9.593501938212036e-05, -2.22641412848119e-05, 0.0, 0.0, 0.0, 4.916769263738688e-07, -2.22641412848119e-05, 5.104665830981123e-05, 0.0, 0.0, 0.0, -2.248132132389405e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.916769263738688e-07, -2.248132132389405e-06, 0.0, 0.0, 0.0, 1.7013588091485252e-06]
  - 
    marker: 
      ids: [3]
      ids_confidence: [1.0]
      pose: 
        position: 
          x: 0.0140226204194
          y: -1.00262787094
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: -0.000983026938579
          w: 0.999999516829
    covariance: [0.00013515838629420734, 9.920123763170366e-07, 0.0, 0.0, 0.0, 6.020879714999367e-06, 9.920123763170366e-07, 1.3787290943207972e-05, 0.0, 0.0, 0.0, 3.510932706789802e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.020879714999367e-06, 3.510932706789802e-06, 0.0, 0.0, 0.0, 2.10976565686069e-06]
  - 
    marker: 
      ids: [4]
      ids_confidence: [1.0]
      pose: 
        position: 
          x: 6.03287422834
          y: 4.93379116194
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.16781934864
          w: 0.985817765219
    covariance: [0.011241161991539792, -0.011615365449532622, 0.0, 0.0, 0.0, -0.002759911553295745, -0.011615365449532622, 0.01903682230368378, 0.0, 0.0, 0.0, 0.00390837573140233, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.002759911553295745, 0.00390837573140233, 0.0, 0.0, 0.0, 0.0008458541363815106]

[
0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.030461741978670857, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.2741556778080377
]
'''


