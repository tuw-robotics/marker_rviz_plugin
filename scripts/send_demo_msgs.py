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

axes = 'sxyz' # 'sxyz' or 'rxyz'
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
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(r, p, y + cos(10 * angle), axes)

    marker_detection.markers.append(marker)

    publisher_detection.publish( marker_detection )

    # MarkerWithCovarianceStamped
    marker_with_cov = MarkerWithCovarianceStamped()
    marker_with_cov.header.frame_id = "/base_link"
    marker_with_cov.header.stamp = stamp

    publisher_cov.publish( marker_with_cov )

    # Send base_link transform
    br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z),
                     (ori.x, ori.y, ori.z, ori.w),
                     stamp,
                     "pose",
                     "base_link")

    angle += .0005
    rate.sleep()

