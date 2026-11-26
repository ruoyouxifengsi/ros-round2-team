#!/usr/bin/env python3
"""Expose map-corrected pose with raw body velocity; never invent a missing transform."""
import copy
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def corrected_pose(position,quaternion,translation,rotation):
    yaw=tf.transformations.euler_from_quaternion(rotation)[2];c,s=math.cos(yaw),math.sin(yaw)
    return (translation[0]+c*position[0]-s*position[1],translation[1]+s*position[0]+c*position[1],position[2]),tf.transformations.quaternion_multiply(rotation,quaternion)

if __name__=='__main__':
    rospy.init_node('localized_odometry')
    listener=tf.TransformListener();estimate=[None]
    publisher=rospy.Publisher('/localized_odom',Odometry,queue_size=5)
    def pose(message):estimate[0]=message
    def odom(message):
        if estimate[0] is None:return
        cov=estimate[0].pose.covariance
        if not all(math.isfinite(v) and v>=0 for v in (cov[0],cov[7],cov[35])):return
        if max(cov[0],cov[7])>.01 or cov[35]>.03:return
        try:
            latest=listener.getLatestCommonTime('map',message.header.frame_id)
            if not -.5<= (rospy.Time.now()-latest).to_sec() <=.5:return
            translation,rotation=listener.lookupTransform('map',message.header.frame_id,message.header.stamp)
        except tf.Exception:return
        value=copy.deepcopy(message);p=message.pose.pose.position;q=message.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,*translation,*rotation)):return
        xyz,quat=corrected_pose((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),translation,rotation)
        value.header.frame_id='map'
        value.pose.pose.position.x,value.pose.pose.position.y,value.pose.pose.position.z=xyz
        value.pose.pose.orientation.x,value.pose.pose.orientation.y,value.pose.pose.orientation.z,value.pose.pose.orientation.w=quat
        value.pose.covariance=list(cov)
        publisher.publish(value)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,pose,queue_size=1)
    rospy.Subscriber('/odom',Odometry,odom,queue_size=5)
    rospy.spin()
