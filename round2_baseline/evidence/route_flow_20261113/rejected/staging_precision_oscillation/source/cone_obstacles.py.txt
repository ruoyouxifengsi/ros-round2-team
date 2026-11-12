#!/usr/bin/env python3
"""Infer stationary official-grid cones from live laser clusters, not spawn data."""
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

def detect(ranges,angle_min,angle_increment,minimum,maximum,tx,ty,yaw):
    segments=[];group=[];previous=None
    for i,r in enumerate(list(ranges)+[float('inf')]):
        valid=math.isfinite(r) and minimum<r<min(maximum,2.5)
        if not valid or (previous is not None and abs(r-previous)>.12):
            if len(group)>=3:segments.append(group)
            group=[]
        if valid:group.append((r*math.cos(angle_min+i*angle_increment),r*math.sin(angle_min+i*angle_increment)))
        previous=r if valid else None
    found=[]
    for points in segments:
        if not .025<math.dist(points[0],points[-1])<.23:continue
        x=sum(p[0] for p in points)/len(points);y=sum(p[1] for p in points)/len(points)
        d=math.hypot(x,y)
        x*=1+.065/d;y*=1+.065/d
        gx=tx+x*math.cos(yaw)-y*math.sin(yaw);gy=ty+x*math.sin(yaw)+y*math.cos(yaw)
        col=round((gx+2.25)/.5);row=round((1.75-gy)/.5)
        if not (0<=col<=4 and 0<=row<=3) or (row<2 and col>2) or (row,col)==(3,4):continue
        cx=-2.25+.5*col;cy=1.75-.5*row
        if math.hypot(gx-cx,gy-cy)<.10:found.append((cx,cy))
    return found

if __name__=='__main__':
    rospy.init_node('cone_obstacles')
    frame=rospy.get_param('~navigation_frame','odom')
    listener=tf.TransformListener();seen={};confirmed=set()
    pub=rospy.Publisher('/cone_footprints',PointCloud2,queue_size=1)
    def scan(msg):
        try:
            pos,q=listener.lookupTransform(frame,msg.header.frame_id,msg.header.stamp)
        except tf.Exception:return
        roll,pitch,yaw=tf.transformations.euler_from_quaternion(q)
        if max(abs(roll),abs(pitch))>math.radians(.7):return
        for center in set(detect(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,pos[0],pos[1],yaw)):
            seen[center]=seen.get(center,0)+1
            if seen[center]>=3 and center not in confirmed:
                confirmed.add(center);rospy.loginfo('Laser-confirmed cone footprint at %.2f %.2f',*center)
        points=[]
        for cx,cy in confirmed:
            for ix in range(-6,7):
                for iy in range(-6,7):
                    if ix*ix+iy*iy<=36:points.append((cx+ix*.025,cy+iy*.025,.05))
        pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=msg.header.stamp,frame_id=frame),points))
    rospy.Subscriber('/nav_scan',LaserScan,scan,queue_size=1)
    rospy.spin()
