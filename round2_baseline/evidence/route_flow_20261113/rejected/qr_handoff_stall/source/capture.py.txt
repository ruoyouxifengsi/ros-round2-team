#!/usr/bin/env python3
"""Read-only sensor/navigation snapshots; never consumes Gazebo truth topics."""
import json
import sys
import time
from pathlib import Path
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

rospy.init_node('baseline_capture', anonymous=True)
listener = tf.TransformListener()
latest = {}

def infeasible(msg):
    if msg.ns=='InfeasibleRobotPose/footprint':
        q=msg.pose.orientation
        latest['infeasible']=dict(t=msg.header.stamp.to_sec(),position=[msg.pose.position.x,msg.pose.position.y],yaw=tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2])
rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_markers',Marker,infeasible,queue_size=100)
def save(key, msg):
    latest[key] = msg
for key, topic, typ in [('odom','/odom',Odometry),('scan','/scan',LaserScan),('local','/move_base/local_costmap/costmap',OccupancyGrid),('global','/move_base/global_costmap/costmap',OccupancyGrid),('plan','/move_base/NavfnROS/plan',RosPath),('status','/baseline/status',String),('guard','/attitude_guard/status',String),('nav_cmd','/nav_cmd_vel',Twist),('cmd','/cmd_vel',Twist)]:
    rospy.Subscriber(topic,typ,lambda msg,k=key:save(k,msg),queue_size=1)
out = Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=True)
clock = rospy.get_time if '--clock' in sys.argv and sys.argv[sys.argv.index('--clock')+1]=='sim' else time.monotonic
end = clock()+float(sys.argv[2] if len(sys.argv)>2 else 100)
i=0
with (out/'trace.jsonl').open('w') as f:
    while clock()<end and not rospy.is_shutdown():
        time.sleep(1)
        if 'odom' not in latest: continue
        o=latest['odom']; p=o.pose.pose; q=p.orientation
        row=dict(t=o.header.stamp.to_sec(),position=[p.position.x,p.position.y],rpy=tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w]),velocity=[o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.angular.z])
        if 'status' in latest: row['status']=json.loads(latest['status'].data)
        if 'guard' in latest: row['guard']=json.loads(latest['guard'].data)
        if 'infeasible' in latest: row['infeasible']=latest['infeasible']
        for key in ('nav_cmd','cmd'):
            if key in latest:
                v=latest[key];row[key]=[v.linear.x,v.linear.y,v.angular.z]
        f.write(json.dumps(row)+'\n'); f.flush()
        if i%5==0:
            for key in ('local','global'):
                if key in latest:
                    m=latest[key];row[key]=dict(resolution=m.info.resolution,width=m.info.width,height=m.info.height,origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data))
            if 'scan' in latest:
                s=latest['scan']
                try:
                    pos,rot=listener.lookupTransform('odom',s.header.frame_id,s.header.stamp)
                    row['scan']=dict(position=pos,rotation=rot,angle_min=s.angle_min,angle_increment=s.angle_increment,ranges=list(s.ranges))
                except tf.Exception: pass
            if 'plan' in latest: row['plan']=[[v.pose.position.x,v.pose.position.y] for v in latest['plan'].poses]
            (out/('snapshot_%03d.json'%i)).write_text(json.dumps(row))
        i+=1
