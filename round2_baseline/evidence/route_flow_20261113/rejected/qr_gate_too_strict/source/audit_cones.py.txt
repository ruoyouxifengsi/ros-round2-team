#!/usr/bin/env python3
"""DEVELOPMENT ONLY: read-only ground-truth scoring, never a competition node.

Writes cone displacement to disk. No publishers, services or controller inputs.
Do not launch this observer during a competition (Gazebo data restriction).
"""
import json
import math
import sys
import threading
import time
from pathlib import Path
import rospy
from gazebo_msgs.msg import ModelStates

out=Path(sys.argv[1]); baseline={}; peaks={}; latest={}; count=0
lock=threading.Lock()
rospy.init_node('development_cone_audit',anonymous=True)
def observe(msg):
    global count
    with lock:
        for name,p in zip(msg.name,msg.pose):
            if not name.startswith('construction_cone_'):continue
            point=[p.position.x,p.position.y,p.position.z]
            baseline.setdefault(name,point)
            shift=math.hypot(point[0]-baseline[name][0],point[1]-baseline[name][1])
            if shift>peaks.get(name,{}).get('displacement_m',-1):
                peaks[name]=dict(displacement_m=shift,sim_time=rospy.Time.now().to_sec(),position=point)
            latest[name]=point
        count+=1
rospy.Subscriber('/gazebo/model_states',ModelStates,observe,queue_size=1)
while not rospy.is_shutdown():
    with lock:
        data=dict(scope='DEVELOPMENT ONLY; never consumed by navigation; displacement is not contact count',samples=count,baseline=baseline,latest=latest,peaks=peaks,moved_over_5mm=[n for n,p in peaks.items() if p['displacement_m']>.005])
        temp=out.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2));temp.replace(out)
    time.sleep(.5)
