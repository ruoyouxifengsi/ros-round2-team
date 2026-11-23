#!/usr/bin/env python3
"""Rebuild a sensor map from a survey bag, excluding every previous SLAM transform."""
import argparse,json,os,subprocess,time
from pathlib import Path
import rosbag,rospy,yaml
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from fixed_mapping import save_map
from trial_processes import stop_processes,wait_for_ports

p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('output',type=Path)
p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config/slam_toolbox.yaml')
p.add_argument('--rate',type=float,default=2.)
args=p.parse_args()
if args.rate<=0:p.error('Replay rate must be positive')
out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
os.environ['ROS_MASTER_URI']='http://localhost:11322'
filtered=out/'sensor_input.bag'
with rosbag.Bag(str(args.bag)) as source,rosbag.Bag(str(filtered),'w') as target:
    for topic,msg,stamp in source.read_messages(topics=['/nav_scan','/odom','/tf','/tf_static']):
        if topic in ('/tf','/tf_static'):
            msg=TFMessage([tr for tr in msg.transforms if tr.header.frame_id.lstrip('/')!='map'])
            if not msg.transforms:continue
        header=dict(type=msg._type,md5sum=msg._md5sum,message_definition=msg._full_text,latching='1') if topic=='/tf_static' else None
        target.write(topic,msg,stamp,connection_header=header)
processes=[];files=[];result={};latest=[None];transforms=[]
def launch(cmd,name):
    f=(out/(name+'.log')).open('w');files.append(f)
    proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
try:
    wait_for_ports((11322,))
    launch(['roscore','-p','11322'],'core');time.sleep(2)
    rospy.init_node('rebuild_sensor_map',anonymous=True);rospy.set_param('/use_sim_time',True)
    config=yaml.safe_load(args.config.read_text());rospy.set_param('/slam_toolbox',config)
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    launch(['rosrun','slam_toolbox','sync_slam_toolbox_node','__name:=slam_toolbox'],'mapper');time.sleep(2)
    def map_cb(m):latest[0]=m
    def tf_cb(m):
        for tr in m.transforms:
            if tr.header.frame_id.lstrip('/')=='map':
                t=tr.transform.translation;q=tr.transform.rotation
                transforms.append([tr.header.stamp.to_sec(),t.x,t.y,q.x,q.y,q.z,q.w])
    rospy.Subscriber('/map',OccupancyGrid,map_cb,queue_size=1);rospy.Subscriber('/tf',TFMessage,tf_cb,queue_size=100)
    play=launch(['rosbag','play','--clock','--delay=2','-r',str(args.rate),str(filtered)],'replay')
    if play.wait(timeout=300):raise RuntimeError('Replay failed')
    time.sleep(3)
    if latest[0] is None:raise RuntimeError('No reconstructed map')
    result['map_sha256']=save_map(out/'map');result['transforms']=transforms
    result['last_map_stamp']=latest[0].header.stamp.to_sec();result['passed']=True
finally:
    result['cleanup']=stop_processes(processes)
    if result['cleanup']['remaining']:result['passed']=False
    (out/'result.json').write_text(json.dumps(result,indent=2))
    for f in files:f.close()
print(json.dumps({k:v for k,v in result.items() if k!='transforms'}))
