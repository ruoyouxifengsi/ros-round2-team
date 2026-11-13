"""Development-only tail comparison; Gazebo truth only resets/observes the scene."""
import argparse,json,math,subprocess,sys,time,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties,GetModelState,SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool
from trial_processes import stop_processes
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--candidate',action='store_true');p.add_argument('--finish-only',action='store_true');p.add_argument('--full',action='store_true');p.add_argument('--prefix',action='store_true');args=p.parse_args()
out=Path(__file__).parent/args.name;out.mkdir(exist_ok=False)
package=Path(__file__).resolve().parents[2]
for name in ('control.py','route_flow.py','finish_region.py','attitude_guard.py','bend_path.py'):
    if (package/'scripts'/name).exists():shutil.copyfile(package/'scripts'/name,out/(name+'.txt'))
shutil.copyfile(package/'config/route_flow.yaml',out/'route_flow.yaml.txt')
shutil.copyfile(package/'launch/control.launch',out/'control.launch.txt')
phases=[];events=[];poses=[];guards=[];commands=[];ready=[False];processes=[];files=[];maps=[]
rospy.init_node('region_tail_compare',anonymous=True)
rospy.Subscriber('/baseline/probe',String,lambda m:phases.append(json.loads(m.data)))
rospy.Subscriber('/baseline/status',String,lambda m:events.append(dict(json.loads(m.data),wall=time.monotonic())))
rospy.Subscriber('/baseline/ready',Bool,lambda m:ready.__setitem__(0,m.data))
rospy.Subscriber('/attitude_guard/status',String,lambda m:guards.append(json.loads(m.data)))
rospy.Subscriber('/cmd_vel',Twist,lambda m:commands.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z]))
rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,lambda m:maps.__setitem__(slice(None),[m]),queue_size=1)
def observe(m):
    q=m.pose.pose.orientation;p=m.pose.pose.position;v=m.twist.twist
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    poses.append(dict(t=m.header.stamp.to_sec(),wall=time.monotonic(),x=p.x,y=p.y,yaw=yaw,v=[v.linear.x,v.linear.y,v.angular.z]))
rospy.Subscriber('/odom',Odometry,observe,queue_size=10)
get_model=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
cones=[name for name in rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)().model_names if 'cone' in name.lower()]
def cone_positions():
    values={}
    for name in cones:
        p=get_model(name,'world').pose.position;values[name]=[p.x,p.y,p.z]
    return values
def reset(x,y,yaw):
    s=ModelState(model_name='car3',reference_frame='world');s.pose.position.x=x;s.pose.position.y=y;s.pose.position.z=.01
    s.pose.orientation.z=math.sin(yaw/2);s.pose.orientation.w=math.cos(yaw/2)
    assert rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(s).success
def launch(command,name):
    f=(out/(name+'.log')).open('w');files.append(f)
    processes.append(subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
result=dict(scope='DEVELOPMENT TAIL ONLY: QR departure through finish; no recognition or official timer',candidate=args.candidate,passed=False,cones_before=cone_positions())
try:
    reset(-.75,1.75,math.pi/2) if args.full else reset(.2,.25,0.)
    time.sleep(2)
    launch(['roslaunch',str(Path(__file__).parent/'instrumented.launch'),'speed_profile:=sprint','continuous_bends:=true',
            'efficient_approach:=true','route_flow:=true','attitude_protection:=false','manual_supervision:=true','timeout_clock:=wall'],'control')
    deadline=time.monotonic()+120
    while not rospy.has_param('/round2_control/flow_settings'):
        if time.monotonic()>deadline:raise RuntimeError('Parameters not loaded')
        time.sleep(.2)
    # Wait for the new launch to overwrite the previous route/parameters.
    while not any(e['state']=='IDLE' for e in events):
        if time.monotonic()>deadline:raise RuntimeError('Controller not initialized')
        time.sleep(.2)
    rospy.set_param('/round2_control/start_position',[-.75,1.75] if args.full else [.2,.25])
    cfg=rospy.get_param('/round2_control/flow_settings');cfg['continuous_regions']=args.candidate
    rospy.set_param('/round2_control/flow_settings',cfg)
    route=rospy.get_param('/round2_control/route');route=route if args.full else route[next(i for i,r in enumerate(route) if r[0]=='qr_exit'):]
    route=route[:next(i for i,r in enumerate(route) if r[0]=='qr')+1] if args.prefix else route
    rospy.set_param('/round2_control/route',route)
    while not ready[0]:
        if time.monotonic()>deadline:raise RuntimeError('Start preflight not ready')
        time.sleep(.2)
    launch(['/home/keyaoersi/round2_team_tools/contact_wall_audit',str(out/'contacts.jsonl')],'observer')
    observer_deadline=time.monotonic()+30
    while not (out/'contacts.jsonl').exists() or not (out/'contacts.jsonl').stat().st_size:
        if time.monotonic()>observer_deadline:raise RuntimeError('Contact observer not ready')
        time.sleep(.2)
    events.clear();start=time.monotonic()
    if args.full:result['scope']='FIXED START TO QR' if args.prefix else 'FULL CONTROL ROUTE; no recognition or official timer'
    response=rospy.ServiceProxy('/nav_start',SetBool)(True);assert response.success,response.message
    print('STARTED '+args.name,flush=True)
    while not any(e['state'] in ('FAILED','CONTROL_ROUTE_DONE','CANCELLED') for e in events):
        if time.monotonic()-start>800:raise RuntimeError('Development watchdog: tail exceeded 420 wall seconds')
        time.sleep(.2)
    time.sleep(1)
    moving=next(e for e in events if e['state']=='MOVING');end=events[-1]
    result.update(events=events[:],route_done=end['state']=='CONTROL_ROUTE_DONE',sim_seconds=end['stamp']-moving['stamp'],wall_seconds=end['wall']-moving['wall'],
        final_pose=poses[-1],max_tilt_deg=max(g['tilt_deg'] for g in guards if g.get('tilt_deg') is not None),
        cones_after=cone_positions(),max_lateral_command=max(abs(c[2]) for c in commands))
except Exception as exc:result['error']=repr(exc)
finally:
    if maps:
        grid=maps[-1]
        (out/'last_costmap.json').write_text(json.dumps(dict(resolution=grid.info.resolution,width=grid.info.width,height=grid.info.height,
            x=grid.info.origin.position.x,y=grid.info.origin.position.y,data=list(grid.data))))
    try:rospy.ServiceProxy('/nav_start',SetBool)(False)
    except Exception:pass
    result['cleanup']=stop_processes(processes)
    if rospy.has_param('/round2_control/start_position'):rospy.delete_param('/round2_control/start_position')
    for f in files:f.close()
    if (out/'contacts.jsonl').exists():
        rows=[json.loads(s) for s in (out/'contacts.jsonl').read_text().splitlines()]
        summaries=[r for r in rows if r['type']=='summary']
        if summaries:result['contacts']=summaries[-1]
    result['passed']=bool(result.get('route_done') and result.get('contacts',{}).get('messages',0)>0 and
        result['contacts']['wall_contact_events']==0 and result['contacts']['cone_contact_events']==0 and result.get('max_tilt_deg',99)<3)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    (out/'trace.json').write_text(json.dumps(dict(events=events,poses=poses,guards=guards,commands=commands,phases=phases)))
    reset(-.75,1.75,math.pi/2)
    print(json.dumps(result,indent=2),flush=True)
