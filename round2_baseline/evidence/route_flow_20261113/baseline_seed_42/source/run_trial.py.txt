#!/usr/bin/env python3
"""WSL development harness. Owns and cleans up its ROS launch processes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import fcntl
import hashlib
import shutil
import math
import xml.etree.ElementTree as ET
import re
from trial_processes import stop_processes, wait_for_ports

os.environ['ROS_MASTER_URI']='http://localhost:11321'
os.environ['GAZEBO_MASTER_URI']='http://localhost:11355'

p=argparse.ArgumentParser()
p.add_argument('output')
p.add_argument('--seconds',type=int,default=600)
p.add_argument('--cones',action='store_true')
p.add_argument('--gui',action='store_true',help='Show Gazebo for manual-start performance regression checks')
p.add_argument('--profile-performance',action='store_true',help='Record host thread load and simulation clock')
p.add_argument('--startup-only',action='store_true',help='Development startup/cleanup check; no navigation or scenery randomization')
p.add_argument('--travel-clearance',type=float,help='Development override for ordinary-stage planner clearance')
p.add_argument('--timeout-clock',choices=['wall','sim'],default='wall',help='sim is for untimed local practice only; official timer requires wall')
p.add_argument('--seed',type=int)
p.add_argument('--timer',action='store_true')
p.add_argument('--low-com',action='store_true',help='Experimental base_link COM z=0.02; official assets remain unchanged')
p.add_argument('--audit-cones',action='store_true',help='Development-only displacement observer; forbidden in competition')
p.add_argument('--speed-profile',choices=['stable','fast','sprint'],default='stable')
p.add_argument('--continuous-bends',action='store_true')
p.add_argument('--bend-path',action='store_true')
p.add_argument('--efficient-approach',action='store_true')
p.add_argument('--attitude-protection',action='store_true',help='Restore tilt stops and curvature speed limiting; disabled by default')
p.add_argument('--build-fixed-map',action='store_true',help='Sensor survey of a separate fixed-only scene; not a race run')
p.add_argument('--map-file',type=Path,help='Load a laser-built fixed map and enable AMCL localization')
p.add_argument('--contact-observer',type=Path,help='Development-only native transport observer executable; no engine plugin')
p.add_argument('--debug-pose',nargs=3,type=float)
p.add_argument('--debug-goal',nargs=3,type=float)
p.add_argument('--debug-distance',type=float)
p.add_argument('--debug-next-goal',nargs=3,type=float,action='append',default=[])
p.add_argument('--debug-config')
p.add_argument('--debug-control',action='store_true')
p.add_argument('--debug-tail',action='store_true')
p.add_argument('--debug-stage',default='qr_exit')
p.add_argument('--debug-prefix-until',help='Development-only route prefix; never a full-round result')
args=p.parse_args()
if args.build_fixed_map and (args.cones or args.timer or args.debug_pose or args.debug_prefix_until or args.map_file):
    p.error('Fixed mapping cannot include random cones, a timer or debug route overrides')
if args.map_file and not args.map_file.is_file():p.error('Saved map YAML does not exist')
if args.map_file and args.debug_pose:p.error('Debug spawn needs a matching localization initialization; use a route prefix instead')
if args.contact_observer and (not args.contact_observer.is_file() or not os.access(args.contact_observer,os.X_OK)):
    p.error('Contact observer executable is missing; build it in a persistent WSL directory before starting')
if args.timer and args.timeout_clock!='wall':p.error('Official timer runs require wall-clock timeouts')
if bool(args.debug_pose)!=bool(args.debug_goal):p.error('Both debug pose and goal are required')
lock_file=open('/tmp/round2_baseline_trial.lock','w')
fcntl.flock(lock_file,fcntl.LOCK_EX)
out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
package=Path(__file__).resolve().parents[1]
for directory in ('config','launch'):
    shutil.copytree(package/directory,out/directory)
    for copied in (out/directory).glob('*.launch'):
        copied.rename(copied.with_suffix('.launch.txt'))
sources=sorted(package.joinpath('scripts').glob('*.py'))+sorted(package.joinpath('scripts').glob('*.cpp'))
sources+=sorted(package.joinpath('src').glob('*.cpp'))+[package/'CMakeLists.txt',package/'package.xml']
(out/'source_hashes.json').write_text(json.dumps({str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
(out/'source').mkdir()
for source in sources:
    shutil.copyfile(source,out/'source'/(source.name+'.txt'))
processes=[];files=[]
def launch(command,name,env=None):
    f=(out/(name+'.log')).open('w');files.append(f)
    proc=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    processes.append(proc);return proc
result={'passed':False,'trigger':'development service','events':[], 'seed':args.seed, 'development_audit':args.audit_cones, 'speed_profile':args.speed_profile,'timeout_clock':args.timeout_clock,'simulation_engine':'official ode'}
result['continuous_bends']=args.continuous_bends
result['bend_path']=args.bend_path
result['efficient_approach']=args.efficient_approach
result['attitude_protection']=args.attitude_protection
result['navigation_mode']='premapped_amcl' if args.map_file else 'rolling_odom'
if args.map_file:
    import yaml
    image_path=args.map_file.parent/yaml.safe_load(args.map_file.read_text())['image']
    if not image_path.is_file():raise ValueError('Saved map image does not exist')
    (out/'prior_map').mkdir()
    shutil.copyfile(args.map_file,out/'prior_map/map.yaml')
    shutil.copyfile(image_path,out/'prior_map'/image_path.name)
    result['prior_map_sha256']={args.map_file.name:hashlib.sha256(args.map_file.read_bytes()).hexdigest(),image_path.name:hashlib.sha256(image_path.read_bytes()).hexdigest()}
class StartupComplete(Exception):pass
startup_started=time.monotonic()
try:
    wait_for_ports((11321,11355))
    simulation=package/'launch/simulation.launch'
    if args.low_com:
        import rospkg
        car=Path(rospkg.RosPack().get_path('car3'))
        model=ET.parse(car/'urdf/car3.urdf')
        origin=model.find('./link[@name="base_link"]/inertial/origin')
        assert origin.get('xyz')=='0 0 0.045754'
        origin.set('xyz','0 0 0.02')
        model_path=out/'low_com.urdf';model.write(model_path)
        tree=ET.parse(car/'launch/gazebo.launch')
        param=tree.find('.//param[@name="robot_description"]')
        param.attrib.clear();param.set('name','robot_description');param.set('textfile',str(model_path))
        tree.find('.//arg[@name="world"]').set('default','round2')
        tree.find('.//arg[@name="gui"]').set('default','false')
        os.environ['GAZEBO_MODEL_PATH']=str(car)+':'+str(car/'models')+':'+str(Path.home()/'.gazebo/models')
        simulation=out/'low_com_simulation.launch.txt';tree.write(simulation)
        result['model_variant']='low COM 0.02; user confirmed permitted 2026-11-11'
    if args.debug_pose:
        import rospkg
        car=Path(rospkg.RosPack().get_path('car3'));tree=ET.parse(simulation if args.low_com else car/'launch/gazebo.launch')
        x,y,yaw=args.debug_pose
        tree.find('.//node[@name="spawn_model"]').set('args',f'-param robot_description -urdf -model car3 -x {x} -y {y} -z 0.01 -Y {yaw}')
        tree.find('.//arg[@name="gui"]').set('default','false')
        os.environ['GAZEBO_MODEL_PATH']=str(car)+':'+str(car/'models')+':'+str(Path.home()/'.gazebo/models')
        simulation=out/'debug_simulation.launch.txt';tree.write(simulation)
        result['scope']='DEBUG SINGLE STAGE, NOT A COMPETITION RUN'
    if args.build_fixed_map:
        from fixed_mapping import prepare_scene
        import rospkg
        car=Path(rospkg.RosPack().get_path('car3'))
        result['mapping_scene']=prepare_scene(car/'world/round2.world',out/'fixed_scene.world')
        result['mapping_method']='SLAM Toolbox laser occupancy rasterization with official odometry; scan matching and loop correction disabled after replay validation'
        tree=ET.parse(simulation if args.low_com else car/'launch/gazebo.launch')
        tree.find('.//include/arg[@name="world_name"]').set('value',str(out/'fixed_scene.world'))
        simulation=out/'fixed_mapping_simulation.launch.txt';tree.write(simulation)
        result['scope']='FIXED SCENE LASER SURVEY; not a competition run'
    sim_env=dict(os.environ)
    sim_env['PYTHONUNBUFFERED']='1'
    for key in tuple(sim_env):
        if key.startswith('ROUND2_COLLISION_') or key=='ROUND2_CONTACT_AUDIT':sim_env.pop(key)
    sim=launch(['roslaunch',str(simulation),'gui:='+str(args.gui).lower()],'simulation',env=sim_env)
    time.sleep(2)
    if sim.poll() is not None:raise RuntimeError('Simulation launch failed before ROS startup')
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool
    rospy.init_node('trial_observer',anonymous=True)
    odom_deadline=time.monotonic()+90
    while True:
        try:
            rospy.wait_for_message('/odom',Odometry,timeout=1)
            break
        except rospy.ROSException:
            log=(out/'simulation.log').read_text(errors='replace')
            if re.search(r'\[gazebo-\d+\] process has died',log):raise RuntimeError('Gazebo exited before odometry; see simulation.log')
            if time.monotonic()>odom_deadline:raise
    rospy.wait_for_message('/scan',LaserScan,timeout=30)
    if args.profile_performance:
        from gazebo_msgs.srv import GetPhysicsProperties
        import yaml
        rospy.wait_for_service('/gazebo/get_physics_properties',timeout=10)
        physics=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)()
        (out/'physics_properties.json').write_text(json.dumps(yaml.safe_load(str(physics)),indent=2))
    result['startup_seconds']=time.monotonic()-startup_started
    if args.startup_only:
        result.update(passed=True,scope='STARTUP ONLY; no route test')
        raise StartupComplete()
    control_args=['roslaunch',str(package/'launch/control.launch'),'speed_profile:='+args.speed_profile,'timeout_clock:='+args.timeout_clock,'continuous_bends:='+str(args.continuous_bends).lower()]
    control_args.append('efficient_approach:='+str(args.efficient_approach).lower())
    control_args.append('bend_path:='+str(args.bend_path).lower())
    control_args.append('attitude_protection:='+str(args.attitude_protection).lower())
    if args.map_file:control_args.append('map_file:='+str(args.map_file.resolve()))
    if args.travel_clearance is not None:
        control_args.append('travel_clearance:='+str(args.travel_clearance))
        result['travel_clearance']=args.travel_clearance
    launch(control_args,'control')
    if args.build_fixed_map:
        launch(['roslaunch',str(package/'launch/mapping.launch')],'mapping')
        launch(['rosbag','record','-O',str(out/'mapping_sensors.bag'),'/nav_scan','/odom','/tf','/tf_static','/slam_toolbox/pose','/map'],'mapping_bag')
        from nav_msgs.msg import OccupancyGrid
        rospy.wait_for_message('/map',OccupancyGrid,timeout=45)
    if args.profile_performance:
        launch(['python3',str(package/'scripts/profile_runtime.py'),str(out/'performance.jsonl')],'performance')
    rospy.wait_for_service('/nav_start',timeout=60)
    ready_deadline=time.monotonic()+30
    if not args.debug_pose:
        while not rospy.wait_for_message('/baseline/ready',Bool,timeout=10).data:
            if time.monotonic()>ready_deadline:raise RuntimeError('Preflight not ready')
    time.sleep(2)
    env=dict(os.environ)
    if args.seed is not None:env['CONE_SEED']=str(args.seed)
    scripts=[] if args.build_fixed_map else ['random_place.py']
    if args.cones:scripts.insert(0,'random_cones.py')
    for script in scripts:
        with (out/(script+'.log')).open('w') as f:
            subprocess.run(['rosrun','car3',script],env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=45)
    time.sleep(3)
    if args.audit_cones:
        launch(['python3',str(package/'scripts/audit_cones.py'),str(out/'cone_audit.json')],'cone_audit')
        time.sleep(2)
    if args.contact_observer:
        result['contact_observer_sha256']=hashlib.sha256(args.contact_observer.read_bytes()).hexdigest()
        launch([str(args.contact_observer.resolve()),str(out/'contacts.jsonl')],'contacts')
        time.sleep(2)
    status={};poses=[];localized=[]
    def guard_status(msg):
        value=json.loads(msg.data)
        result['guard_max_trips']=max(result.get('guard_max_trips',0),value.get('trips',0))
        result['guard_last_mode']=value.get('mode')
    rospy.Subscriber('/attitude_guard/status',String,guard_status,queue_size=10)
    def state(msg):
        value=json.loads(msg.data)
        if value!=status:
            result['events'].append(dict(value,wall=time.monotonic()))
            print(value,flush=True)
        status.clear();status.update(value)
    def odom(msg):
        q=msg.pose.pose.orientation
        import tf
        poses.append(dict(t=msg.header.stamp.to_sec(),wall=time.monotonic(),x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,rpy=tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w]),v=[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z]))
    rospy.Subscriber('/baseline/status',String,state)
    rospy.Subscriber('/odom',Odometry,odom,queue_size=10)
    if args.map_file:
        def localized_pose(msg):
            q=msg.pose.pose.orientation
            import tf
            localized.append(dict(t=msg.header.stamp.to_sec(),wall=time.monotonic(),frame=msg.header.frame_id,
                x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,rpy=tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w]),
                covariance=list(msg.pose.covariance)))
        rospy.Subscriber('/localized_odom',Odometry,localized_pose,queue_size=10)
    launch(['python3',str(Path(__file__).with_name('capture.py')),str(out/'snapshots'),str(args.seconds),'--clock',args.timeout_clock],'capture')
    start=time.monotonic()
    sim_start=rospy.get_time();last_clock=sim_start;clock_progress=time.monotonic()
    debug_client=None
    if args.build_fixed_map:
        from fixed_mapping import survey_route
        rospy.set_param('/round2_control/route',survey_route(rospy.get_param('/round2_control/route')))
    if args.debug_prefix_until:
        route=rospy.get_param('/round2_control/route')
        route=route[:next(i+1 for i,row in enumerate(route) if row[0]==args.debug_prefix_until)]
        rospy.set_param('/round2_control/route',route)
        result['scope']='DEBUG ROUTE PREFIX, NOT A COMPETITION RUN'
    if args.debug_control:
        if not args.debug_pose:raise RuntimeError('Debug control requires a debug spawn')
        rospy.set_param('/round2_control/start_position',args.debug_pose[:2])
        route=[[args.debug_stage]+args.debug_goal]
        if args.debug_tail:
            route=rospy.get_param('/round2_control/route')
            route=route[next(i for i,row in enumerate(route) if row[0]=='qr_exit'):]
        rospy.set_param('/round2_control/route',route)
        response=rospy.ServiceProxy('/nav_start',SetBool)(True)
        if not response.success:raise RuntimeError(response.message)
        result['trigger']='debug control service'
    elif args.debug_pose:
        import actionlib
        if args.debug_config:
            import yaml
            from dynamic_reconfigure.client import Client
            settings=yaml.safe_load(Path(args.debug_config).read_text())
            Client('/move_base/TebLocalPlannerROS',timeout=5).update_configuration(settings)
            result['debug_settings']=settings
        if args.debug_distance is not None:
            from dynamic_reconfigure.client import Client
            Client('/move_base/TebLocalPlannerROS',timeout=5).update_configuration({'min_obstacle_dist':args.debug_distance})
            result['debug_obstacle_distance']=args.debug_distance
        from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
        debug_client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        if not debug_client.wait_for_server(rospy.Duration(10)):raise RuntimeError('Navigation unavailable')
        def send_debug(values):
            x,y,yaw=values;goal=MoveBaseGoal();goal.target_pose.header.frame_id='odom';goal.target_pose.header.stamp=rospy.Time.now();goal.target_pose.pose.position.x=x;goal.target_pose.pose.position.y=y;goal.target_pose.pose.orientation.z=math.sin(yaw/2);goal.target_pose.pose.orientation.w=math.cos(yaw/2);debug_client.send_goal(goal)
        send_debug(args.debug_goal)
        result['debug_goals']=[args.debug_goal]+args.debug_next_goal[:]
        result['debug_arrivals']=[]
        result['trigger']='debug action goal'
    elif args.timer:
        result['trigger']='unchanged official Draw widget'
        launch(['python3',str(Path(__file__).with_name('timer_trial.py')),str(out/'timer')],'timer')
    else:
        response=rospy.ServiceProxy('/nav_start',SetBool)(True)
        if not response.success:raise RuntimeError(response.message)
    while (rospy.get_time()-sim_start if args.timeout_clock=='sim' else time.monotonic()-start)<args.seconds:
        current_clock=rospy.get_time()
        if current_clock>last_clock:clock_progress=time.monotonic();last_clock=current_clock
        if time.monotonic()-clock_progress>5:raise RuntimeError('Simulation clock stopped advancing')
        if debug_client and debug_client.get_state() in (2,3,4,5,8,9):
            if debug_client.get_state()==3:
                result['debug_arrivals'].append(dict(t=rospy.Time.now().to_sec(),wall=time.monotonic()))
                if args.debug_next_goal:
                    time.sleep(.6)
                    send_debug(args.debug_next_goal.pop(0))
                    continue
            result['passed']=debug_client.get_state()==3
            result['action_state']=debug_client.get_state();break
        if status.get('state') in ('FAILED','CONTROL_ROUTE_DONE'):
            result['passed']=status['state']=='CONTROL_ROUTE_DONE';break
        if any(v.poll() is not None for v in processes[:2]):raise RuntimeError('Launch process exited')
        time.sleep(1)
    result['elapsed_wall']=time.monotonic()-start
    result['elapsed_sim']=rospy.get_time()-sim_start
    result['final_status']=dict(status)
    rospy.ServiceProxy('/nav_start',SetBool)(False)
    time.sleep(2)
    (out/'poses.json').write_text(json.dumps(poses))
    if args.map_file:(out/'localized_poses.json').write_text(json.dumps(localized))
    if args.build_fixed_map and result['passed']:
        from fixed_mapping import save_map
        result['map_sha256']=save_map(out/'map')
except StartupComplete:
    pass
except Exception as exc:
    import traceback
    traceback.print_exc()
    result['passed']=False
    result['error']=repr(exc)
finally:
    (out/'result.json').write_text(json.dumps(result,indent=2))
    result['cleanup']=stop_processes(processes)
    if result['cleanup']['remaining']:result['passed']=False
    (out/'result.json').write_text(json.dumps(result,indent=2))
    for f in files:f.close()
print(json.dumps(result),flush=True)
raise SystemExit(0 if result['passed'] else 1)
