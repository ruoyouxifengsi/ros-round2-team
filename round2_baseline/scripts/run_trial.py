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

lock_file=open('/tmp/round2_baseline_trial.lock','w')
fcntl.flock(lock_file,fcntl.LOCK_EX)
os.environ['ROS_MASTER_URI']='http://localhost:11321'
os.environ['GAZEBO_MASTER_URI']='http://localhost:11355'

p=argparse.ArgumentParser()
p.add_argument('output')
p.add_argument('--seconds',type=int,default=600)
p.add_argument('--cones',action='store_true')
p.add_argument('--gui',action='store_true',help='Show Gazebo for manual-start performance regression checks')
p.add_argument('--profile-performance',action='store_true',help='Record host thread load and simulation clock')
p.add_argument('--experimental-collision-library',type=Path,help='Development-only Gazebo collision adapter; not approved for competition')
p.add_argument('--collision-threads',type=int,default=4)
p.add_argument('--verify-collisions',action='store_true')
p.add_argument('--collision-verify-stride',type=int,default=1,help='Verify every N physics updates; default verifies all')
p.add_argument('--seed',type=int)
p.add_argument('--timer',action='store_true')
p.add_argument('--low-com',action='store_true',help='Experimental base_link COM z=0.02; official assets remain unchanged')
p.add_argument('--audit-cones',action='store_true',help='Development-only displacement observer; forbidden in competition')
p.add_argument('--speed-profile',choices=['stable','fast'],default='stable')
p.add_argument('--debug-pose',nargs=3,type=float)
p.add_argument('--debug-goal',nargs=3,type=float)
p.add_argument('--debug-distance',type=float)
p.add_argument('--debug-next-goal',nargs=3,type=float,action='append',default=[])
p.add_argument('--debug-config')
p.add_argument('--debug-control',action='store_true')
p.add_argument('--debug-tail',action='store_true')
p.add_argument('--debug-stage',default='qr_exit')
args=p.parse_args()
if not 1<=args.collision_threads<=20:p.error('Collision threads must be between 1 and 20')
if args.collision_verify_stride<1:p.error('Collision verification stride must be positive')
if args.verify_collisions and not args.experimental_collision_library:p.error('Collision verification requires the experimental library')
if bool(args.debug_pose)!=bool(args.debug_goal):p.error('Both debug pose and goal are required')
out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
package=Path(__file__).resolve().parents[1]
for directory in ('config','launch'):
    shutil.copytree(package/directory,out/directory)
    for copied in (out/directory).glob('*.launch'):
        copied.rename(copied.with_suffix('.launch.txt'))
(out/'source_hashes.json').write_text(json.dumps({str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest() for p in package.joinpath('scripts').glob('*.py')},indent=2))
(out/'source').mkdir()
for source in package.joinpath('scripts').glob('*.py'):
    shutil.copyfile(source,out/'source'/(source.name+'.txt'))
processes=[];files=[]
def launch(command,name,env=None):
    f=(out/(name+'.log')).open('w');files.append(f)
    proc=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    processes.append(proc);return proc
result={'passed':False,'trigger':'development service','events':[], 'seed':args.seed, 'development_audit':args.audit_cones, 'speed_profile':args.speed_profile}
try:
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
    sim_env=dict(os.environ)
    if args.experimental_collision_library:
        library=args.experimental_collision_library.resolve(strict=True)
        tree=ET.parse(simulation)
        include=tree.find('.//include[@file="$(find gazebo_ros)/launch/empty_world.launch"]')
        if include is None:raise RuntimeError('Experimental adapter requires --low-com')
        ET.SubElement(include,'arg',name='extra_gazebo_args',value='-s '+str(library))
        ET.SubElement(include,'arg',name='physics',value='round2_parallel_ode')
        simulation=out/'experimental_simulation.launch.txt';tree.write(simulation)
        sim_env['ROUND2_COLLISION_THREADS']=str(args.collision_threads)
        sim_env['OMP_WAIT_POLICY']='PASSIVE'
        sim_env['ROUND2_COLLISION_VERIFY_STRIDE']=str(args.collision_verify_stride)
        sim_env.pop('ROUND2_COLLISION_VERIFY',None)
        if args.verify_collisions:sim_env['ROUND2_COLLISION_VERIFY']='1'
        result['experimental_collision']=dict(library=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest(),threads=args.collision_threads,verify=args.verify_collisions,verify_stride=args.collision_verify_stride)
        engine_source=library.with_name('adapter.cc')
        if engine_source.exists():
            shutil.copyfile(engine_source,out/'engine_source.cc.txt')
            result['experimental_collision']['source_sha256']=hashlib.sha256(engine_source.read_bytes()).hexdigest()
    sim=launch(['roslaunch',str(simulation),'gui:='+str(args.gui).lower()],'simulation',env=sim_env)
    time.sleep(2)
    if sim.poll() is not None:raise RuntimeError('Simulation launch failed before ROS startup')
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool
    rospy.init_node('trial_observer',anonymous=True)
    rospy.wait_for_message('/odom',Odometry,timeout=90)
    rospy.wait_for_message('/scan',LaserScan,timeout=30)
    if args.profile_performance:
        from gazebo_msgs.srv import GetPhysicsProperties
        import yaml
        rospy.wait_for_service('/gazebo/get_physics_properties',timeout=10)
        physics=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)()
        (out/'physics_properties.json').write_text(json.dumps(yaml.safe_load(str(physics)),indent=2))
    launch(['roslaunch',str(package/'launch/control.launch'),'speed_profile:='+args.speed_profile],'control')
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
    scripts=['random_place.py']
    if args.cones:scripts.insert(0,'random_cones.py')
    for script in scripts:
        with (out/(script+'.log')).open('w') as f:
            subprocess.run(['rosrun','car3',script],env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=45)
    time.sleep(3)
    if args.audit_cones:
        launch(['python3',str(package/'scripts/audit_cones.py'),str(out/'cone_audit.json')],'cone_audit')
        time.sleep(2)
    status={};poses=[]
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
    launch(['python3',str(Path(__file__).with_name('capture.py')),str(out/'snapshots'),str(args.seconds)],'capture')
    start=time.monotonic()
    debug_client=None
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
    while time.monotonic()-start<args.seconds:
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
    result['final_status']=dict(status)
    rospy.ServiceProxy('/nav_start',SetBool)(False)
    time.sleep(2)
    (out/'poses.json').write_text(json.dumps(poses))
except Exception as exc:
    result['error']=repr(exc)
finally:
    (out/'result.json').write_text(json.dumps(result,indent=2))
    for proc in reversed(processes):
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    for proc in reversed(processes):
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL)
                proc.wait(timeout=3)
    for f in files:f.close()
print(json.dumps(result),flush=True)
raise SystemExit(0 if result['passed'] else 1)
