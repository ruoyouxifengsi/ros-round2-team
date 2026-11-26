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
from trial_processes import stop_processes, wait_for_gazebo_window, wait_for_ports

os.environ['ROS_MASTER_URI']='http://localhost:11321'
os.environ['GAZEBO_MASTER_URI']='http://localhost:11355'

p=argparse.ArgumentParser()
p.add_argument('output')
p.add_argument('--seconds',type=int,default=600)
p.add_argument('--cones',action='store_true')
p.add_argument('--gui',action='store_true',help='Show Gazebo for manual-start performance regression checks')
p.add_argument('--profile-performance',action='store_true',help='Record host thread load and simulation clock')
p.add_argument('--stationary-profile-urdf', type=Path,
               help='Profile a separate URDF at the stationary hybrid spawn, without issuing motion commands')
p.add_argument('--gui-start-hold', type=float, default=8.,
               help='Seconds to hold the stationary spawn under the hybrid trial GUI')
p.add_argument('--startup-only',action='store_true',help='Development startup/cleanup check; no navigation or scenery randomization')
p.add_argument('--timeout-clock',choices=['wall','sim'],default='wall',help='sim is for untimed local practice only; official timer requires wall')
p.add_argument('--seed',type=int)
p.add_argument('--layout-seed',type=int,help='Development only: reproduce the official QR and block draw')
p.add_argument('--timer',action='store_true')
p.add_argument('--low-com',action='store_true',help='Experimental base_link COM z=0.02; official assets remain unchanged')
p.add_argument('--audit-cones',action='store_true',help='Development-only displacement observer; forbidden in competition')
p.add_argument('--speed-profile',choices=['stable','fast','sprint'],default='stable')
p.add_argument('--bend-radius', type=float, help='Override the fixed bend radius for this trial (metres)')
p.add_argument('--debug-bends-only', action='store_true', help='Measure through bend8 with the normal QR approach tangent, then stop')
p.add_argument('--fixed-qr-trial', action='store_true',
               help='Drive from the start to the QR box using fixed paths, without move_base or braking checks')
p.add_argument('--qr-blocks-trial', action='store_true',
               help='Spawn inside QR and test one fixed trajectory to the block waypoint (cone seed 31)')
p.add_argument('--hybrid-full', action='store_true',
               help='Run the complete route with live Hybrid A* from the last bend through QR to the blocks')
p.add_argument('--no-hybrid-optimize', action='store_true', help='Use the original full Hybrid A* speed and geometry')
p.add_argument('--hybrid-qr-blocks', action='store_true',
               help='GUI trial from bend7 through QR to the blocks')
p.add_argument('--offline-cone-layout', nargs=5, metavar='COL,ROW',
               help='Explicit offline test fixture for the Hybrid A* tail')
p.add_argument('--fast-qr', action='store_true',
               help='Plan the QR approach from observed cones and profile the straight reverse')
p.add_argument('--hybrid-qr-only', action='store_true',
               help='Stop the hybrid trial after checking QR entry')
p.add_argument('--qr-blocks-teb', action='store_true',
               help='Use the original QR exit and TEB block approach in the QR-to-blocks trial')
p.add_argument('--debug-no-bend-brake', action='store_true',
               help='Bypass predictive braking for a Gazebo bend-only comparison')
p.add_argument('--debug-joined-uturn', action='store_true',
               help='Let the two bend5 fillets share the connector without a mandatory straight')
p.add_argument('--debug-smooth-corner', nargs=3, action='append', default=[],
               metavar=('WAYPOINT', 'ENTRY_METRES', 'EXIT_METRES'),
               help='Use a smooth corner with independent entry and exit trims')
p.add_argument('--debug-bend-limit', nargs=2, action='append', default=[], metavar=('LIMIT', 'VALUE'),
               help='Override a tracking speed, acceleration or yaw-rate limit for a bend-only trial')
p.add_argument('--bend-y-offset', nargs=2, action='append', default=[], metavar=('WAYPOINT', 'METRES'),
               help='Development-only offset of a reference waypoint along the y axis')
p.add_argument('--bend-x-offset', nargs=2, action='append', default=[], metavar=('WAYPOINT', 'METRES'),
               help='Development-only offset of a reference waypoint along the x axis')
p.add_argument('--bend-corner-radius', nargs=2, action='append', default=[], metavar=('WAYPOINT', 'METRES'),
               help='Development-only radius at a single reference corner')
qr_preposition = p.add_mutually_exclusive_group()
qr_preposition.add_argument('--preposition-qr',action='store_true',help='Move the folded QR camera during the approach')
qr_preposition.add_argument('--no-preposition-qr',dest='preposition_qr',action='store_false')
p.set_defaults(preposition_qr=True)
p.add_argument('--preposition-block', action='store_true', help='Extend the arm to the first block pose after clearing the cones')
return_alignment = p.add_mutually_exclusive_group()
return_alignment.add_argument('--align-return',action='store_true',help='Turn toward the return waypoint when the rotation is clear')
return_alignment.add_argument('--no-align-return',dest='align_return',action='store_false')
p.set_defaults(align_return=True)
p.add_argument('--debug-from-stage',help='Development-only tail from a named route stage')
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
if args.stationary_profile_urdf and (
        not args.hybrid_qr_blocks or not args.profile_performance or not args.stationary_profile_urdf.is_file()):
    p.error('Stationary URDF profiling requires a local URDF, the hybrid GUI trial and performance profiling')
if not 0 <= args.gui_start_hold < math.inf:
    p.error('GUI start hold must be nonnegative and finite')
if args.bend_radius is not None and not 0 < args.bend_radius < math.inf:
    p.error('Bend radius must be positive and finite')
def waypoint_values(entries, *, positive=False):
    values = {}
    for name, raw in entries:
        try:
            value = float(raw)
        except ValueError:
            p.error(f'Invalid bend waypoint value for {name}: {raw}')
        if not math.isfinite(value) or (positive and value <= 0):
            p.error(f'Invalid bend waypoint value for {name}: {raw}')
        values[name] = value
    return values

bend_offsets = waypoint_values(args.bend_y_offset)
x_offsets = waypoint_values(args.bend_x_offset)
corner_radii = waypoint_values(args.bend_corner_radius, positive=True)
bend_limits = waypoint_values(args.debug_bend_limit, positive=True)
smooth_entries = waypoint_values([(name, entry) for name, entry, _ in args.debug_smooth_corner], positive=True)
smooth_exits = waypoint_values([(name, exit_trim) for name, _, exit_trim in args.debug_smooth_corner], positive=True)
if bend_limits.keys() - {'max_speed','acceleration','max_yaw_rate'}:
    p.error('Bend trial limits are max_speed, acceleration and max_yaw_rate')
if args.debug_bends_only and (args.timer or args.debug_prefix_until or args.build_fixed_map or args.debug_pose):
    p.error('Bend trials measure the regular entrance and exit without an official timer or another route override')
if args.fixed_qr_trial and (args.timer or args.debug_bends_only or args.debug_prefix_until or
                            args.build_fixed_map or args.debug_pose or args.map_file or args.attitude_protection or
                            args.qr_blocks_trial):
    p.error('Fixed QR trials use a fresh odometry scene without the official timer or another control mode')
if args.qr_blocks_trial and (args.timer or args.debug_bends_only or args.debug_prefix_until or
                             args.build_fixed_map or args.debug_pose or args.map_file or args.debug_control or
                             args.startup_only):
    p.error('The QR-to-blocks trial already defines a spawn and an endpoint')
if args.qr_blocks_trial and (not args.cones or args.seed != 31):
    p.error('The QR-to-blocks reference is designed for --cones --seed 31')
if args.hybrid_qr_blocks and (not args.cones or not args.gui or not args.contact_observer or
                             args.fixed_qr_trial or args.qr_blocks_trial or args.timer or args.debug_pose or
                             args.debug_bends_only or args.debug_prefix_until or args.build_fixed_map or
                             args.map_file or args.debug_control or args.startup_only):
    p.error('The hybrid trial uses its own route, GUI, random cones and independent contact observer')
if args.hybrid_full and (not args.gui or not args.cones or not args.contact_observer or args.map_file or
                         args.fixed_qr_trial or args.qr_blocks_trial or args.hybrid_qr_blocks or
                         args.debug_pose or args.debug_prefix_until or args.debug_from_stage or
                         args.debug_bends_only or args.startup_only or args.build_fixed_map):
    p.error('The full hybrid trial requires the complete odom route, GUI, cones and a contact observer')
if (args.offline_cone_layout or args.hybrid_qr_only or args.fast_qr) and not args.hybrid_qr_blocks:
    p.error('Offline layout, QR-only and fast QR options require --hybrid-qr-blocks')
if args.hybrid_qr_blocks:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'scripts'))
    from hybrid_path import SPAWN, legal_layouts
    try:
        fixture = [tuple(map(int, cell.split(','))) for cell in (args.offline_cone_layout or [])]
    except ValueError:
        p.error('Each offline fixture cell must use COL,ROW')
    if fixture and frozenset(fixture) not in set(legal_layouts()):
        p.error('The offline fixture must contain five distinct cells in a legal layout')
if args.qr_blocks_teb and not args.qr_blocks_trial:
    p.error('--qr-blocks-teb requires --qr-blocks-trial')
if (args.debug_no_bend_brake or args.debug_joined_uturn or bend_limits or smooth_entries) and not args.debug_bends_only:
    p.error('Experimental tracking limits, joined U-turn and braking bypass require --debug-bends-only')
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
tools_dir = Path(__file__).resolve().parent
package = tools_dir.parent
spawn_pose = SPAWN if args.hybrid_qr_blocks else args.debug_pose
if args.qr_blocks_trial:
    import yaml
    course = yaml.safe_load((package/'config/route.yaml').read_text())
    spawn_pose = next(point[1:] for point in course if point[0] == 'qr')
for directory in ('config','launch'):
    shutil.copytree(package/directory,out/directory)
    for copied in (out/directory).glob('*.launch'):
        copied.rename(copied.with_suffix('.launch.txt'))
sources=sorted((package/'scripts').glob('*.py')) + sorted(tools_dir.glob('*.py')) + sorted(tools_dir.glob('*.cpp'))
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
result['efficient_approach']=args.efficient_approach
result['attitude_protection']=args.attitude_protection
result['navigation_mode']='premapped_amcl' if args.map_file else 'rolling_odom'
result['layout_seed']=args.layout_seed
result['qr_preposition']=args.preposition_qr
result['block_preposition']=args.preposition_block
result['align_return']=args.align_return
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
    if spawn_pose:
        import rospkg
        car=Path(rospkg.RosPack().get_path('car3'));tree=ET.parse(simulation if args.low_com else car/'launch/gazebo.launch')
        x,y,yaw=spawn_pose
        tree.find('.//node[@name="spawn_model"]').set('args',f'-param robot_description -urdf -model car3 -x {x} -y {y} -z 0.01 -Y {yaw}')
        tree.find('.//arg[@name="gui"]').set('default','false')
        if args.stationary_profile_urdf:
            model = out/'diagnostic.urdf'
            shutil.copyfile(args.stationary_profile_urdf, model)
            tree.find('.//param[@name="robot_description"]').set('textfile', str(model))
            result['stationary_urdf_sha256'] = hashlib.sha256(model.read_bytes()).hexdigest()
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
    control_args=['roslaunch',str(package/'launch/control.launch'),'speed_profile:='+args.speed_profile,'timeout_clock:='+args.timeout_clock]
    control_args.append('efficient_approach:='+str(args.efficient_approach).lower())
    control_args.append('qr_preposition:='+str(args.preposition_qr).lower())
    control_args.append('block_preposition:='+str(args.preposition_block).lower())
    control_args.append('align_return:='+str(args.align_return).lower())
    control_args.append('attitude_protection:='+str(args.attitude_protection).lower())
    control_args.append('vision_evidence_root:='+str(out/'vision'))
    if args.hybrid_full:
        control_args.extend(['hybrid_full:=true', 'hybrid_optimize:='+str(not args.no_hybrid_optimize).lower(), 'hybrid_evidence:='+str(out/'hybrid.json')])
        result.update(scope='FULL HYBRID ROUTE', navigation_mode='hybrid_astar_through_finish')
    if args.build_fixed_map:control_args.append('vision:=false')
    if args.fixed_qr_trial:
        control_args.extend(['fixed_qr_trial:=true', 'vision:=false'])
        result.update(scope='FIXED START TO QR BASELINE', navigation_mode='fixed_odometry',
                      braking_collision_check=False, qr_preposition=False)
    if args.hybrid_qr_blocks:
        control_args.extend(['fixed_qr_trial:=true', 'vision:=false'])
        result.update(scope='BEND7 TO QR' if args.hybrid_qr_only else 'BEND7 THROUGH QR TO BLOCKS',
                      navigation_mode='hybrid_astar',
                      qr_preposition=False, offline_cone_layout=fixture)
    if args.qr_blocks_trial:
        control_args.append('vision:=false')
        result.update(scope='QR TO BLOCKS TRIAL',
                      navigation_mode='rolling_odom' if args.qr_blocks_teb else 'fixed_odometry',
                      qr_preposition=False)
    if args.map_file:control_args.append('map_file:='+str(args.map_file.resolve()))
    launch(control_args,'control')
    if args.build_fixed_map:
        launch(['roslaunch',str(package/'launch/mapping.launch')],'mapping')
        launch(['rosbag','record','-O',str(out/'mapping_sensors.bag'),'/nav_scan','/odom','/tf','/tf_static','/slam_toolbox/pose','/map'],'mapping_bag')
        from nav_msgs.msg import OccupancyGrid
        rospy.wait_for_message('/map',OccupancyGrid,timeout=45)
    if args.profile_performance:
        launch(['python3',str(tools_dir/'profile_runtime.py'),str(out/'performance.jsonl')],'performance')
    rospy.wait_for_service('/nav_start',timeout=60)
    if args.hybrid_qr_blocks:
        rospy.set_param('/round2_control/start_position', list(spawn_pose[:2]))
        route = rospy.get_param('/round2_control/route')
        rospy.set_param('/round2_control/route', [row for row in route if row[0] in ('qr', 'qr_exit', 'blocks')])
        rospy.set_param('/round2_control/hybrid_trial', dict(
            output=str(out/'hybrid.json'), world=str(car/'world/round2.world'),
            dwell=1., known_layout=fixture, qr_only=args.hybrid_qr_only, fast_qr=args.fast_qr))
    if args.qr_blocks_trial:
        rospy.set_param('/round2_control/start_position', spawn_pose[:2])
        rospy.set_param('/round2_control/flow_settings/qr_blocks_trial', not args.qr_blocks_teb)
        stages = ('qr_exit', 'cone_entry', 'cone_exit', 'blocks') if args.qr_blocks_teb else (
            'qr', 'qr_exit', 'cone_entry', 'blocks')
        rospy.set_param('/round2_control/route',
                        [point for point in course if point[0] in stages])
    radius_param = '/round2_control/bend_settings/corner_radius'
    if args.bend_radius is not None:
        rospy.set_param(radius_param, args.bend_radius)
    result['bend_radius'] = rospy.get_param(radius_param)
    if bend_offsets or x_offsets or corner_radii or smooth_entries:
        route = rospy.get_param('/round2_control/route')
        configured_bends = {row[0] for row in route if row[0].startswith('bend')}
        if not (bend_offsets.keys() | x_offsets.keys() | corner_radii.keys() | smooth_entries.keys()) <= configured_bends:
            raise ValueError('Bend trial overrides must refer to configured bend waypoints')
        for key, values in (('reference_x_offsets', x_offsets), ('reference_y_offsets', bend_offsets),
                            ('reference_corner_radii', corner_radii)):
            if values:
                rospy.set_param('/round2_control/bend_settings/' + key, values)
    result['bend_x_offsets'] = rospy.get_param('/round2_control/bend_settings/reference_x_offsets', {})
    result['bend_y_offsets'] = rospy.get_param('/round2_control/bend_settings/reference_y_offsets', {})
    result['bend_corner_radii'] = rospy.get_param('/round2_control/bend_settings/reference_corner_radii', {})
    if args.debug_bends_only:
        rospy.set_param('/round2_control/flow_settings/debug_bends_only', True)
        rospy.set_param('/round2_control/flow_settings/debug_no_bend_brake', args.debug_no_bend_brake)
        if args.debug_joined_uturn:
            rospy.set_param('/round2_control/bend_settings/join_bend5_corners', True)
        rospy.set_param('/round2_control/bend_settings/smooth_corners',
                        {name: [entry, smooth_exits[name]] for name, entry in smooth_entries.items()})
        for name, value in bend_limits.items():
            rospy.set_param('/round2_control/bend_settings/fixed_prefix_limits/' + name, value)
        result['scope'] = 'DEBUG BENDS ONLY, NOT A COMPETITION RUN'
    result['bend_limits'] = rospy.get_param('/round2_control/bend_settings')
    result['bend_braking_check'] = (result['bend_limits'].get('fixed_prefix_braking_check', True)
                                    and not args.debug_no_bend_brake)
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
        command = ['rosrun', 'car3', script]
        if script == 'random_place.py' and args.layout_seed is not None:
            import rospkg
            placement = Path(rospkg.RosPack().get_path('car3'))/'scripts'/script
            command = [
                'python3', '-c',
                "import random, runpy, sys; "
                "place = runpy.run_path(sys.argv[1])['main']; "
                "place.__globals__['random'] = random.Random(int(sys.argv[2])); place()",
                str(placement), str(args.layout_seed),
            ]
        with (out/(script+'.log')).open('w') as f:
            subprocess.run(command,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=45)
    time.sleep(3)
    if args.audit_cones:
        launch(['python3',str(tools_dir/'audit_cones.py'),str(out/'cone_audit.json')],'cone_audit')
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
    if args.hybrid_qr_blocks or args.hybrid_full:
        wait_for_gazebo_window()
        print('Gazebo GUI opened. Reviewing stationary spawn before motion.', flush=True)
        time.sleep(args.gui_start_hold)
        if args.stationary_profile_urdf:
            result.update(passed=True, scope='STATIONARY URDF PROFILE', trigger='no motion')
            raise StartupComplete()
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
    if args.debug_from_stage:
        if not args.debug_control:raise RuntimeError('Named tail requires a debug control spawn')
        route=rospy.get_param('/round2_control/route')
        route=route[next(i for i,row in enumerate(route) if row[0]==args.debug_from_stage):]
        rospy.set_param('/round2_control/route',route)
    if args.debug_control:
        if not args.debug_pose:raise RuntimeError('Debug control requires a debug spawn')
        rospy.set_param('/round2_control/start_position',args.debug_pose[:2])
        route=[[args.debug_stage]+args.debug_goal]
        if args.debug_from_stage:
            route=rospy.get_param('/round2_control/route')
        elif args.debug_tail:
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
    result['executed_route']=rospy.get_param('/round2_control/route')
    result['continuous_regions']=True
    rospy.ServiceProxy('/nav_start',SetBool)(False)
    time.sleep(2)
    if args.timer:
        timer_path = out/'timer/timer.json'
        timer_state = json.loads(timer_path.read_text()) if timer_path.is_file() else {}
        timer_results = timer_state.get('results', [])
        result['official_timer_stopped'] = bool(timer_results)
        result['official_timer_results'] = timer_results
        result['passed'] = result['passed'] and result['official_timer_stopped']
    (out/'poses.json').write_text(json.dumps(poses))
    if args.hybrid_qr_blocks or args.hybrid_full:
        trial_file = out/'hybrid.json'
        if trial_file.is_file():
            trial = json.loads(trial_file.read_text())
            result['observation_wait_sim_s'] = trial.get('observation_wait_sim_s', 0.) + sum(
                leg.get('wait_sim_s', 0.) for leg in trial.get('exploration', []))
            if 'qr_stop' in trial:
                result['spawn_to_qr_sim_s'] = trial['qr_stop']['stamp'] - trial['departure_stamp']
                ingress = [pose for pose in poses if trial['departure_stamp'] <= pose['t'] <= trial['qr_stop']['stamp']]
                result['ingress_distance_m'] = sum(math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                                                  for a, b in zip(ingress, ingress[1:]))
                result['peak_ingress_speed_mps'] = max((pose['v'][0] for pose in ingress), default=0.)
            if 'reverse_start' in trial:
                reverse = next((pose for pose in poses
                                if pose['t'] >= trial['reverse_start']['stamp'] and pose['v'][0] < -.015), None)
                if reverse:
                    result['qr_dwell_sim_s'] = reverse['t'] - trial['qr_stop']['stamp']
                if reverse and 'reverse_stop' in trial:
                    result['qr_reverse_sim_s'] = trial['reverse_stop']['stamp'] - reverse['t']
                    result['peak_reverse_speed_mps'] = max((-pose['v'][0] for pose in poses
                                                            if reverse['t'] <= pose['t'] <= trial['reverse_stop']['stamp']),
                                                           default=0.)
                if reverse and 'blocks_stop' in trial:
                    end = trial['blocks_stop']['stamp']
                    result['reverse_to_blocks_sim_s'] = end - reverse['t']
                    forward = [pose for pose in poses if trial['reverse_stop']['stamp'] <= pose['t'] <= end]
                    result['forward_distance_m'] = sum(math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                                                       for a, b in zip(forward, forward[1:]))
                    result['peak_forward_speed_mps'] = max((pose['v'][0] for pose in forward), default=0.)
        print('Gazebo GUI remains open briefly for stop inspection.', flush=True)
        time.sleep(8)
    if args.hybrid_full:
        result['vision'] = [json.loads(path.read_text())
                            for path in sorted((out/'vision').glob('*/result.json'))]
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
    if args.hybrid_qr_blocks or args.hybrid_full:
        contact_path = out/'contacts.jsonl'
        audit = [json.loads(line) for line in contact_path.read_text().splitlines()] if contact_path.is_file() else []
        summary = next((row for row in reversed(audit) if row['type'] == 'summary'), {})
        result['contact_summary'] = summary
        result['passed'] &= bool(summary.get('messages')) and not any(
            row['type'] == 'contact_begin' for row in audit)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    for f in files:f.close()
print(json.dumps(result),flush=True)
raise SystemExit(0 if result['passed'] else 1)
