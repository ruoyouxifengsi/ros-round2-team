"""Check real launch frames, static/live layers, rigid pose transforms and survey scope."""
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import yaml
from fixed_mapping import prepare_scene
from localized_odometry import corrected_pose

with tempfile.TemporaryDirectory() as folder:
    source=Path(folder)/'source.world';dest=Path(folder)/'fixed.world'
    source.write_text('<sdf><world name="round2"><gravity>0 0 -9.8</gravity><physics type="ode"><max_step_size>0.001</max_step_size></physics><model name="wall"><pose>1 2 0 0 0 0</pose></model><model name="cube_0"/><model name="Construction Cone"/><state><model name="wall"/><model name="cube_0"/></state></world></sdf>')
    manifest=prepare_scene(source,dest);a=ET.parse(source);b=ET.parse(dest)
    assert manifest['removed_models']==['Construction Cone','cube_0']
    for tag in ('gravity','physics','model[@name="wall"]'):
        assert ET.tostring(a.find('world/'+tag))==ET.tostring(b.find('world/'+tag))
    assert [m.get('name') for m in b.findall('world/model')]==['wall']
    assert [m.get('name') for m in b.findall('world/state/model')]==['wall']
xyz,q=corrected_pose((1.,0.,0.),(0.,0.,0.,1.),(2.,3.,0.),(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)))
assert math.dist(xyz,(2.,4.,0.))<1e-9 and abs(q[2]-math.sin(math.pi/4))<1e-9
for file,frame in [('', 'odom'),('/tmp/laser_built_map.yaml','map')]:
    args=['roslaunch','--dump-params','round2_baseline','baseline.launch','timer:=false','map_file:='+file]
    cfg=yaml.safe_load(subprocess.check_output(args))
    assert cfg['/round2_control/navigation_frame']==frame
    assert cfg['/cone_obstacles/navigation_frame']==frame
    assert cfg['/move_base/local_costmap/global_frame']==frame
    assert cfg['/move_base/global_costmap/global_frame']==frame
    assert cfg['/round2_control/pose_topic']==('/localized_odom' if file else '/odom')
    assert cfg['/round2_control/docking_inset']==(.03 if file else 0.)
    names=[p['name'] for p in cfg['/move_base/global_costmap/plugins']]
    assert ('static_layer' in names)==bool(file)
    assert 'obstacle_layer' in names and 'cone_layer' in names
    if file:
        assert cfg['/amcl/odom_model_type']=='omni-corrected'
        assert cfg['/amcl/do_beamskip']
        assert cfg['/move_base/NavfnROS/allow_unknown'] is False
print('PASS: fixed-only survey keeps physics; map pose transform; static and live layers; original mode unchanged')
