#!/usr/bin/env python3
"""Check real launch configuration and reject prohibited/deceptive CLI modes."""
from pathlib import Path
import subprocess
import sys
import yaml
import xml.etree.ElementTree as ET

for timer,expected in [('false','sim'),('true','wall')]:
    params=yaml.safe_load(subprocess.check_output(['roslaunch','--dump-params','round2_baseline','baseline.launch',
        'timer:='+timer,'speed_profile:=fast','timeout_clock:=sim']))
    assert params['/round2_control/timeout_clock']==expected
    assert params['/move_base/TebLocalPlannerROS/max_vel_x']==.4
    assert params['/attitude_guard/linear_acceleration']==.5
    print('PASS: timer='+timer+', control timeout clock='+expected)
params=yaml.safe_load(subprocess.check_output(['roslaunch','--dump-params','round2_baseline','baseline.launch',
    'timer:=false','speed_profile:=sprint','continuous_bends:=true','efficient_approach:=true']))
assert params['/round2_control/continuous_bends'] is True
assert params['/round2_control/efficient_approach'] is True
assert params['/move_base/TebLocalPlannerROS/max_vel_x']==.5
assert params['/move_base/TebLocalPlannerROS/acc_lim_x']==.65
assert params['/attitude_guard/linear_acceleration']==.65
print('PASS: continuous sprint launch forwards consistent planner and guard limits')
original=ET.fromstring(params['/robot_description'])
manual=yaml.safe_load(subprocess.check_output(['roslaunch','--dump-params','round2_baseline','baseline.launch',
    'timer:=true','low_com:=true','speed_profile:=sprint','continuous_bends:=true','efficient_approach:=true']))
lowered=ET.fromstring(manual['/robot_description'])
origin=original.find('./link[@name="base_link"]/inertial/origin')
assert origin.get('xyz')=='0 0 0.045754'
origin.set('xyz','0 0 0.02')
def structure(node):return (node.tag,dict(node.attrib),(node.text or '').strip(),[structure(c) for c in node])
assert structure(original)==structure(lowered),'Manual low-COM model differs beyond the approved origin'
assert manual['/round2_control/timeout_clock']=='wall'
assert manual['/round2_control/efficient_approach'] and manual['/round2_control/continuous_bends']
nodes=subprocess.check_output(['roslaunch','--nodes','round2_baseline','baseline.launch','timer:=true','low_com:=true']).decode().splitlines()
assert '/accum_timer' in nodes
assert not any('trial' in n for n in nodes),nodes
print('PASS: manual timer entry uses approved low COM only; original stays default; no automatic trial driver')
script=Path(__file__).with_name('run_trial.py')
for flags in [ ['--timer','--timeout-clock','sim'], ['--experimental-collision-library','unused.so'],
               ['--contact-observer','/tmp/round2_observer_not_built'] ]:
    result=subprocess.run([sys.executable,str(script),'/tmp/round2_rejected_options',*flags],capture_output=True,text=True)
    assert result.returncode==2,result.stderr
    print('PASS: rejected '+ ' '.join(flags))
