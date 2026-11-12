#!/usr/bin/env python3
"""Check real launch configuration and reject prohibited/deceptive CLI modes."""
from pathlib import Path
import subprocess
import sys
import yaml

for timer,expected in [('false','sim'),('true','wall')]:
    params=yaml.safe_load(subprocess.check_output(['roslaunch','--dump-params','round2_baseline','baseline.launch',
        'timer:='+timer,'speed_profile:=fast','timeout_clock:=sim']))
    assert params['/round2_control/timeout_clock']==expected
    assert params['/move_base/TebLocalPlannerROS/max_vel_x']==.4
    assert params['/attitude_guard/linear_acceleration']==.5
    print('PASS: timer='+timer+', control timeout clock='+expected)
params=yaml.safe_load(subprocess.check_output(['roslaunch','--dump-params','round2_baseline','baseline.launch',
    'timer:=false','speed_profile:=sprint','continuous_bends:=true']))
assert params['/round2_control/continuous_bends'] is True
assert params['/move_base/TebLocalPlannerROS/max_vel_x']==.5
assert params['/move_base/TebLocalPlannerROS/acc_lim_x']==.65
assert params['/attitude_guard/linear_acceleration']==.65
print('PASS: continuous sprint launch forwards consistent planner and guard limits')
script=Path(__file__).with_name('run_trial.py')
for flags in [ ['--timer','--timeout-clock','sim'], ['--experimental-collision-library','unused.so'],
               ['--contact-observer','/tmp/round2_observer_not_built'] ]:
    result=subprocess.run([sys.executable,str(script),'/tmp/round2_rejected_options',*flags],capture_output=True,text=True)
    assert result.returncode==2,result.stderr
    print('PASS: rejected '+ ' '.join(flags))
