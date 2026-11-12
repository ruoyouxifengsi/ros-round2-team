#!/usr/bin/env python3
"""Exercise the unchanged official Draw widget; never supplies navigation goals."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import tkinter as tk
import rospy
import rospkg
import yaml
from std_msgs.msg import String

out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
pkg=Path(rospkg.RosPack().get_path('ucar_accumtimer'))
source=pkg/'scripts/AccumTimer.py'
rospy.init_node('accum_timer')
for key,value in yaml.safe_load((pkg/'config/AccumTimer.yaml').read_text()).items():rospy.set_param('~'+key,value)
rospy.set_param('my_address',str(out)+'/')
record={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'draw_invocations':0,'status':None}
rospy.Subscriber('/baseline/status',String,lambda m:record.update(status=json.loads(m.data)))
spec=importlib.util.spec_from_file_location('official_timer',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
root=tk.Tk()
def walk(w):
    yield w
    for child in w.winfo_children():yield from walk(child)
def poll():
    record['labels']=[str(w.cget('text')) for w in walk(root) if w.winfo_class() in ('Label','TLabel')]
    (out/'timer.json').write_text(json.dumps(record,indent=2))
    root.after(1000,poll)
def draw():
    entries=[w for w in walk(root) if w.winfo_class()=='TEntry']
    buttons=[w for w in walk(root) if w.winfo_class() in ('Button','TButton') and w.cget('text')=='Draw']
    if len(entries)!=1 or len(buttons)!=1:raise RuntimeError('Unexpected official UI')
    entries[0].insert(0,'7');record['draw_invocations']+=1;record['draw_wall']=time.time()
    buttons[0].invoke()
root.after(1000,poll);root.after(2000,draw)
module.AccumTimerApp(root)
