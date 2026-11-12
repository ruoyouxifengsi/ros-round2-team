#!/usr/bin/env python3
"""Development-only wall/simulation clock and Linux thread load sampler."""
import json
import os
from pathlib import Path
import sys
import time
import rospy
from rosgraph_msgs.msg import Clock

rospy.init_node('runtime_profiler', anonymous=True)
sim = None
def clock(msg):
    global sim
    sim = msg.clock.to_sec()
rospy.Subscriber('/clock', Clock, clock, queue_size=1)
previous = {}
hz = os.sysconf('SC_CLK_TCK')
last = time.monotonic()
with Path(sys.argv[1]).open('w') as output:
    while not rospy.is_shutdown():
        time.sleep(1)
        now = time.monotonic()
        processes = []
        current = {}
        for entry in Path('/proc').glob('[0-9]*'):
            try:
                name = (entry/'comm').read_text().strip()
                if name not in ('gzserver', 'gzclient', 'move_base'):continue
                loads = []
                for task in (entry/'task').iterdir():
                    fields = (task/'stat').read_text().rsplit(')', 1)[1].split()
                    ticks = int(fields[11])+int(fields[12])
                    key = (entry.name, task.name)
                    current[key] = ticks
                    if key in previous:
                        loads.append(100*(ticks-previous[key])/hz/(now-last))
                processes.append(dict(pid=int(entry.name), name=name,
                    cpu_percent=sum(loads), busiest_thread_percent=max(loads, default=0),
                    threads=len(loads)))
            except (OSError, ValueError, IndexError):continue
        output.write(json.dumps(dict(wall=now, sim=sim, cpus=os.cpu_count(), processes=processes))+'\n')
        output.flush()
        previous = current
        last = now
