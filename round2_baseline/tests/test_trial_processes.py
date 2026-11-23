#!/usr/bin/env python3
"""Exercise the independent child sessions used by ROS launch processes."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from trial_processes import stop_processes,wait_for_ports

with tempfile.TemporaryDirectory() as directory:
    marker=Path(directory)/'child.pid'
    child_code='import time; time.sleep(120)'
    parent_code='import subprocess,sys,time; from pathlib import Path; p=subprocess.Popen([sys.executable,"-c",sys.argv[2]],start_new_session=True); Path(sys.argv[1]).write_text(str(p.pid)); time.sleep(120)'
    parent=subprocess.Popen([sys.executable,'-c',parent_code,str(marker),child_code],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    child=None
    try:
        deadline=time.monotonic()+5
        while not marker.exists() and time.monotonic()<deadline:time.sleep(.05)
        child=int(marker.read_text())
        stop_processes([parent])
        child_stat=Path(f'/proc/{child}/stat')
        alive=child_stat.exists() and child_stat.read_text().rsplit(')',1)[1].split()[0]!='Z'
        assert not alive,'ROS-style child session survives parent/process-group cleanup'
        print('PASS: independently grouped child is stopped')
    finally:
        if child:
            try:os.kill(child,signal.SIGKILL)
            except ProcessLookupError:pass
        if parent.poll() is None:parent.kill()
        parent.wait()

with socket.socket() as occupied:
    occupied.bind(('127.0.0.1',0));occupied.listen()
    port=occupied.getsockname()[1]
    try:wait_for_ports((port,),timeout=.1)
    except RuntimeError:print('PASS: occupied port rejected without stopping its owner')
    else:raise AssertionError('Occupied port was accepted')
