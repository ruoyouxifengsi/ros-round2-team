"""Lifecycle of processes started by the development harness."""
import os
import signal
import subprocess
from pathlib import Path
import socket
import time


def process_table():
    table={}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields=path.read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z':table[int(path.parent.name)]=(int(fields[1]),fields[19])
        except (OSError,ValueError,IndexError):pass
    return table


def wait_for_ports(ports,timeout=10):
    deadline=time.monotonic()+timeout
    while True:
        busy=[]
        for port in ports:
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try:sock.bind(('127.0.0.1',port))
                except OSError:busy.append(port)
        if not busy:return
        if time.monotonic()>=deadline:raise RuntimeError(f'Trial ports already in use: {busy}; existing processes were not stopped')
        time.sleep(.1)


def stop_processes(processes):
    table=process_table()
    owned={p.pid:table[p.pid][1] for p in processes if p.pid in table}
    while True:
        children={pid:start for pid,(parent,start) in table.items() if parent in owned and pid not in owned}
        if not children:break
        owned.update(children)
    for proc in reversed(processes):
        if proc.poll() is None:
            try:proc.send_signal(signal.SIGINT)
            except ProcessLookupError:pass
    deadline=time.monotonic()+25
    while any(p.poll() is None for p in processes) and time.monotonic()<deadline:time.sleep(.1)
    def survivors():
        current=process_table()
        return [pid for pid,start in owned.items() if pid in current and current[pid][1]==start]
    for sig in (signal.SIGTERM,signal.SIGKILL):
        for pid in survivors():
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+3
        while survivors() and time.monotonic()<deadline:time.sleep(.1)
    for proc in processes:
        try:proc.wait(timeout=1)
        except subprocess.TimeoutExpired:pass
    return dict(owned_processes=len(owned),remaining=survivors())
