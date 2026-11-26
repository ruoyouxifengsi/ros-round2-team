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


def wait_for_gazebo_window(timeout=20.):
    """Wait for the Gazebo main window on the trial's X11 display."""
    import ctypes as c

    x11 = c.CDLL('libX11.so.6')
    pointer, window = c.c_void_p, c.c_ulong
    windows = c.POINTER(window)
    x11.XOpenDisplay.restype = pointer
    x11.XDefaultRootWindow.argtypes = [pointer]
    x11.XDefaultRootWindow.restype = window
    x11.XQueryTree.argtypes = [pointer, window, c.POINTER(window), c.POINTER(window),
                             c.POINTER(windows), c.POINTER(c.c_uint)]
    x11.XFetchName.argtypes = [pointer, window, c.POINTER(c.c_char_p)]
    x11.XFree.argtypes = [pointer]
    x11.XCloseDisplay.argtypes = [pointer]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot connect to the Gazebo GUI display')

    def has_window(parent, depth):
        name = c.c_char_p()
        if x11.XFetchName(display, parent, c.byref(name)) and name:
            try:
                if name.value == b'Gazebo':
                    return True
            finally:
                x11.XFree(name)
        if depth:
            root, owner, children, count = window(), window(), windows(), c.c_uint()
            if x11.XQueryTree(display, parent, c.byref(root), c.byref(owner), c.byref(children), c.byref(count)):
                try:
                    return any(has_window(children[i], depth - 1) for i in range(count.value))
                finally:
                    if children:
                        x11.XFree(children)
        return False

    try:
        root = x11.XDefaultRootWindow(display)
        deadline = time.monotonic() + timeout
        while not has_window(root, 2):
            if time.monotonic() >= deadline:
                raise RuntimeError('Gazebo GUI main window did not appear')
            time.sleep(.25)
    finally:
        x11.XCloseDisplay(display)
