"""Plot only sensor-derived observations and odometry; no simulator truth."""
import json
import math
import sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

folder=Path(sys.argv[1]);poses=json.loads((folder/'poses.json').read_text());result=json.loads((folder/'result.json').read_text())
fig,ax=plt.subplots(figsize=(9,8))
footprint=np.array(yaml.safe_load((folder/'config/navigation.yaml').read_text())['local_costmap']['footprint'])
for path in sorted((folder/'snapshots').glob('snapshot_*.json')):
    row=json.loads(path.read_text())
    if 'scan' not in row or max(abs(v) for v in row['rpy'][:2])>math.radians(.7):continue
    s=row['scan'];q=s['rotation'];yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));rr=np.array(s['ranges']);a=s['angle_min']+np.arange(len(rr))*s['angle_increment']+yaw
    valid=np.isfinite(rr)&(rr>0)&(rr<5)
    ax.scatter((s['position'][0]+rr*np.cos(a))[valid][::3],(s['position'][1]+rr*np.sin(a))[valid][::3],s=1,c='#839096',alpha=.3)
ax.plot([p['x'] for p in poses],[p['y'] for p in poses],color='#087e8b',lw=2,label='Odometry trajectory')
for event in result['events']:
    if event.get('state')!='ARRIVED':continue
    p=min(poses,key=lambda p:abs(p['t']-event['stamp']));ax.scatter(p['x'],p['y'],c='#d55336',s=18);ax.annotate(event['detail'],(p['x'],p['y']),xytext=(4,5),textcoords='offset points',fontsize=8)
    if event['detail'] in ('qr','finish'):
        yaw=p['rpy'][2];c,s=math.cos(yaw),math.sin(yaw)
        body=footprint@np.array([[c,s],[-s,c]])+[p['x'],p['y']]
        ax.add_patch(Polygon(body,facecolor='#eab44d',edgecolor='#875813',alpha=.65,zorder=4))
ax.add_patch(Rectangle((0,0),.5,.5,fill=False,edgecolor='#b344b6',lw=2));ax.text(.05,.04,'QR',color='#b344b6')
ax.add_patch(Rectangle((-.5,2),.5,.5,fill=False,edgecolor='#b344b6',lw=2));ax.text(-.47,2.04,'FINISH',color='#b344b6')
ax.set(xlim=(-2.7,1.7),ylim=(-.2,3.2),xlabel='x (m)',ylabel='y (m)',title='Control baseline: laser observations + odometry\n'+folder.name)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left');fig.tight_layout();fig.savefig(folder/'trajectory.png',dpi=160)
