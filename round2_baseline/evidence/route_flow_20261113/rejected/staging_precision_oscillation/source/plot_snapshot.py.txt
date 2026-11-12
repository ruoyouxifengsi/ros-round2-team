import json
import sys
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

p=Path(sys.argv[1]);d=json.loads(p.read_text())
fig,ax=plt.subplots(figsize=(9,7))
m=d.get('global',d.get('local'));a=np.array(m['data']).reshape(m['height'],m['width']);x,y=m['origin'];r=m['resolution']
ax.imshow(a,origin='lower',extent=[x,x+r*m['width'],y,y+r*m['height']],vmin=0,vmax=100,cmap='Greys',alpha=.7)
if d.get('plan'):
 a=np.array(d['plan']);ax.plot(a[:,0],a[:,1],'g-',label='plan')
x,y=d['position'];yaw=d['rpy'][2]
vertices=yaml.safe_load((p.parent.parent/'config/navigation.yaml').read_text())['local_costmap']['footprint']
corners=np.array(vertices+[vertices[0]])
rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
c=corners@rot.T+[x,y];ax.plot(c[:,0],c[:,1],'b-',label='footprint');ax.arrow(x,y,.25*np.cos(yaw),.25*np.sin(yaw),color='b')
if 'scan' in d:
 s=d['scan'];q=s['rotation'];syaw=np.arctan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));rr=np.array(s['ranges']);angles=s['angle_min']+np.arange(len(rr))*s['angle_increment']+syaw
 ax.scatter(s['position'][0]+rr*np.cos(angles),s['position'][1]+rr*np.sin(angles),s=2,c='red',label='live scan')
ax.set_xlim(-2.7,1.7);ax.set_ylim(-.2,3.3);ax.set_aspect('equal');ax.grid();ax.legend();ax.set_title(str(d.get('status'))+'\n'+str(d['rpy']),fontsize=8)
fig.savefig(p.with_suffix('.png'),dpi=130)
