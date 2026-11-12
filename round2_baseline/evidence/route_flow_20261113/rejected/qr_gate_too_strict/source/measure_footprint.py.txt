"""Measure URDF collision meshes; does not modify model or consume simulator truth."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from tf.transformations import euler_matrix, rotation_matrix
base=Path(__file__).resolve().parents[2]/'第二次分站赛下发/src/car3'
root=ET.parse(base/'urdf/car3.urdf').getroot()
def transform(origin):
    t=np.eye(4)
    if origin is not None:
        t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=euler_matrix(*np.fromstring(origin.get('rpy','0 0 0'),sep=' '))[:3,:3]
    return t
angles=dict(zip(['arm_joint'+str(i) for i in range(1,6)],[0,-.785,1.57,1.57,0]))
frames={'base_footprint':np.eye(4)}
pending=list(root.findall('joint'))
while pending:
    count=len(pending)
    for j in pending[:]:
        parent=j.find('parent').get('link');child=j.find('child').get('link')
        if parent not in frames:continue
        t=transform(j.find('origin'));q=angles.get(j.get('name'),0)
        if q:
            a=np.fromstring(j.find('axis').get('xyz'),sep=' ');t=t@rotation_matrix(q,a)
        frames[child]=frames[parent]@t;pending.remove(j)
    if len(pending)==count:raise RuntimeError('unresolved URDF tree')
points=[];bounds={}
for link in root.findall('link'):
    for col in link.findall('collision'):
        mesh=col.find('geometry/mesh')
        if mesh is None:continue
        path=base/mesh.get('filename').replace('package://car3/','')
        data=path.read_bytes();v=np.frombuffer(data[84:],dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]))['v'].astype(float)
        scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ');v*=scale
        t=frames[link.get('name')]@transform(col.find('origin'))
        v=v@t[:3,:3].T+t[:3,3]
        v=v.reshape(-1,3)
        if len(v):
            points.extend(v[:,:2]);bounds[link.get('name')]=[v.min(0).tolist(),v.max(0).tolist()]
def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
p=sorted(set(map(tuple,points)));lower=[];upper=[]
for seq,hull in [(p,lower),(reversed(p),upper)]:
    for pt in seq:
        while len(hull)>=2 and cross(hull[-2],hull[-1],pt)<=0:hull.pop()
        hull.append(pt)
h=np.array(lower[:-1]+upper[:-1])
out=dict(height_envelope='all collision mesh vertices',arm_pose=angles,footprint=h.tolist(),bounds=bounds)
(Path(__file__).resolve().parents[1]/'evidence/footprint_measurement.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out))
