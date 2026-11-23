"""An arrival must not use motion from the following stage or stale evidence."""
from arrival_evidence import arrival_pose

event=dict(stamp=575.912,wall=22918.113805632)
before=dict(t=575.894,wall=22917.769072027,v=[0.,0.,0.])
after=dict(t=575.929,wall=22918.830645523,v=[0.,0.,.067935])
assert abs(after['t']-event['stamp'])<abs(before['t']-event['stamp'])
assert arrival_pose([after,before],event) is before
assert arrival_pose([after],event) is None
assert arrival_pose([dict(before,t=575.7)],event) is None
assert arrival_pose([dict(before,wall=22916.)],event) is None
moving=dict(before,v=[.03,0.,0.])
assert arrival_pose([moving,after],event) is moving
assert arrival_pose([dict(before,wall=22919.)],event) is None
print('PASS: causal arrival sample; reject future, stale or missing evidence; preserve actual motion')
