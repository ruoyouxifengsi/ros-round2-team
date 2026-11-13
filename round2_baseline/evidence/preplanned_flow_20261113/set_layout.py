"""Development placement only; use the official generator and unchanged cone models."""
import importlib.util,json,random,sys
from pathlib import Path
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
root=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('official_cones',root/'第二次分站赛下发/src/car3/scripts/random_cones.py')
official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
rospy.init_node('development_cone_layout',anonymous=True)
if sys.argv[1]=='restore':
    positions=json.loads((Path(__file__).parent/'baseline/result.json').read_text())['cones_before']
else:
    random.seed(int(sys.argv[1]))
    positions={'construction_cone_'+str(i):[*official.cell_xy(*cell),0.] for i,cell in enumerate(sorted(official.pick_cone_cells(5)))}
if '--check-only' in sys.argv:
    original=json.loads((Path(__file__).parent/'baseline/result.json').read_text())['cones_before']
    print(json.dumps(dict(seed=sys.argv[1],matches_original=all(abs(original[n][0]-xyz[0])+abs(original[n][1]-xyz[1])<.001 for n,xyz in positions.items()),positions=positions)))
    sys.exit(0)
for name,xyz in positions.items():
    state=ModelState(model_name=name,reference_frame='world')
    state.pose.position.x,state.pose.position.y,state.pose.position.z=xyz
    state.pose.orientation.w=1.
    assert rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state).success
print(json.dumps(positions,indent=2))
