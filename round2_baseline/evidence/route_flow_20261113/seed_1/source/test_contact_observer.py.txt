"""Development-only positive contact fixture in the unchanged official engine."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from trial_processes import stop_processes,wait_for_ports

folder=Path(sys.argv[1]).resolve();folder.mkdir(parents=True,exist_ok=False)
os.environ['ROS_MASTER_URI']='http://localhost:11321'
os.environ['GAZEBO_MASTER_URI']='http://localhost:11355'
processes=[];files=[]
def launch(command,name):
    f=(folder/name).open('w');files.append(f)
    p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p);return p
try:
    wait_for_ports((11321,11355))
    launch(['roslaunch','round2_baseline','simulation.launch','gui:=false'],'simulation.log')
    import rospy
    from nav_msgs.msg import Odometry
    from gazebo_msgs.msg import ModelStates,ModelState
    from gazebo_msgs.srv import SetModelState
    rospy.init_node('contact_observer_fixture',anonymous=True)
    odom=rospy.wait_for_message('/odom',Odometry,timeout=90)
    models=rospy.wait_for_message('/gazebo/model_states',ModelStates,timeout=10)
    name=next(n for n in models.name if 'Cone' in n or 'cone' in n)
    launch([sys.argv[2],str(folder/'contacts.jsonl')],'observer.log')
    time.sleep(4)
    before=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
    assert before[-1]['messages']>0 and before[-1]['cone_contact_events']==0,before[-1]
    state=ModelState();state.model_name=name;state.reference_frame='world'
    state.pose.position.x=odom.pose.pose.position.x
    state.pose.position.y=odom.pose.pose.position.y+.1
    state.pose.orientation.w=1.
    assert rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state).success
    time.sleep(6)
    after=[json.loads(line) for line in (folder/'contacts.jsonl').read_text().splitlines()]
    assert any(row['type']=='contact_begin' for row in after),after[-1]
    (folder/'result.json').write_text(json.dumps(dict(passed=True,negative=before[-1],positive=after[-1],
        scope='DEVELOPMENT FIXTURE ONLY: deliberately placed cone into stationary vehicle; no navigation'),indent=2))
    print('PASS: official transport streams contacts; clear vehicle negative; deliberately overlapping cone positive')
finally:
    print(stop_processes(processes))
    for f in files:f.close()
