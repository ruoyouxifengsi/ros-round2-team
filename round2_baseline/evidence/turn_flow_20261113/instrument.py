import functools,json,runpy,sys,time
from pathlib import Path
import rospy
from std_msgs.msg import String

source=Path(__file__).resolve().parents[2]/'scripts/control.py'
sys.argv=sys.argv[1:]
ns=runpy.run_path(str(source),run_name='instrumented_control')
import route_flow
rospy.init_node('round2_control')
publisher=rospy.Publisher('/baseline/probe',String,queue_size=100)
def wrap(function):
    @functools.wraps(function)
    def measured(*args,**kwargs):
        start=time.monotonic()
        def emit(edge):
            publisher.publish(String(json.dumps(dict(phase=function.__name__,edge=edge,stamp=rospy.get_time(),wall=time.monotonic(),id=start))))
        emit('begin')
        try:return function(*args,**kwargs)
        finally:emit('end')
    return measured
for name in ('stopped','rotate_short','straight_move','follow_bends'):
    setattr(ns['Control'],name,wrap(getattr(ns['Control'],name)))
route_flow.navigate=wrap(route_flow.navigate)
control=ns['Control']()
rospy.spin()
