"""Send recognition requests and wait for the vision action to finish."""

import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from round2_vision.msg import RecognizeTaskAction, RecognizeTaskGoal


_vision_client = None
def vision_client():
    """惰性创建图像识别 action 客户端。"""
    global _vision_client
    if _vision_client is None:
        client = actionlib.SimpleActionClient('/round2_vision/recognize', RecognizeTaskAction)
        if not client.wait_for_server(rospy.Duration(15)):
            raise RuntimeError('Image recognition server unavailable')
        _vision_client = client
    return _vision_client
def image_task(control,stage,expected='',timeout=None):
    """阻塞等待图像任务完成：有超时、可被取消、必须等到结果才继续。"""
    timeout = float(timeout or rospy.get_param('~image_timeout',40.0))
    client = vision_client()
    control.report('IMAGE_WAIT',stage)
    client.send_goal(RecognizeTaskGoal(stage=stage,expected=expected or '',timeout=timeout))
    deadline = time.monotonic() + timeout + 2
    terminal = (GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST)
    while client.get_state() not in terminal:
        if rospy.is_shutdown() or control.cancelled.is_set():
            client.cancel_goal()
            raise RuntimeError('Image task cancelled: '+stage)
        if time.monotonic() >= deadline:
            client.cancel_goal()
            raise RuntimeError('Image task watchdog timeout: '+stage)
        time.sleep(0.05)
    result = client.get_result()
    if result is None or not result.success:
        detail = result.detail if result is not None else 'no result'
        raise RuntimeError('Image task failed (%s): %s' % (stage,detail))
    control.report('RECOGNIZED','%s=%s' % (stage,result.task_label))
    rospy.loginfo('vision %s -> %s | %s',stage,result.task_label,result.utterance)
    return result
