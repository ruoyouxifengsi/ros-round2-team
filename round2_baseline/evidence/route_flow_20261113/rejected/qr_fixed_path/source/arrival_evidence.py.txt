"""Select fresh odometry available by the arrival event, never the next stage."""
def arrival_pose(poses,event):
    wall=event.get('wall')
    candidates=[p for p in poses if p['t']<=event['stamp'] and
                (wall is None or p.get('wall',float('inf'))<=wall)]
    if not candidates:return None
    pose=max(candidates,key=lambda p:(p['t'],p.get('wall',0.)))
    if event['stamp']-pose['t']>.1:return None
    if wall is not None and wall-pose.get('wall',float('-inf'))>1.:return None
    return pose
