"""Prepare a fixed-only survey scene and save sensor-built maps without editing cells."""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

VARIABLE_MODELS={'Construction Cone','qr_board_s','qr_board_goods','qr_board_electronic','cube_0','cube_1','cube_2'}

def prepare_scene(source,destination):
    tree=ET.parse(source);world=tree.find('world');removed=[]
    for parent in [world]+list(world.findall('state')):
        for model in list(parent.findall('model')):
            if model.get('name') in VARIABLE_MODELS:
                removed.append(model.get('name'));parent.remove(model)
    tree.write(destination)
    return dict(source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
        fixed_scene_sha256=hashlib.sha256(Path(destination).read_bytes()).hexdigest(),removed_models=sorted(set(removed)),
        method='Remove movable task objects only before scanning; retain fixed scene and physics. No geometry is converted into map cells.')

def survey_route(route):
    result=[p for p in route if p[0] not in ('qr','qr_exit','blocks','finish')]
    # The start corridor is a dead end from below; do not request a return through its wall.
    result.append(['survey_return',-.70,1.20,0.])
    return result

def save_map(folder):
    import rospy
    from slam_toolbox_msgs.srv import SerializePoseGraph
    folder=Path(folder);folder.mkdir(exist_ok=True)
    rospy.wait_for_service('/slam_toolbox/serialize_map',timeout=20)
    rospy.ServiceProxy('/slam_toolbox/serialize_map',SerializePoseGraph)(str(folder/'fixed_graph'))
    subprocess.run(['rosrun','map_server','map_saver','-f','fixed_map','map:=/map'],cwd=str(folder),timeout=35,check=True)
    names=('fixed_map.yaml','fixed_map.pgm','fixed_graph.posegraph','fixed_graph.data')
    return {name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in names}
