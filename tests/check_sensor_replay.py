"""Read recorded CDR, then verify actual rosbag replay in a fresh ROS domain."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from check_milestone_4 import SPECS, BAG_TOPICS, stamp, require, stop_process

parser=argparse.ArgumentParser(); parser.add_argument('--evidence',type=Path,required=True); args=parser.parse_args()
result={'status':'FAIL'}; player=None
rclpy.init(); node=Node('m4_replay_check'); buffer=Buffer(); listener=TransformListener(buffer,node)
try:
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.evidence/'sensors-bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}
    require(set(BAG_TOPICS)<=set(types),f'Missing bag topics {set(BAG_TOPICS)-set(types)}')
    expected=defaultdict(set); counts=Counter(); ranges=defaultdict(list)
    while reader.has_next():
        topic,data,t=reader.read_next(); counts[topic]+=1
        if topic in SPECS:
            msg=deserialize_message(data,get_message(types[topic]))
            require(msg.header.frame_id==SPECS[topic][1],f'Bag frame {topic}')
            expected[topic].add(hashlib.sha256(serialize_message(msg)).hexdigest())
            ranges[topic].append(stamp(msg))
    require(all(counts[t]>0 for t in BAG_TOPICS),'Empty required topic')
    seen=defaultdict(set); errors=[]
    def receive(topic,msg):
        digest=hashlib.sha256(serialize_message(msg)).hexdigest()
        if digest not in expected[topic]: errors.append(topic)
        seen[topic].add(digest)
    for topic,(typ,_,_) in SPECS.items():
        node.create_subscription(typ,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
    with (args.evidence/'play.log').open('w') as log:
        player=subprocess.Popen(['ros2','bag','play',str(args.evidence/'sensors-bag'),'--rate','0.75','--delay','2'],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if player.poll() is not None:
                end=time.monotonic()+.5
                while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.01)
                break
        require(player.poll()==0,'Replay timed out/failed')
    require(not errors,'Replayed payload changed')
    for topic in SPECS:
        require(len(seen[topic])>=len(expected[topic])*.9,f'Replay message loss {topic}: {len(seen[topic])}/{len(expected[topic])}')
    for frame in ('lidar_link','imu_link','camera_optical_frame'):
        require(buffer.can_transform('odom',frame,Time()),f'Replay missing TF {frame}')
    result.update(status='PASS',bag_message_counts=dict(counts),
                  replay_unique_messages={t:len(v) for t,v in seen.items()},
                  recorded_unique_messages={t:len(v) for t,v in expected.items()},
                  measurement_span_s={t:(max(v)-min(v))*1e-9 for t,v in ranges.items()},
                  payloads_match_recording=True,isolated_ros_domain=43)
    subprocess.run(['ros2','bag','info',str(args.evidence/'sensors-bag')],stdout=(args.evidence/'bag-info.txt').open('w'),check=True)
except Exception as exc:
    result['error']=str(exc); raise
finally:
    stop_process(player); node.destroy_node(); rclpy.shutdown()
    (args.evidence/'replay-result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
