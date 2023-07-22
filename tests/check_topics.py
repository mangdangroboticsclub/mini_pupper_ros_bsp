#!/usr/bin/python
import rclpy
from MangDang.mini_pupper.capabilities import Capabilities

expected_v1 = ["/camera_info",
               "/image_raw",
               "/image_raw/compressed",
               "/image_raw/compressedDepth",
               "/image_raw/theora",
               "/mini_pupper_lcd/image_raw",
               "/joint_group_effort_controller/joint_trajectory",
               "/joy",
               "/joy/set_feedback",
               "/parameter_events",
               "/rosout",
               "/tf_static"]

expected_v2 = ["/tf",
               "/imu/data",
               "/imu/data_raw"]

mp = Capabilities()
hw = mp.get_capability('version')

if hw == 'mini_pupper':
    expected = expected_v1
else:
    expected = expected_v1 + expected_v2

rclpy.init()
node = rclpy.create_node("list_elements")
available_topics = node.get_topic_names_and_types()
found = []
for name, topic_type in available_topics:
    found.append(name)
if set(found) != set(expected):
    for entry in set(found).difference(set(expected)):
        print("unexpected topic %s" % entry)
    for entry in set(expected).difference(set(found)):
        print("missing topic %s" % entry)
    raise SystemExit("topic list NOT OK")
node.destroy_node()
rclpy.shutdown()
print("topic list OK")
