#!/usr/bin/python
import rclpy

expected = ["/camera_info",
            "/image_raw",
            "/image_raw/compressed",
            "/image_raw/compressedDepth",
            "/image_raw/theora",
            "/mini_pupper_lcd/image_raw",
            "/imu/data",
            "/imu/data_raw",
            "/joint_group_effort_controller/joint_trajectory",
            "/joy",
            "/joy/set_feedback",
            "/parameter_events",
            "/rosout",
            "/tf",
            "/tf_static"]

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
