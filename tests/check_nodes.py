#!/usr/bin/python
import rclpy
from ros2node.api import get_node_names

expected = ["/list_elements",
            "/base_link_to_base_laser_ld06",
            "/base_link_to_imu",
            "/imu_complementary_filter",
            "/joy_node",
            "/mini_pupper_imu_driver_node",
            "/servo_interface",
            "/display_interface",
            "/v4l2_camera"]

rclpy.init()
node = rclpy.create_node("list_elements")
available_nodes = get_node_names(node=node, include_hidden_nodes=False)
found = []
for name, namespace, full_name in available_nodes:
    found.append(full_name)
if set(found) != set(expected):
    for entry in set(found).difference(set(expected)):
        print("unexpected node %s" % entry)
    for entry in set(expected).difference(set(found)):
        print("missing node %s" % entry)
    raise SystemExit("node list NOT OK")
node.destroy_node()
rclpy.shutdown()
print("node list OK")
