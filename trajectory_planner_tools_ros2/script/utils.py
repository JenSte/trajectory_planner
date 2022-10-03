# This file contains code that is shared between the different nodes.

import math

import rclpy
import yaml

from rcl_interfaces.srv import GetParameters


def get_footprint(node, costmap_node):
    """Get the robot's footprint polygon from the costmap node."""

    # ros2 does not allow easy access to parameters of another node any more,
    # a service call to the other node has to be used to get it.

    service_name = f"{costmap_node}/get_parameters"
    service_client = node.create_client(GetParameters, service_name)

    while not service_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f"Waiting for service '{service_name}'...")

    request = GetParameters.Request()
    request.names.append("footprint")
    future = service_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(f"Error getting footprint parameter: {e}")

    try:
        string_value = response.values[0].string_value
        return yaml.safe_load(string_value)
    except Exception as e:
        raise RuntimeError(f"Error reading service response: {e}")


def estimate_wheel_distance(footprint):
    """Estimate the distance between the robot's wheels from the footprint."""

    distance = 0.05

    for x, y in footprint:
        distance = max(distance, math.sqrt(x**2 + y**2))

    return distance * 2.0 / 3.0
