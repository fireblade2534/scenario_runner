import math

import carla
import numpy as np
from scipy.spatial.transform import Rotation


def strtobool(val):
    """Convert a string representation of truth to true (1) or false (0).

    True values are 'y', 'yes', 't', 'true', 'on', and '1'; false values
    are 'n', 'no', 'f', 'false', 'off', and '0'.  Raises ValueError if
    'val' is anything else.
    """
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    else:
        raise ValueError(f"invalid truth value {val!r}")

def to_radians(vector: carla.Vector3D):
    return carla.Vector3D(
        math.radians(vector.x),
        math.radians(vector.y),
        math.radians(vector.z)
    )

def object_to_sensor(actor: carla.ActorSnapshot, actor_bounding: carla.BoundingBox, ego: carla.ActorSnapshot, sensor_transform: carla.Transform):
    actor_transform = actor.get_transform()

    center_world = carla.Location(
        x=actor_bounding.location.x,
        y=actor_bounding.location.y,
        z=actor_bounding.location.z,
    )
    actor_transform.transform(center_world)

    pos = carla.Location(
        x=center_world.x,
        y=center_world.y,
        z=center_world.z,
    )
    sensor_transform.inverse_transform(pos)

    ego_omega = to_radians(ego.get_angular_velocity())
    sensor_offset = sensor_transform.location - ego.get_transform().location
    sensor_vel = ego.get_velocity() + ego_omega.cross(sensor_offset)

    actor_omega = to_radians(actor.get_angular_velocity())
    center_offset = center_world - actor_transform.location
    object_vel = actor.get_velocity() + actor_omega.cross(center_offset)

    relative_vel = object_vel - sensor_vel

    vx = relative_vel.dot(sensor_transform.get_forward_vector())
    vy = relative_vel.dot(sensor_transform.get_right_vector())
    vz = relative_vel.dot(sensor_transform.get_up_vector())

    return {
        "position": pos,
        "velocity": carla.Vector3D(vx, vy, vz),
        "size": actor_bounding.extent * 2,
    }

# Written by Chat GPT 6
def bbox_orientation_to_sensor(bounding_box: carla.BoundingBox, actor_transform: carla.Transform, sensor_transform: carla.Transform):
    world_to_sensor_matrix = np.array(
        sensor_transform.get_inverse_matrix()
    )

    actor_to_world_matrix = np.array(
        actor_transform.get_matrix()
    )

    bounding_box_to_actor_matrix = np.array(
        carla.Transform(
            bounding_box.location,
            bounding_box.rotation
        ).get_matrix()
    )

    bounding_box_to_sensor_matrix = (
        world_to_sensor_matrix
        @ actor_to_world_matrix
        @ bounding_box_to_actor_matrix
    )

    carla_to_ros_matrix = np.diag([1.0, -1.0, 1.0])

    bounding_box_rotation_carla = bounding_box_to_sensor_matrix[:3, :3]

    bounding_box_rotation_ros = (
        carla_to_ros_matrix
        @ bounding_box_rotation_carla
        @ carla_to_ros_matrix
    )

    quaternion_x, quaternion_y, quaternion_z, quaternion_w = (
        Rotation.from_matrix(bounding_box_rotation_ros).as_quat()
    )

    return {
        "x": float(quaternion_x),
        "y": float(quaternion_y),
        "z": float(quaternion_z),
        "w": float(quaternion_w)
    }