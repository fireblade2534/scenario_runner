import carla
import roslibpy
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.util import bbox_orientation_to_sensor, object_to_sensor

ROS_PUBLISHER = roslibpy.Ros(host="localhost", port=9080)

class Sensor:
    def __init__(self, self_name: str, type: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        self.self_name = self_name          # Sensor name
        self.type = type                    # Sensor type
        self.transform = transform          # Relative transform of the sensor
        self.attributes = attributes        # Sensor attributes

        self.sensor_bp = []                 # Sensor blueprints
        self.sensors = []                   # Sensors

        self.parent_name = ""               # Sensor parent name

    def setup(self, parent: carla.Actor):
        self.parent_name = parent.attributes.get("role_name")

        self.sensor_bp.append(CarlaDataProvider.get_blueprint_library().find(self.type))

        for attribute, value in self.attributes.items():
            self.sensor_bp[0].set_attribute(str(attribute), str(value))

        self.sensor_bp[0].set_attribute("ros_name", self.self_name)

        self.sensors.append(CarlaDataProvider.spawn_actor(
            self.sensor_bp[0],
            self.transform,
            must_spawn=True,
            attach_to=parent
        ))
        self.sensors[0].enable_for_ros()

class RGBCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.rgb", transform, attributes)


class DepthCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.depth", transform, attributes)


class SemanticSegmentationCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.semantic_segmentation", transform, attributes)


class InstanceSegmentationCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.instance_segmentation", transform, attributes)


class DVSCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.dvs", transform, attributes)


class OpticalFlowCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.optical_flow", transform, attributes)


class NormalsCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.normals", transform, attributes)


class WideAngleRGBCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.rgb.wide_angle_lens", transform, attributes)


class WideAngleDepthCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.depth.wide_angle_lens", transform, attributes)


class WideAngleSemanticSegmentationCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.semantic_segmentation.wide_angle_lens", transform, attributes)


class WideAngleInstanceSegmentationCamera(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.camera.instance_segmentation.wide_angle_lens", transform, attributes)
        

class GNSS(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.gnss", transform, attributes)


class IMU(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.imu", transform, attributes)


class CollisionDetector(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.collision", transform, attributes)


class LaneInvasionDetector(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.lane_invasion", transform, attributes)


class ObstacleDetector(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.obstacle", transform, attributes)


class Lidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.ray_cast", transform, attributes)


class SemanticLidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.ray_cast_semantic", transform, attributes)


class HybridSolidStateLidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.hss_lidar", transform, attributes)


class Radar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.radar", transform, attributes)


class SemanticRadar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.radar_semantic", transform, attributes)
        self.required_hits = 5

    def _on_lidar(self, data: carla.SemanticLidarMeasurement):
        lidar_frame = data.frame
        current_frame = CarlaDataProvider.get_world().get_snapshot().frame

        if lidar_frame != current_frame:
            print(
                f"Frame mismatch! "
                f"Semantic Radar: {lidar_frame}, "
                f"World: {current_frame}"
            )
            
        tracks_hits = {}
        tracks = []

        parent = CarlaDataProvider.get_actor_by_id(self.parent_id)

        if parent is None:
            print(f"Semantic radar parent {self.parent_id} not found")
            return

        for hits in data:
            if hits.object_tag in {12, 14, 15, 16, 17, 18, 19}:
                if hits.object_idx in tracks_hits:
                    tracks_hits[hits.object_idx][0] += 1
                else:
                    tracks_hits[hits.object_idx] = [1, hits.object_tag]

        for object_id, object_data in tracks_hits.items():
            if object_data[0] < self.required_hits:
                continue

            actor = CarlaDataProvider.get_actor_by_id(object_id)
            if actor is None or actor.id == self.parent_id:
                continue

            sensor_relative = object_to_sensor(actor, parent, data.transform)

            pos = sensor_relative["position"]
            vel = sensor_relative["velocity"]
            size = sensor_relative["size"]

            orientation = bbox_orientation_to_sensor(
                actor, data.transform
            )

            tracks.append({
                "object_idx": int(object_id),
                "object_tag": int(object_data[1]),

                "bbox": {
                    "center": {
                        "position": {
                            "x": float(pos.x),
                            "y": float(-pos.y),
                            "z": float(pos.z)
                        },
                        "orientation": orientation
                    },
                    "size": {
                        "x": float(size.x),
                        "y": float(size.y),
                        "z": float(size.z)
                    }
                },

                "velocity": {
                    "x": float(vel.x),
                    "y": float(-vel.y),
                    "z": float(vel.z)
                }
            })


        total_ns = round(data.timestamp * 1_000_000_000)
        sec, nanosec = divmod(total_ns, 1_000_000_000)

        message = roslibpy.Message({
            "header": {
                "stamp": {
                    "sec": int(sec),
                    "nanosec": int(nanosec)
                },
                "frame_id": self.self_name
            },
            "tracks": tracks
        })

        self.publisher.publish(message)



    def setup(self, parent: carla.Actor):
        global ROS_PUBLISHER

        self.parent_name = parent.attributes.get("role_name")
        self.parent_id = parent.id

        self.sensor_bp.append(CarlaDataProvider.get_blueprint_library().find("sensor.lidar.ray_cast_semantic"))

        for attribute, value in self.attributes.items():
            match attribute:
                case "required_hits":
                    self.required_hits = int(value)
                case "range":
                    self.sensor_bp[0].set_attribute("range", str(value))
                case "sensor_tick":
                    self.sensor_bp[0].set_attribute("sensor_tick", str(value))
                case "points_per_second":
                    self.sensor_bp[0].set_attribute("points_per_second", str(value))
                case "horizontal_fov":
                    self.sensor_bp[0].set_attribute("horizontal_fov", str(value))
                case "vertical_fov":
                    fov = float(value)

                    self.sensor_bp[0].set_attribute("upper_fov", str(fov / 2.0))
                    self.sensor_bp[0].set_attribute("lower_fov", str(fov / -2.0))
                case _:
                    raise Exception

        settings = CarlaDataProvider.get_world().get_settings()
        if settings.fixed_delta_seconds is not None and settings.fixed_delta_seconds > 0:
            self.sensor_bp[0].set_attribute("rotation_frequency", str(1.0 / settings.fixed_delta_seconds))
        else:
            raise Exception("Semantic radar cannot be used without fixed timesteps")

        if not ROS_PUBLISHER.is_connected:
            print("Connecting to the ros publisher")
            ROS_PUBLISHER.run()

        self.publisher = roslibpy.Topic(
            ROS_PUBLISHER,
            f"/carla/{self.parent_name}/{self.self_name}",
            "carla_custom_interfaces/msg/SemanticRadarMeasurement"
        )
        self.publisher.advertise()

        self.sensors.append(CarlaDataProvider.spawn_actor(
            self.sensor_bp[0],
            self.transform,
            must_spawn=True,
            attach_to=parent
        ))
        self.sensors[0].listen(lambda data: self._on_lidar(data))