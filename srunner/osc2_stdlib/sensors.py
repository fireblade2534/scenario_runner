import carla
from scenerio_runner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider

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
        

class Lidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.ray_cast", transform, attributes)


class SemanticLidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.ray_cast_semantic", transform, attributes)


class Radar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.other.radar", transform, attributes)


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


class HybridSolidStateLidar(Sensor):
    def __init__(self, self_name: str, transform: carla.Transform, attributes: dict[str, str]) -> None:
        super().__init__(self_name, "sensor.lidar.hss_lidar", transform, attributes)