import carla

from srunner.osc2_stdlib.sensors import SemanticRadar, Sensor
import srunner.osc2_stdlib.misc_object as misc
import srunner.scenariomanager.carla_data_provider as carla_provider
from srunner.tools.osc2_helper import OSC2Helper


class Vehicle:
    def __init__(self) -> None:
        self.model = "vehicle.*"  # vehicle.tesla.model3
        self.rolename = "scenario"  # variable name
        self.position = misc.Position()  # doesn't depend on the position of
        self.transform = carla.Transform()  # initial position
        self.speed = 0  # the initial speed, m/s，default 0
        self.autopilot = False
        self.random_location = (
            True  # TODO: Random location that may conflict with map path constraints
        )
        self.color = None
        self.category = "car"  # vehicle or pedestrian, vehicleCategory  car
        self.args = dict()
        self.sensors = []

    def set_arg(self, kw):
        self.args.update(kw)

    def get_arg(self, key):
        return self.args[key]

    def set_name(self, name: str) -> None:
        self.rolename = name

    def get_name(self) -> str:
        return self.rolename

    def set_model(self, model: str) -> None:
        self.model = model

    def set_position(self, pos: misc.Position) -> None:
        self.position = pos
        # convert position to transform，reference convert_position_to_transform function
        if type(pos) is misc.WorldPosition:
            x = float(pos.x)
            y = float(pos.y)
            z = float(pos.z)
            # yaw = math.degrees(float(pos.h))
            # pitch = math.degrees(float(pos.p))
            # roll = math.degrees(float(pos.r))
            yaw = float(pos.yaw)
            pitch = float(pos.pitch)
            roll = float(pos.roll)
            # if not OpenScenarioParser.use_carla_coordinate_system:
            #     y = y * (-1.0)
            #     yaw = yaw * (-1.0)
            self.transform = carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(yaw=yaw, pitch=pitch, roll=roll),
            )
        elif type(pos) is misc.LanePosition:
            pass
        else:
            print("no implement position type")

    def get_transform(self) -> carla.Transform:
        if OSC2Helper.wait_for_ego and self.rolename == OSC2Helper.ego_name:
            actor = carla_provider.CarlaDataProvider.get_actor_by_name(self.rolename)
            ret = carla_provider.CarlaDataProvider.get_transform(actor)
            return ret
        if self.args.get("init_transform"):
            return self.args["init_transform"]

        actor = carla_provider.CarlaDataProvider.get_actor_by_name(self.rolename)
        ret = carla_provider.CarlaDataProvider.get_transform(actor)
        return ret

    def set_sensors(self, sensors: list[Sensor]):
        self.sensors = sensors

    def get_sensors(self) -> list[Sensor]:
        return self.sensors

    def setup_sensors(self, parent: carla.Actor):
        for sensor in self.sensors:
            sensor.setup(parent)

class Car(Vehicle):
    def __init__(self) -> None:
        super().__init__()


class Model3(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.tesla.model3")


class Mkz2017(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.lincoln.mkz_2017")

class Carlacola(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.carlamotors.carlacola")


class Rubicon(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.jeep.wrangler_rubicon")


class AudiA2(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.audi.a2")


class AudiEtron(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.audi.etron")


class AudiTT(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.audi.tt")


class BmwGrandtourer(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.bmw.grandtourer")


class ChevroletImpala(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.chevrolet.impala")


class CitroenC3(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.citroen.c3")


class DodgeCharger2020(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.dodge.charger_2020")


class DodgeChargerPolice(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.dodge.charger_police")


class DodgeChargerPolice2020(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.dodge.charger_police_2020")


class FordCrown(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.ford.crown")


class FordMustang(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.ford.mustang")


class LincolnMkz2020(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.lincoln.mkz_2020")


class MercedesCoupe(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mercedes.coupe")


class MercedesCoupe2020(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mercedes.coupe_2020")


class MicroMicrolino(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.micro.microlino")


class MiniCooperS(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mini.cooper_s")


class MiniCooperS2021(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mini.cooper_s_2021")


class NissanMicra(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.nissan.micra")


class NissanPatrol(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.nissan.patrol")


class NissanPatrol2021(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.nissan.patrol_2021")


class SeatLeon(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.seat.leon")


class ToyotaPrius(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.toyota.prius")


class CarlaMotorsEuropeanHGV(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.carlamotors.european_hgv")


class CarlaMotorsFiretruck(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.carlamotors.firetruck")


class TeslaCybertruck(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.tesla.cybertruck")


class FordAmbulance(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.ford.ambulance")


class MercedesSprinter(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mercedes.sprinter")


class VolkswagenT2(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.volkswagen.t2")


class VolkswagenT22021(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.volkswagen.t2_2021")


class MitsubishiFusorosa(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.mitsubishi.fusorosa")


class HarleyDavidsonLowRider(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.harley-davidson.low_rider")


class KawasakiNinja(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.kawasaki.ninja")


class VespaZX125(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.vespa.zx125")


class YamahaYZF(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.yamaha.yzf")


class BhCrossbike(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.bh.crossbike")


class DiamondbackCentury(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.diamondback.century")


class GazelleOmafiets(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.gazelle.omafiets")


class Ego(Car):
    def __init__(self) -> None:
        super().__init__()
        self.set_model("vehicle.tesla.model3")
        self.set_sensors([
            SemanticRadar("front_radar", carla.Transform(carla.Location(x=1.5, z=2.2)), {"vertical_fov": 30, "horizontal_fov": 30, "range": 60})
        ])


