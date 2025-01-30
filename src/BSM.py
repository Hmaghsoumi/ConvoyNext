import json

class BasicSafetyMessage:
    """Implements the J2735 Basic Safety Message (BSM) data structure."""

    def __init__(self, time=None, latitude=None, longitude=None, elevation=None, position_accuracy=None,
                 speed=None, heading=None, acceleration=None, yaw_rate=None, steering_wheel_angle=None,
                 transmission_state=None, brake_system_status=None, vehicle_length=None, vehicle_width=None,
                 path_history=None, path_prediction=None, exterior_lights=None, event_flags=None):
        self.time = time
        self.latitude = latitude
        self.longitude = longitude
        self.elevation = elevation
        self.position_accuracy = position_accuracy
        self.speed = speed
        self.heading = heading
        self.acceleration = acceleration
        self.yaw_rate = yaw_rate
        self.steering_wheel_angle = steering_wheel_angle
        self.transmission_state = transmission_state
        self.brake_system_status = brake_system_status
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.path_history = path_history if path_history is not None else []
        self.path_prediction = path_prediction if path_prediction is not None else []
        self.exterior_lights = exterior_lights if exterior_lights is not None else []
        self.event_flags = event_flags if event_flags is not None else {}

    def to_json(self):
        return json.dumps(self.__dict__)

def create_bsm(**kwargs):
    return BasicSafetyMessage(**kwargs)
