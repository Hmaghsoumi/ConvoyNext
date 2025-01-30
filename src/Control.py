from sensor_manager import SensorManager
from communication import CommunicationManager
from speed_controller import SpeedController
from heading_controller import HeadingController

class Control:
    def __init__(self, rosargs):
        self.args = rosargs
        self.sensor_manager = SensorManager()
        self.comm_manager = CommunicationManager(self)
        self.speed_controller = SpeedController(self)
        self.heading_controller = HeadingController(self)

    def update_sensors(self, telem, satellite, heading, imu):
        self.sensor_manager.update_telem(telem)
        self.sensor_manager.update_satellite(satellite)
        self.sensor_manager.update_heading(heading)
        self.sensor_manager.update_imu(imu)

    def run(self):
        """ Main loop for running control logic """
        while True:
            self.comm_manager.listen()
            self.comm_manager.broadcast()
