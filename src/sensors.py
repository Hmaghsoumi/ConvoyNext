class SensorManager:
    def __init__(self):
        self.satellite = None
        self.telem = None
        self.heading = None
        self.imu = None

    def sensors_ok(self):
        return all(sensor is not None for sensor in [self.satellite, self.telem, self.heading, self.imu])

    def update_telem(self, msg):
        self.telem = msg

    def update_satellite(self, msg):
        self.satellite = msg

    def update_heading(self, msg):
        self.heading = msg

    def update_imu(self, msg):
        self.imu = msg
