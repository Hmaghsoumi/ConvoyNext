class ROSArgs:
    def __init__(self, car_number, wheelbase, follow_distance=None, speed_max=None, speed_min=None, accel_max=None, accel_min=None, 
                 steer_max=None, steer_min=None, velocity_weight=None, broadcast_interval=None, listen_interval=None, drop_rate=None, 
                 track_name=None, center_lat=None, center_lon=None, center_orientation=None, heading_con_type=None, speed_con_type=None, 
                 I_max=None, alpha=None, P_Kp=None, I_Ki=None, PI_Kp=None, PI_Ki=None, PD_Kp=None, PD_Kd=None, PID_Kp=None, PID_Ki=None, 
                 PID_Kd=None, k=None, ks=None, save_path=None, **kwargs):

        self.car_number = car_number
        self.wheelbase = wheelbase
        self.follow_distance = follow_distance
        self.speed_max = speed_max
        self.speed_min = speed_min
        self.accel_max = accel_max
        self.accel_min = accel_min
        self.steer_max = steer_max
        self.steer_min = steer_min
        self.velocity_weight = velocity_weight
        self.broadcast_interval = broadcast_interval
        self.listen_interval = listen_interval
        self.drop_rate = drop_rate
        self.track_name = track_name
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.center_orientation = center_orientation
        self.heading_con_type = heading_con_type
        self.speed_con_type = speed_con_type
        self.I_max = I_max
        self.alpha = alpha
        self.P_Kp = P_Kp
        self.I_Ki = I_Ki
        self.PI_Kp = PI_Kp
        self.PI_Ki = PI_Ki
        self.PD_Kp = PD_Kp
        self.PD_Kd = PD_Kd
        self.PID_Kp = PID_Kp
        self.PID_Ki = PID_Ki
        self.PID_Kd = PID_Kd
        self.k = k    
        self.ks = ks  
        self.save_path = save_path
       
        for key, value in kwargs.items():
            setattr(self, key, value)

def setup_ros_args(**kwargs):
    return ROSArgs(**kwargs)
