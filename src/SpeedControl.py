import numpy as np

def speed_controller(self, v_ego, v_desired, dt):
    """   Calculates accelaration using various speed controllers.  """

    # Calculate velocity error
    error = v_desired - v_ego

    # Append current error to the error buffer
    self.error_buffer.append(error)

    # Calculate derivative and integral of error if sufficient buffer length
    if len(self.error_buffer) >= 2:
        I_error = sum(self.error_buffer) * dt
        I_error = np.clip(I_error, -self.args.I_max, self.args.I_max)
        if dt == 0:
            D_error = 0
        else:
            raw_D_error = (self.error_buffer[-1] - self.error_buffer[-2]) / dt
            if not hasattr(self, 'previous_D_error'):
                # Initialize previous_D_error on the first run
                self.previous_D_error = raw_D_error

            # Smooth the derivative
            D_error = self.args.alpha * raw_D_error + (1 - self.args.alpha) * self.previous_D_error
            self.previous_D_error = D_error
            
    else:
        I_error = 0
        D_error = 0
        self.previous_D_error = None  # Safe fallback for early iterations

    # Select acceleration control law based on controller type  
    if self.args.speed_con_type == 'P':
        accel = self.args.P_Kp * error
    elif self.args.speed_con_type == 'I':
        accel = self.args.I_Ki * I_error
    elif self.args.speed_con_type == 'PI':
        accel = (self.args.PI_Kp * error) + (self.args.PI_Ki * I_error)
    elif self.args.speed_con_type == 'PD':
        accel = (self.args.PD_Kp * error) + (self.args.PD_Kd * D_error)
    elif self.args.speed_con_type == 'PID':
        accel = (self.args.PID_Kp * error) + (self.args.PID_Ki * I_error) + (self.args.PID_Kd * D_error)
    else:
        raise ValueError("Invalid controller type")
        
    accel = np.clip(accel, self.args.accel_min, self.args.accel_max)
    print(f"v_ego: {v_ego}, v_desired: {v_desired}, accel: {accel}")

    return accel
