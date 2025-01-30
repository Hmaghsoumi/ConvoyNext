import math

def coords_to_local(self, target_lat, target_lon):
    """  Converts GPS coordinates to local cartesian coordinates with respect to the track center point. """

    # Convert degrees to radians
    phi1 = math.radians(self.args.center_lat)
    lambda1 = math.radians(self.args.center_lon)
    phi2 = math.radians(target_lat)
    lambda2 = math.radians(target_lon)

    # Equirectangular approximation to convert to local Cartesian coordinates
    x = EARTH_RADIUS * math.cos((phi1 + phi2) / 2) * (lambda2 - lambda1) 
    y = EARTH_RADIUS * (phi2 - phi1)

    # Adjust for the center orientation angle  (Center orientation is the angle of y axis of local cartesian) 
    theta_rad = math.radians(self.args.center_orientation - DUE_EAST)    
    qx = math.cos(theta_rad) * x - math.sin(theta_rad) * y
    qy = math.sin(theta_rad) * x + math.cos(theta_rad) * y

    return qx, qy
