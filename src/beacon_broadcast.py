import random
import socket
import struct
import pyproj
import json
import math
from time import time, sleep
import argparse
import threading
import numpy as np
import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, os.pardir)))
from crossplatform import BasicSafetyMessage

# constants
EARTH_RADIUS = 6371e3
DUE_EAST = 90.0
ABORT = False

# set up for broadcast
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

# Global geodesic
geodesic = pyproj.Geod(ellps='WGS84')

# set up args
parser = argparse.ArgumentParser()
parser.add_argument('--track_name', type=str, default="../tracks/garage_c_loop_small.json")
parser.add_argument('--closed_loop', type=str, default=True)
parser.add_argument('--broadcast_interval', type=float, default=0.1)
parser.add_argument('--drop_rate', type=float, default=0.0)


def calculate_straight(start, end, seg_dist, speed):
    """
    Generates a sequence of waypoints along a 'straight' path
    by applying the same initial bearing at each step (rhumb-like).
    
    Parameters:
        start: (lat_start, lon_start)
        end:   (lat_end, lon_end)
        seg_dist: segment distance (meters)
        speed: speed (m/s or any unit you prefer)
    
    Returns:
        segs:    list of (lat, lon, bearing, speed) for each waypoint
        bearing: the initial bearing used
    """
    # initialization
    leftover_threshold=0.05

    # Unpack start & end
    lat_s, lon_s = start
    lat_e, lon_e = end

    # Compute the initial bearing and the great-circle distance
    bearing, _, total_dist = geodesic.inv(lon_s, lat_s, lon_e, lat_e)
    bearing = bearing % 360
    
    # Determine how many full segments fit in total_dist
    seg_count = int(total_dist // seg_dist)

    # Initialize list of waypoints
    segs = [(lat_s, lon_s, bearing, speed)]

    # Step 1: Add full seg_dist steps
    for _ in range(seg_count):
        lat_prev, lon_prev, _, _ = segs[-1]
        # Use pyproj.Geod.fwd with (lon, lat) order
        lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing, seg_dist)
        segs.append((lat_next, lon_next, bearing, speed))

    # Step 2: Handle leftover distance (if total_dist not multiple of seg_dist)
    leftover = total_dist - seg_count * seg_dist
    if leftover > leftover_threshold:
        lat_prev, lon_prev, _, _ = segs[-1]
        lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing, leftover)
        segs.append((lat_next, lon_next, bearing, speed))

    return segs, bearing


def calculate_turn(start, start_bearing, end, end_bearing, seg_dist, speed, direction='left'):
    """
    Generates a smooth geodesic turn from start to end bearings, 
    using standard compass headings:
      - 0° = North, 90° = East, 180° = South, 270° = West
      - Right turn => heading increases (+)
      - Left turn  => heading decreases (-)
    Clamps turn angles to 180° max by default.
    """

    # Normalize bearings to [0, 360)
    start_bearing = start_bearing % 360
    end_bearing   = end_bearing % 360

    #----------------------------------------------------------------
    # 1) Compute raw turn angle based on *forced* direction
    #----------------------------------------------------------------
    if direction == 'left':
        # Left turn in standard compass means heading should go down
        # If we do "start - end" we get how many degrees we must rotate CCW (which is 'left').
        turn_angle = (start_bearing - end_bearing) % 360
    else:  # direction == 'right'
        # Right turn means heading goes up
        # end_bearing - start_bearing is how many degrees we must rotate CW (which is 'right').
        turn_angle = (end_bearing - start_bearing) % 360

    #----------------------------------------------------------------
    # 2) Approximate turn radius via chord length
    #----------------------------------------------------------------
    # chord_length = geodesic distance between start and end positions
    _, _, chord_length = geodesic.inv(start[1], start[0], end[1], end[0])
    # radius ~ chord_length / (2 * sin(turn_angle/2))
    radius = chord_length / (2 * math.sin(math.radians(turn_angle / 2.0)))

    #----------------------------------------------------------------
    # 3) Determine step size in degrees for each segment
    #----------------------------------------------------------------
    # arc_length = radius * turn_angle_in_radians
    # segment_count ≈ arc_length / seg_dist
    # angle_step_in_radians = seg_dist / radius
    angle_step = math.degrees(seg_dist / radius)

    #----------------------------------------------------------------
    # 4) Build the waypoint list
    #----------------------------------------------------------------
    segs = [(start[0], start[1], start_bearing, speed)]
    current_bearing = start_bearing

    while True:
        # How many degrees remain (in the forced direction sense)?
        # We'll measure the difference in "the same sense" we used above.
        if direction == 'left':
            # If we're forcing left, measure how far we still have to turn "left".
            remaining = (current_bearing - end_bearing) % 360
        else:  # 'right'
            # If we're forcing right, measure how far we still have to turn "right".
            remaining = (end_bearing - current_bearing) % 360

        # If the remaining is less than one step, we're done.
        if remaining < angle_step:
            current_bearing = end_bearing
            # Make sure we use the final position precisely
            lon_next, lat_next, _ = geodesic.fwd(
                segs[-1][1], segs[-1][0], current_bearing, seg_dist
            )
            segs.append((lat_next, lon_next, current_bearing, speed))
            break

        # Otherwise, move one step
        if direction == 'left':
            current_bearing = (current_bearing - angle_step) % 360
        else:
            current_bearing = (current_bearing + angle_step) % 360

        # Compute new geodesic position from the last waypoint
        lon_next, lat_next, _ = geodesic.fwd(
            segs[-1][1], segs[-1][0],
            current_bearing, seg_dist
        )
        segs.append((lat_next, lon_next, current_bearing, speed))

    return segs


def calculate_track(straights, turns, broadcast_interval, closed_loop):
    """
    Calculates a full track from lists of straights and turns.
    
    :param straights: list of dicts, each with:
        {
          'start': {'lat': float, 'lon': float},
          'end':   {'lat': float, 'lon': float},
          'speed': float,
        }
    :param turns: list of dicts, each with:
        {
          'start': {'lat': float, 'lon': float},
          'end':   {'lat': float, 'lon': float},
          'speed': float,
          'direction': 'left' or 'right',
        }
    :param broadcast_interval: time step between waypoints (seconds)
    :param closed_loop: if True, automatically wrap the last bearing to the first
    :return: a list of waypoints (lat, lon, bearing, speed)
    """
    straight_points = []
    bearings = []

    #---------------------------------------------------
    # 1) Generate each straight segment & store bearings
    #---------------------------------------------------
    for straight in straights:
        seg_dist = broadcast_interval * straight['speed']
        start = (straight['start']['lat'], straight['start']['lon'])
        end   = (straight['end']['lat'],   straight['end']['lon'])

        segs, bearing = calculate_straight(start, end, seg_dist, straight['speed'])
        straight_points.append(segs)
        bearings.append(bearing)

    #---------------------------------------------------------
    # 2) Pair consecutive bearings for each turn's (start->end)
    #    - If closed_loop=True, wrap last bearing to first
    #---------------------------------------------------------
    bearing_pairs = []
    if closed_loop and len(bearings) > 1:
        # Wrap around: from b[i] to b[i+1], and finally b[-1] -> b[0]
        bearing_pairs = [(bearings[i], bearings[(i+1) % len(bearings)])
                         for i in range(len(bearings))]
    else:
        # Open track: only pair up to the second-last bearing
        for i in range(len(bearings) - 1):
            bearing_pairs.append((bearings[i], bearings[i+1]))

    #--------------------------------------------
    # 3) Generate turn segments for each bearing pair
    #    Only process as many turns as we have bearing pairs
    #--------------------------------------------
    turn_points = []
    n = min(len(turns), len(bearing_pairs))
    for i in range(n):
        turn = turns[i]
        start_bearing, end_bearing = bearing_pairs[i]
        seg_dist = broadcast_interval * turn['speed']

        start = (turn['start']['lat'], turn['start']['lon'])
        end   = (turn['end']['lat'],   turn['end']['lon'])

        segs = calculate_turn(
            start,
            start_bearing,
            end,
            end_bearing,
            seg_dist,
            turn['speed'],
            turn['direction']
        )
        turn_points.append(segs)

    #----------------------------------------------------
    # 4) Combine straights + turns in an interleaved way
    #    up to the smallest matching count (n)
    #----------------------------------------------------
    full_track = []
    for i in range(n):
        if i>0:
            previous_turn_segment = turn_points[i-1]
            straight_segment = straight_points[i]
            
            # Check the last point of the turn vs. the first point of the straight
            if straight_segment and previous_turn_segment:
                last_turn = previous_turn_segment[-1]
                first_straight = straight_segment[0]
                
                # Compare positions with a small distance threshold
                if distance_between_points(last_turn, first_straight) < 0.05:
                    # If they're essentially the same, remove the first point of straight
                    # to prevent duplicates in the final track
                    straight_segment = straight_segment[1:]
    
        elif i==0:
            straight_segment = straight_points[i]
            
        turn_segment = turn_points[i]
        
        # Check the last point of the straight vs. the first point of the turn
        if straight_segment and turn_segment:
            last_straight = straight_segment[-1]
            first_turn = turn_segment[0]
            
            # Compare positions with a small distance threshold
            if distance_between_points(last_straight, first_turn) < 0.05:
                # If they're essentially the same, remove the first point of turn
                # to prevent duplicates in the final track
                turn_segment = turn_segment[1:]
        
        # Now concatenate
        full_track += straight_segment + turn_segment

    # If we have leftover straights (i.e., #straights > #turns),
    # append them at the end.
    # Typically for an open track, you might have one more straight
    # after the final turn.
    if len(straight_points) > n:
        for leftover_i in range(n, len(straight_points)):
            full_track += straight_points[leftover_i]

    # Similarly, if we have leftover turns (rare in practice),
    # you could add them here, though normally you wouldn't have a turn
    # that doesn't follow a straight in most path definitions.
    # if len(turn_points) > n:
    #     for leftover_i in range(n, len(turn_points)):
    #         full_track += turn_points[leftover_i]

    return full_track


def distance_between_points(wp1, wp2):
    """
    Returns approximate distance between
    (lat1, lon1, _, _), (lat2, lon2, _, _) in meters.
    """
    lat1, lon1, *_ = wp1
    lat2, lon2, *_ = wp2
    # Quick geodesic distance:
    _, _, dist = geodesic.inv(lon1, lat1, lon2, lat2)
    return dist


def listen_for_stop():
    global ABORT
    while True:
        input() # wait for the user to press enter
        ABORT = True


if __name__ == '__main__':
    args = parser.parse_args()
    with open(args.track_name, 'r') as f:
        track = json.load(f)

    full_track = calculate_track(track['straights'], track['turns'], args.broadcast_interval, args.closed_loop)

    stop_thread = threading.Thread(target=listen_for_stop)
    stop_thread.daemon = True
    stop_thread.start()

    for (lat, lon, head, speed) in full_track:
        # Check abort condition
        if ABORT:
            print("Stop signal received. Exiting loop.")
            break
        
        # randomly drop packets
        if args.drop_rate > 0 and random.random() < args.drop_rate:
            sleep(args.broadcast_interval)
            continue

        # send the packet
        bsm_msg = BasicSafetyMessage(
            time=time(),
            latitude=round(lat, 8),
            longitude=round(lon, 8),
            speed=speed,
            heading=round(head, 2),
            acceleration=0,
            event_flags={
                "abort": False,
                "car": 0,
            }
        )
        msg = bsm_msg.to_json()
        print(msg)
        msg = msg.encode()
        sock.sendto(msg, ('224.0.0.1', 5004))

        # wait for next broadcast interval
        sleep(args.broadcast_interval)
