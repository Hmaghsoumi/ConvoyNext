import socket
import struct
import random
import numpy as np
from collections import deque, defaultdict
from time import time

MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4

EARTH_RADIUS = 6371e3
DUE_EAST = 90

class Control:
    def __init__(self, rosargs, *args):
        self.args = rosargs
        self.datapoints = []
        self.error_buffer = deque(maxlen=10)
        self.car_positions = defaultdict(list)
        self.mission_status = MISSIONSTART

        # set up UDP communication
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def sensors_ok(self):
        return self.satellite is not None and self.telem is not None and self.heading is not None and self.imu is not None
    
    def should_broadcast(self):
        return True

    def should_listen(self, sender_number):
        return sender_number < self.args.car_number

    def _disarm(self):
        pass

    def _broadcast(self):
        if self.satellite is None or self.heading is None or (self.args.drop_rate > 0 and random.random() < self.args.drop_rate):
            return

        if self.should_broadcast():
            msg = self.state.to_json()
            msg = msg.encode()
            self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def _listen(self):
        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())

        bsm_message = BasicSafetyMessage(**data_json)

        if bsm_message.event_flags['abort'] and not self.mission_status in [DISARMING, MISSIONCOMPLETE]:
            self._disarm()

        if self.should_listen(bsm_message.event_flags['car']):
            self.car_positions[bsm_message.event_flags['car']].append(bsm_message)
            self.car_positions[bsm_message.event_flags['car']] = self.car_positions[bsm_message.event_flags['car']][-4:]

def initialize_control(args):
    return Control(args)
