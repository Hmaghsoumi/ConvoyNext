import socket
import struct
import json
import numpy as np

class CommunicationManager:
    def __init__(self, base_control):
        self.base_control = base_control
        self.beacon_started = False
        self._setup_udp()

    def _setup_udp(self):
        """ Sets up UDP sockets for communication. """
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def should_broadcast(self):
        return True

    def should_listen(self, sender_number):
        return sender_number < self.base_control.args.car_number

    def broadcast(self):
        if not self.base_control.sensor_manager.satellite or not self.base_control.sensor_manager.heading:
            return
        if self.base_control.args.drop_rate > 0 and np.random.random() < self.base_control.args.drop_rate:
            return

        if self.should_broadcast():
            msg = json.dumps(self.base_control.state.to_dict()).encode()
            self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def listen(self):
        data, _ = self.listen_sock.recvfrom(1024)
        message = json.loads(data.decode())
        sender_number = message.get("car_number")

        if self.should_listen(sender_number):
            self.base_control.car_positions[sender_number].append(message)
            self.base_control.car_positions[sender_number] = self.base_control.car_positions[sender_number][-4:]

        if len(self.base_control.car_positions[0]) >= 2:
            self.beacon_started = True
