"""Minimal RobStride private-protocol driver (CAN 2.0, 29-bit IDs, 1 Mbit/s) over python-can.

Constants are from the RS00/RS02/RS04 user manuals in ~/Developer/robstride_assets
(communication types 1-4, 6, 17, 18). MIT-style operation control only: one frame carries
position, velocity, kp, kd and feed-forward torque; every command returns a type-2 feedback frame.

Any python-can interface works. On macOS the simplest is a candleLight / gs_usb adapter
(interface='gs_usb'); an slcan adapter is interface='slcan', channel='/dev/tty.usbmodem...'.
"""
import math
import struct
import time

import can

# model: (torque range N.m, velocity range rad/s, kp max, kd max)
RANGE = {'RS04': (120.0, 15.0, 5000.0, 100.0), 'RS02': (17.0, 44.0, 500.0, 5.0),
         'RS00': (14.0, 33.0, 500.0, 5.0), 'EL05': (6.0, 50.0, 500.0, 5.0)}
P_MAX = 4 * math.pi
HOST = 0xFD


def u16(x, lo, hi):
    x = min(max(x, lo), hi)
    return int((x - lo) / (hi - lo) * 65535)


def f16(v, lo, hi):
    return lo + v / 65535 * (hi - lo)


class Motor:
    def __init__(self, bus, can_id, model):
        self.bus, self.id, self.model = bus, can_id, model
        self.T, self.V, self.KP, self.KD = RANGE[model]
        self.pos = self.vel = self.tau = self.temp = None
        self.fault = 0

    def _send(self, typ, mid16, data=bytes(8)):
        self.bus.send(can.Message(arbitration_id=(typ << 24) | (mid16 << 8) | self.id,
                                  data=data, is_extended_id=True))
        return self._recv()

    def _recv(self, timeout=0.02):
        end = time.time() + timeout
        while time.time() < end:
            m = self.bus.recv(timeout=max(end - time.time(), 0))
            if m is None:
                break
            typ = m.arbitration_id >> 24
            if typ == 2 and (m.arbitration_id >> 8) & 0xFF == self.id:
                p, v, t, tc = struct.unpack('>HHHH', bytes(m.data))
                self.pos = f16(p, -P_MAX, P_MAX)
                self.vel = f16(v, -self.V, self.V)
                self.tau = f16(t, -self.T, self.T)
                self.temp = tc / 10.0
                self.fault = (m.arbitration_id >> 16) & 0x3F
                return True
        return False

    def enable(self):
        return self._send(3, HOST)

    def stop(self, clear_fault=False):
        return self._send(4, HOST, bytes([1 if clear_fault else 0]) + bytes(7))

    def set_zero(self):
        """Current position becomes mechanical zero (type 6). Motor must be stopped."""
        return self._send(6, HOST, bytes([1]) + bytes(7))

    def mit(self, pos, vel=0.0, kp=0.0, kd=0.0, tau=0.0):
        data = struct.pack('>HHHH', u16(pos, -P_MAX, P_MAX), u16(vel, -self.V, self.V),
                           u16(kp, 0, self.KP), u16(kd, 0, self.KD))
        return self._send(1, u16(tau, -self.T, self.T), data)

    def read(self):
        """Poll feedback without changing anything: a zero-gain, zero-torque command."""
        return self.mit(self.pos or 0.0)


def open_bus(interface='gs_usb', channel=0, bitrate=1_000_000):
    return can.Bus(interface=interface, channel=channel, bitrate=bitrate)
