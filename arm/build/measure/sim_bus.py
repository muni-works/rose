"""A simulated RobStride CAN bus for dry runs: the scripts' real frames in, type-2 feedback out.

`LYMB_SIM=1` makes robstride.open_bus() return SimBus instead of hardware. Each simulated motor
integrates its own joint: MIT torque kp*(p*-p) + kd*(v*-v) + tau_ff against the model's gravity torque
(from arm.gravity at the current pose and the SIM_LOAD_KG payload), with a lumped inertia, viscous
friction, and a first-order case temperature driven by tau^2. Encoder noise is SIM_NOISE_DEG (1-sigma).
`hand(q_model_deg)` stands in for Cam's hands during teach: it moves the joints to a pose, plus
SIM_ERR_DEG of zero error per joint (fixed per run), so report.py has something to find.
"""
import math
import os
import random
import struct
import threading
import time

import can

import robstride as R

J_EFF = {'RS04': 0.08, 'RS02': 0.02, 'RS00': 0.01}     # kg m^2 at the output, lumped with the link
B_VISC = {'RS04': 2.0, 'RS02': 0.4, 'RS00': 0.2}
KT_HEAT = {'RS04': 0.022, 'RS02': 0.6, 'RS00': 0.8}    # steady-state °C per (N.m)^2: 25 N.m -> ~+14 °C on an RS04
TAU_TH = 120.0                                          # s, case time constant (sped up for a dry run)
AMBIENT = 25.0


class SimMotor:
    def __init__(self, cid, model, joint, err):
        self.id, self.model, self.joint, self.err = cid, model, joint, err
        self.p = self.v = self.tau = 0.0
        self.temp = AMBIENT
        self.enabled = False
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)


class SimBus:
    def __init__(self, jm, load_kg=None):
        import arm
        self.arm, self.jm = arm, jm
        self.load = float(os.environ.get('SIM_LOAD_KG', 0.0)) if load_kg is None else load_kg
        rng = random.Random(int(os.environ.get('SIM_SEED', 1)))
        e = float(os.environ.get('SIM_ERR_DEG', 0.05))
        self.noise = math.radians(float(os.environ.get('SIM_NOISE_DEG', 0.01)))
        self.m = {v['id']: SimMotor(v['id'], v['model'], j, math.radians(rng.gauss(0, e)))
                  for j, v in jm.items()}
        self.out = []
        self.lock = threading.Lock()
        self.t = time.time()
        self.held = None      # the ball in its seat pins the arm, as the cone does on the real plate

    # --- physics
    def _step(self):
        now = time.time()
        dt = min(now - self.t, 0.02)
        self.t = now
        q = {m.joint: self.arm.to_model(self.jm, m.joint, m.p) for m in self.m.values()}
        g = self.arm.gravity(q, self.load)
        for m in self.m.values():
            T, V, KP, KD = R.RANGE[m.model]
            if self.held is not None:
                m.v = 0.0
                m.p = self.held[m.id]
            if m.enabled:
                p_, v_, kp, kd, tff = m.cmd
                tau = kp * (p_ - m.p) + kd * (v_ - m.v) + tff
                tau = max(-T, min(T, tau))
            else:
                tau = 0.0
            m.tau = tau
            grav = -g[m.joint] / self.jm[m.joint]['sign']        # what gravity does to the motor shaft
            acc = (tau + grav - B_VISC[m.model] * m.v) / J_EFF[m.model]
            if self.held is None:
                m.v += acc * dt
                m.p += m.v * dt
            ss = AMBIENT + KT_HEAT[m.model] * tau * tau
            m.temp += (ss - m.temp) * dt / TAU_TH * 10

    def hand(self, q_model_deg):
        """Cam puts the probe in the seat: joints jump to the pose (true angles), motors at rest."""
        with self.lock:
            for m in self.m.values():
                qm = math.radians(q_model_deg[m.joint])
                m.p = self.arm.to_motor(self.jm, m.joint, qm) - m.err
                m.v = 0.0
            self.held = {m.id: m.p for m in self.m.values()}

    def release(self):
        with self.lock:
            self.held = None

    # --- python-can surface
    def send(self, msg):
        with self.lock:
            self._step()
            aid = msg.arbitration_id
            typ, mid16, cid = aid >> 24, (aid >> 8) & 0xFFFF, aid & 0xFF
            m = self.m.get(cid)
            if m is None:
                return
            T, V, KP, KD = R.RANGE[m.model]
            if typ == 1:
                p, v, kp, kd = struct.unpack('>HHHH', bytes(msg.data))
                m.cmd = (R.f16(p, -R.P_MAX, R.P_MAX), R.f16(v, -V, V), R.f16(kp, 0, KP), R.f16(kd, 0, KD), R.f16(mid16, -T, T))
            elif typ == 3:
                m.enabled = True
            elif typ == 4:
                m.enabled = False
            elif typ == 6:
                m.err += m.p          # the zero moves with the reading
                m.p = 0.0
            pos = m.p + random.gauss(0, self.noise)
            data = struct.pack('>HHHH', R.u16(pos, -R.P_MAX, R.P_MAX), R.u16(m.v, -V, V),
                               R.u16(m.tau, -T, T), int(m.temp * 10))
            fid = (2 << 24) | (cid << 8) | R.HOST
            self.out.append(can.Message(arbitration_id=fid, data=data, is_extended_id=True))

    def recv(self, timeout=None):
        with self.lock:
            return self.out.pop(0) if self.out else None

    def shutdown(self):
        pass
