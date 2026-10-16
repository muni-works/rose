"""CAN-FD bus load for one limb bus at the S0 rate (brain/DESIGN.md, benchmark 3).

    python3 canfd_load.py [RATE_HZ] [ARB_KBPS] [DATA_MBPS]

One bus per limb (hub to limb controller). Per tick: one state frame up,
one command frame down, plus the hub's sync frame. Joint fields are int16:
state q, dq, tau (6 B/joint) + IMU (12 B) + status (4 B); command q*, kp, kd
(6 B/joint) + status (4 B). Frame bits follow ISO 11898-1 CAN-FD with an
11-bit id and bit stuffing (worst case: fixed stuff in the data phase at
1 per 4 bits, dynamic stuff in arbitration at 1 per 5 bits).
"""
import sys

DLC = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def fd_frame_us(payload, arb_bps, data_bps):
    n = next(d for d in DLC if d >= payload)
    crc = 17 if n <= 16 else 21
    arb_bits = 1 + 11 + 1 + 1 + 1 + 1          # SOF, id, RRS, IDE, FDF, res
    arb_bits += arb_bits // 5                  # stuffing in arbitration
    data_bits = 1 + 1 + 4 + 8 * n + 4 + crc    # BRS, ESI, DLC, data, stuff count, CRC
    data_bits += data_bits // 4                # fixed stuff bits
    tail_bits = 1 + 1 + 1 + 7 + 3              # CRC delim, ACK, ACK delim, EOF, IFS
    return (arb_bits + tail_bits) / arb_bps * 1e6 + data_bits / data_bps * 1e6, n


def limb(name, joints, rate, arb, data):
    up = joints * 6 + 12 + 4
    down = joints * 6 + 4
    frames = [fd_frame_us(up, arb, data), fd_frame_us(down, arb, data), fd_frame_us(8, arb, data)]
    busy = sum(f[0] for f in frames)
    load = busy * rate / 1e6
    print(f'{name:>6}: {joints} joints, up {up} B -> {frames[0][1]}, down {down} B -> {frames[1][1]}, '
          f'{busy:6.1f} µs per tick, load {100 * load:5.1f} %')
    return load


if __name__ == '__main__':
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 1000
    arb = float(sys.argv[2]) * 1e3 if len(sys.argv) > 2 else 1e6
    data = float(sys.argv[3]) * 1e6 if len(sys.argv) > 3 else 5e6
    print(f'{rate:.0f} Hz, arbitration {arb / 1e3:.0f} kbit/s, data {data / 1e6:.0f} Mbit/s')
    for name, j in (('arm', 4), ('leg', 6), ('head', 2)):
        limb(name, j, rate, arb, data)
