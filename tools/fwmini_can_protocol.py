"""FW-mini four-wheel steering CAN motion-frame contract (no ROS / CAN writes).

The vendor DBC defines signed 16-bit fields at start bits 4, 20 and 36.
vx/vy = 0.001 m/s per bit; yaw rate = 0.01 degree/s per bit.
"""

from dataclasses import dataclass
import math

CTRL_COMMAND_ID = 0x18C4D1D0
CTRL_FEEDBACK_ID = 0x18C4D1EF
CAN_EFF_MASK = 0x1FFFFFFF
DEG_TO_RAD = math.pi / 180.0


@dataclass(frozen=True)
class MotionFeedback:
    gear: int
    vx_mps: float
    vy_mps: float
    wz_radps: float
    wz_degps: float
    counter: int


def _decode_signed16(value):
    return value - 65536 if value & 0x8000 else value


def decode_motion_frame(can_id, payload):
    """Decode and checksum-validate one chassis feedback frame, else ValueError."""
    if (can_id & CAN_EFF_MASK) != CTRL_FEEDBACK_ID:
        raise ValueError("unexpected CAN ID")
    if len(payload) != 8:
        raise ValueError("expected 8 CAN data bytes")
    if payload[7] != (payload[0] ^ payload[1] ^ payload[2] ^
                      payload[3] ^ payload[4] ^ payload[5] ^ payload[6]):
        raise ValueError("invalid XOR checksum")
    x = (payload[0] >> 4) | (payload[1] << 4) | ((payload[2] & 0x0f) << 12)
    z = (payload[2] >> 4) | (payload[3] << 4) | ((payload[4] & 0x0f) << 12)
    y = (payload[4] >> 4) | (payload[5] << 4) | ((payload[6] & 0x0f) << 12)
    wz_degps = _decode_signed16(z) * 0.01
    return MotionFeedback(
        gear=payload[0] & 0x0f,
        vx_mps=_decode_signed16(x) * 0.001,
        vy_mps=_decode_signed16(y) * 0.001,
        wz_degps=wz_degps,
        wz_radps=wz_degps * DEG_TO_RAD,
        counter=payload[6] >> 4,
    )


def encode_motion_command(gear, vx_mps, vy_mps, wz_radps, counter):
    """Offline CAN encoding reference; does not transmit or assert drive mode.

    The gear enum depends on the actual FW-mini VCU firmware. This helper
    only validates its 4-bit wire representation and never selects a gear.
    """
    if not isinstance(gear, int) or not 0 <= gear <= 15:
        raise ValueError("gear must be a 4-bit integer")
    if not isinstance(counter, int) or not 0 <= counter <= 15:
        raise ValueError("counter must be a 4-bit integer")

    def signed_scaled(value, scale):
        if not math.isfinite(value):
            raise ValueError("non-finite velocity")
        raw = math.trunc(value * scale)
        if not -32768 <= raw <= 32767:
            raise ValueError("CAN int16 encoding overflow")
        return raw & 0xffff

    x = signed_scaled(vx_mps, 1000)
    y = signed_scaled(vy_mps, 1000)
    z = signed_scaled(wz_radps / DEG_TO_RAD, 100)
    data = bytearray(8)
    data[0] = gear | ((x & 0x0f) << 4)
    data[1] = (x >> 4) & 0xff
    data[2] = ((x >> 12) & 0x0f) | ((z & 0x0f) << 4)
    data[3] = (z >> 4) & 0xff
    data[4] = ((z >> 12) & 0x0f) | ((y & 0x0f) << 4)
    data[5] = (y >> 4) & 0xff
    data[6] = ((y >> 12) & 0x0f) | (counter << 4)
    data[7] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]
    return bytes(data)


def integrate_body_twist(x, y, yaw, vx, vy, wz, dt):
    """Midpoint SE(2) integration; positive y is robot-left.

    Reject out-of-order/stale time rather than integrating enormous jumps.
    """
    if not all(math.isfinite(v) for v in (x, y, yaw, vx, vy, wz, dt)):
        raise ValueError("non-finite odometry")
    if dt < 0 or dt > 0.5:
        raise ValueError("invalid odom integration dt")
    mid_yaw = yaw + 0.5 * wz * dt
    return (x + (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt,
            y + (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt,
            yaw + wz * dt)
