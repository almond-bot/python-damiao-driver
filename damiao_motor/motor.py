import can
from typing import Dict, Any

# -----------------------
# Motor parameter limits
# -----------------------
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.0, 45.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -18.0, 18.0

# -----------------------
# Motor state codes
# -----------------------
DM_MOTOR_DISABLED = 0x0
DM_MOTOR_ENABLED = 0x1
DM_MOTOR_OVER_VOLTAGE = 0x8
DM_MOTOR_UNDER_VOLTAGE = 0x9
DM_MOTOR_OVER_CURRENT = 0xA
DM_MOTOR_MOS_OVER_TEMP = 0xB
DM_MOTOR_ROTOR_OVER_TEMP = 0xC
DM_MOTOR_LOST_COMM = 0xD
DM_MOTOR_OVERLOAD = 0xE

_STATE_NAME_MAP = {
    DM_MOTOR_DISABLED: "DISABLED",
    DM_MOTOR_ENABLED: "ENABLED",
    DM_MOTOR_OVER_VOLTAGE: "OVER_VOLTAGE",
    DM_MOTOR_UNDER_VOLTAGE: "UNDER_VOLTAGE",
    DM_MOTOR_OVER_CURRENT: "OVER_CURRENT",
    DM_MOTOR_MOS_OVER_TEMP: "MOS_OVER_TEMP",
    DM_MOTOR_ROTOR_OVER_TEMP: "ROTOR_OVER_TEMP",
    DM_MOTOR_LOST_COMM: "LOST_COMM",
    DM_MOTOR_OVERLOAD: "OVERLOAD",
}


def decode_state_name(state_code: int) -> str:
    """Return human-readable name for a motor state code."""
    return _STATE_NAME_MAP.get(state_code, f"UNKNOWN({state_code})")


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    x_clipped = min(max(x, x_min), x_max)
    return int((x_clipped - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return float(x_int) * span / ((1 << bits) - 1) + x_min


class DaMiaoMotor:
    """
    Lightweight DaMiao motor wrapper for MIT-style control over a CAN bus.

    This is essentially the same encoding/decoding logic as in minimal_single_motor.py,
    but packaged as a reusable class.
    """

    def __init__(self, motor_id: int, feedback_id: int, bus: can.Bus) -> None:
        self.motor_id = motor_id
        self.feedback_id = feedback_id
        self.bus = bus

        # last decoded feedback
        self.state: Dict[str, Any] = {}

    # -----------------------
    # Encode messages
    # -----------------------
    def encode_cmd_msg(self, pos: float, vel: float, torq: float, kp: float, kd: float) -> bytes:
        pos_u = float_to_uint(pos, P_MIN, P_MAX, 16)
        vel_u = float_to_uint(vel, V_MIN, V_MAX, 12)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 12)
        torq_u = float_to_uint(torq, T_MIN, T_MAX, 12)

        data = [
            (pos_u >> 8) & 0xFF,
            pos_u & 0xFF,
            (vel_u >> 4) & 0xFF,
            ((vel_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0xF) << 4) | ((torq_u >> 8) & 0xF),
            torq_u & 0xFF,
        ]
        return bytes(data)

    @staticmethod
    def encode_enable_msg() -> bytes:
        return bytes([0xFF] * 7 + [0xFC])

    @staticmethod
    def encode_disable_msg() -> bytes:
        return bytes([0xFF] * 7 + [0xFD])

    # -----------------------
    # Sending CAN frames
    # -----------------------
    def send_raw(self, data: bytes) -> None:
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def enable(self) -> None:
        self.send_raw(self.encode_enable_msg())

    def disable(self) -> None:
        self.send_raw(self.encode_disable_msg())

    def send_cmd(self, pos: float, vel: float, torq: float, kp: float = 0.0, kd: float = 0.0) -> None:
        data = self.encode_cmd_msg(pos, vel, torq, kp, kd)
        self.send_raw(data)

    # -----------------------
    # Decode feedback
    # -----------------------
    def decode_sensor_feedback(self, data: bytes) -> Dict[str, float]:
        if len(data) != 8:
            raise ValueError("Feedback frame must have length 8")

        can_id = data[0] & 0x0F
        state = data[0] >> 4
        pos_int = (data[1] << 8) | data[2]
        vel_int = (data[3] << 4) | (data[4] >> 4)
        torq_int = ((data[4] & 0xF) << 8) | data[5]
        t_mos = float(data[6])
        t_rotor = float(data[7])

        decoded = {
            "can_id": can_id,
            "state": state,
            "state_name": decode_state_name(state),
            "pos": uint_to_float(pos_int, P_MIN, P_MAX, 16),
            "vel": uint_to_float(vel_int, V_MIN, V_MAX, 12),
            "torq": uint_to_float(torq_int, T_MIN, T_MAX, 12),
            "t_mos": t_mos,
            "t_rotor": t_rotor,
        }
        self.state = decoded
        return decoded


