import can
import struct
import time
from typing import Dict, Any, Optional, Literal
from dataclasses import dataclass

# -----------------------
# Register table
# -----------------------

@dataclass
class RegisterInfo:
    """Information about a motor register."""
    rid: int
    variable: str
    description: str
    access: Literal["RW", "RO"]
    range_str: str
    data_type: Literal["float", "uint32"]

# Register table based on manufacturer documentation
REGISTER_TABLE: Dict[int, RegisterInfo] = {
    # Protection and basic parameters (0-6)
    0: RegisterInfo(0, "UV_Value", "Under-voltage protection value", "RW", "(10.0, 3.4E38]", "float"),
    1: RegisterInfo(1, "KT_Value", "Torque coefficient", "RW", "[0.0, 3.4E38]", "float"),
    2: RegisterInfo(2, "OT_Value", "Over-temperature protection value", "RW", "[80.0, 200)", "float"),
    3: RegisterInfo(3, "OC_Value", "Over-current protection value", "RW", "(0.0, 1.0)", "float"),
    4: RegisterInfo(4, "ACC", "Acceleration", "RW", "(0.0, 3.4E38)", "float"),
    5: RegisterInfo(5, "DEC", "Deceleration", "RW", "[-3.4E38, 0.0)", "float"),
    6: RegisterInfo(6, "MAX_SPD", "Maximum speed", "RW", "(0.0, 3.4E38]", "float"),
    
    # System identification and configuration (7-10)
    7: RegisterInfo(7, "MST_ID", "Feedback ID", "RW", "[0, 0x7FF]", "uint32"),
    8: RegisterInfo(8, "ESC_ID", "Receive ID", "RW", "[0, 0x7FF]", "uint32"),
    9: RegisterInfo(9, "TIMEOUT", "Timeout alarm time", "RW", "[0, 2^32-1]", "uint32"),
    10: RegisterInfo(10, "CTRL_MODE", "Control mode", "RW", "[1, 4]", "uint32"),
    
    # Motor physical parameters (11-20) - Read Only
    11: RegisterInfo(11, "Damp", "Motor viscous damping coefficient", "RO", "/", "float"),
    12: RegisterInfo(12, "Inertia", "Motor moment of inertia", "RO", "/", "float"),
    13: RegisterInfo(13, "hw_ver", "Reserved", "RO", "/", "uint32"),
    14: RegisterInfo(14, "sw_ver", "Software version number", "RO", "/", "uint32"),
    15: RegisterInfo(15, "SN", "Reserved", "RO", "/", "uint32"),
    16: RegisterInfo(16, "NPP", "Motor pole pairs", "RO", "/", "uint32"),
    17: RegisterInfo(17, "Rs", "Motor phase resistance", "RO", "/", "float"),
    18: RegisterInfo(18, "Ls", "Motor phase inductance", "RO", "/", "float"),
    19: RegisterInfo(19, "Flux", "Motor flux linkage value", "RO", "/", "float"),
    20: RegisterInfo(20, "Gr", "Gear reduction ratio", "RO", "/", "float"),
    
    # Mapping ranges (21-23)
    21: RegisterInfo(21, "PMAX", "Position mapping range", "RW", "(0.0, 3.4E38]", "float"),
    22: RegisterInfo(22, "VMAX", "Speed mapping range", "RW", "(0.0, 3.4E38]", "float"),
    23: RegisterInfo(23, "TMAX", "Torque mapping range", "RW", "(0.0, 3.4E38]", "float"),
    
    # Control loop parameters (24-28)
    24: RegisterInfo(24, "I_BW", "Current loop control bandwidth", "RW", "[100.0, 10000.0]", "float"),
    25: RegisterInfo(25, "KP_ASR", "Speed loop Kp", "RW", "[0.0, 3.4E38]", "float"),
    26: RegisterInfo(26, "KI_ASR", "Speed loop Ki", "RW", "[0.0, 3.4E38]", "float"),
    27: RegisterInfo(27, "KP_APR", "Position loop Kp", "RW", "[0.0, 3.4E38]", "float"),
    28: RegisterInfo(28, "KI_APR", "Position loop Ki", "RW", "[0.0, 3.4E38]", "float"),
    
    # Protection and efficiency (29-32)
    29: RegisterInfo(29, "OV_Value", "Overvoltage protection value", "RW", "TBD", "float"),
    30: RegisterInfo(30, "GREF", "Gear torque efficiency", "RW", "(0.0, 1.0]", "float"),
    31: RegisterInfo(31, "Deta", "Speed loop damping coefficient", "RW", "[1.0, 30.0]", "float"),
    32: RegisterInfo(32, "V_BW", "Speed loop filter bandwidth", "RW", "(0.0, 500.0)", "float"),
    
    # Enhancement coefficients (33-34)
    33: RegisterInfo(33, "IQ_c1", "Current loop enhancement coefficient", "RW", "[100.0, 10000.0]", "float"),
    34: RegisterInfo(34, "VL_c1", "Speed loop enhancement coefficient", "RW", "(0.0, 10000.0]", "float"),
    
    # CAN and version (35-36)
    35: RegisterInfo(35, "can_br", "CAN baud rate code", "RW", "[0, 4]", "uint32"),
    36: RegisterInfo(36, "sub_ver", "Sub-version number", "RO", "/", "uint32"),
    
    # Calibration parameters (50-55) - Read Only
    50: RegisterInfo(50, "u_off", "U-phase offset", "RO", "", "float"),
    51: RegisterInfo(51, "v_off", "V-phase offset", "RO", "", "float"),
    52: RegisterInfo(52, "k1", "Compensation factor 1", "RO", "", "float"),
    53: RegisterInfo(53, "k2", "Compensation factor 2", "RO", "", "float"),
    54: RegisterInfo(54, "m_off", "Angle offset", "RO", "", "float"),
    55: RegisterInfo(55, "dir", "Direction", "RO", "", "float"),
    
    # Position feedback (80-81) - Read Only
    80: RegisterInfo(80, "p_m", "Motor position", "RO", "", "float"),
    81: RegisterInfo(81, "xout", "Output shaft position", "RO", "", "float"),
}

# CAN baud rate codes
CAN_BAUD_RATE_CODES = {
    0: 125000,   # 125K
    1: 200000,   # 200K
    2: 250000,   # 250K
    3: 500000,   # 500K
    4: 1000000,  # 1M
}

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
    Lightweight DaMiao motor wrapper over a CAN bus.
    """

    def __init__(self, motor_id: int, feedback_id: int, bus: can.Bus) -> None:
        self.motor_id = motor_id
        self.feedback_id = feedback_id
        self.bus = bus

        # last decoded feedback
        self.state: Dict[str, Any] = {}

    def get_states(self) -> Dict[str, Any]:
        """
        Get the current motor state dictionary.
        
        Returns:
            Dictionary containing current motor state information:
            - can_id: CAN ID
            - status: Status code
            - state_name: Human-readable state name
            - pos: Position
            - vel: Velocity
            - torq: Torque
            - t_mos: MOSFET temperature
            - t_rotor: Rotor temperature
            - arbitration_id: CAN arbitration ID
        """
        return self.state.copy() if self.state else {}

    # -----------------------
    # Encode messages
    # -----------------------
    def encode_cmd_msg(self, pos: float, vel: float, torq: float, kp: float, kd: float) -> bytes:
        """
        Encode a command to CAN frame for sending to the motor.
        Check 
        """
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

    def send_cmd(
        self,
        target_position: float = 0.0,
        target_velocity: float = 0.0,
        stiffness: float = 0.0,
        damping: float = 0.0,
        feedforward_torque: float = 0.0,
    ) -> None:
        data = self.encode_cmd_msg(target_position, target_velocity, feedforward_torque, stiffness, damping)
        self.send_raw(data)

    # -----------------------
    # Decode feedback
    # -----------------------
    def decode_sensor_feedback(self, data: bytes, arbitration_id: int | None = None) -> Dict[str, float]:
        if len(data) != 8:
            raise ValueError("Feedback frame must have length 8")

        can_id = data[0] & 0x0F
        status = data[0] >> 4
        pos_int = (data[1] << 8) | data[2]
        vel_int = (data[3] << 4) | (data[4] >> 4)
        torq_int = ((data[4] & 0xF) << 8) | data[5]
        t_mos = float(data[6])
        t_rotor = float(data[7])

        decoded = {
            "can_id": can_id,
            "arbitration_id": arbitration_id,
            "status": decode_state_name(status),
            "pos": uint_to_float(pos_int, P_MIN, P_MAX, 16),
            "vel": uint_to_float(vel_int, V_MIN, V_MAX, 12),
            "torq": uint_to_float(torq_int, T_MIN, T_MAX, 12),
            "t_mos": t_mos,
            "t_rotor": t_rotor,
        }
        self.state = decoded
        return decoded

    # -----------------------
    # Register read/write operations
    # -----------------------
    def _encode_can_id(self, can_id: int) -> tuple[int, int]:
        """Encode CAN ID into low and high bytes."""
        return can_id & 0xFF, (can_id >> 8) & 0xFF

    def _send_register_cmd(self, cmd_byte: int, rid: int, data: Optional[bytes] = None) -> None:
        """
        Send a register command (read/write/store).
        
        Args:
            cmd_byte: Command byte (0x33 for read, 0x55 for write, 0xAA for store)
            rid: Register ID (0-81)
            data: Optional 4-byte data for write operations
        """
        canid_l, canid_h = self._encode_can_id(self.motor_id)
        
        if data is None:
            # Read or store command - D[4-7] are don't care
            msg_data = bytes([canid_l, canid_h, cmd_byte, rid, 0x00, 0x00, 0x00, 0x00])
        else:
            # Write command - D[4-7] contain the data
            if len(data) != 4:
                raise ValueError("Data must be 4 bytes for write operations")
            msg_data = bytes([canid_l, canid_h, cmd_byte, rid]) + data
        
        msg = can.Message(arbitration_id=0x7FF, data=msg_data, is_extended_id=False)
        self.bus.send(msg)

    def read_register(self, rid: int, timeout: float = 1.0) -> float | int:
        """
        Read a register value from the motor.
        
        Args:
            rid: Register ID (0-81)
            timeout: Timeout in seconds to wait for response
        
        Returns:
            Register value as float or int depending on register data type
        
        Raises:
            KeyError: If register ID is not in the register table
            ValueError: If register is write-only (should not happen for RO registers)
        """
        # Check if register exists in table
        if rid not in REGISTER_TABLE:
            raise KeyError(f"Register {rid} not found in register table")
        
        reg_info = REGISTER_TABLE[rid]
        
        # Check if register is readable
        if reg_info.access == "RO" or reg_info.access == "RW":
            pass  # Both RO and RW are readable
        else:
            raise ValueError(f"Register {rid} ({reg_info.variable}) is not readable")
        
        # Send read command
        self._send_register_cmd(0x33, rid)
        
        # Wait for response
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            
            # Check if this is a response to our read command
            if len(msg.data) == 8:
                if msg.data[0] == (self.motor_id & 0xFF) and msg.data[1] == ((self.motor_id >> 8) & 0xFF):
                    if msg.data[2] == 0x33 and msg.data[3] == rid:
                        # Extract data from D[4-7] (D4 is low byte, D7 is high byte)
                        data_bytes = bytes([msg.data[4], msg.data[5], msg.data[6], msg.data[7]])
                        
                        # Use data type from register table
                        if reg_info.data_type == "float":
                            return struct.unpack("<f", data_bytes)[0]  # Little-endian float
                        elif reg_info.data_type == "uint32":
                            return struct.unpack("<I", data_bytes)[0]  # Little-endian uint32
                        else:
                            raise ValueError(f"Unknown data_type: {reg_info.data_type} for register {rid}")
        
        raise TimeoutError(f"Timeout waiting for register {rid} ({reg_info.variable}) read response")

    def write_register(self, rid: int, value: float | int) -> None:
        """
        Write a value to a register.
        
        Args:
            rid: Register ID (0-81)
            value: Value to write (float or int)
        
        Raises:
            KeyError: If register ID is not in the register table
            ValueError: If register is read-only or value is out of range
        """
        # Check if register exists in table
        if rid not in REGISTER_TABLE:
            raise KeyError(f"Register {rid} not found in register table")
        
        reg_info = REGISTER_TABLE[rid]
        
        # Check if register is writable
        if reg_info.access != "RW":
            raise ValueError(f"Register {rid} ({reg_info.variable}) is read-only (access: {reg_info.access})")
        
        # Encode value to 4 bytes using data type from register table
        if reg_info.data_type == "float":
            data_bytes = struct.pack("<f", float(value))
        elif reg_info.data_type == "uint32":
            data_bytes = struct.pack("<I", int(value))
        else:
            raise ValueError(f"Unknown data_type: {reg_info.data_type} for register {rid}")
        
        # Send write command
        self._send_register_cmd(0x55, rid, data_bytes)
        
        # Wait for echo response (optional - motor echoes back the written data)
        # Note: You may want to verify the echo matches, but for now we just send

    def get_register_info(self, rid: int) -> RegisterInfo:
        """
        Get information about a register.
        
        Args:
            rid: Register ID
        
        Returns:
            RegisterInfo object with register details
        
        Raises:
            KeyError: If register ID is not in the register table
        """
        if rid not in REGISTER_TABLE:
            raise KeyError(f"Register {rid} not found in register table")
        return REGISTER_TABLE[rid]

    def store_parameters(self) -> None:
        """
        Store all parameters to flash memory.
        After successful write, all parameters will be written to the chip.
        """
        self._send_register_cmd(0xAA, 0x01)

    def request_motor_feedback(self) -> None:
        """
        Request motor feedback/status information.
        After successful transmission, the motor driver will return current status information.
        """
        canid_l, canid_h = self._encode_can_id(self.motor_id)
        msg_data = bytes([canid_l, canid_h, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00])
        msg = can.Message(arbitration_id=0x7FF, data=msg_data, is_extended_id=False)
        self.bus.send(msg)

    def read_all_registers(self, timeout: float = 0.05) -> Dict[int, float | int]:
        """
        Read all registers from the motor.
        
        Args:
            timeout: Timeout in seconds per register read
        
        Returns:
            Dictionary mapping register ID to value
        """
        results: Dict[int, float | int] = {}
        for rid, reg_info in REGISTER_TABLE.items():
            if reg_info.access in ["RO", "RW"]:  # Readable registers
                try:
                    results[rid] = self.read_register(rid, timeout=timeout)
                except (TimeoutError, KeyError, ValueError) as e:
                    # Store error as string for debugging
                    results[rid] = f"ERROR: {e}"
        return results

    # -----------------------
    # Setter methods for all writable registers
    # -----------------------
    def set_under_voltage_protection(self, value: float) -> None:
        """Set under-voltage protection value (register 0)."""
        self.write_register(0, value)

    def set_torque_coefficient(self, value: float) -> None:
        """Set torque coefficient (register 1)."""
        self.write_register(1, value)

    def set_over_temperature_protection(self, value: float) -> None:
        """Set over-temperature protection value (register 2)."""
        self.write_register(2, value)

    def set_over_current_protection(self, value: float) -> None:
        """Set over-current protection value (register 3)."""
        self.write_register(3, value)

    def set_acceleration(self, value: float) -> None:
        """Set acceleration (register 4)."""
        self.write_register(4, value)

    def set_deceleration(self, value: float) -> None:
        """Set deceleration (register 5)."""
        self.write_register(5, value)

    def set_maximum_speed(self, value: float) -> None:
        """Set maximum speed (register 6)."""
        self.write_register(6, value)

    def set_feedback_id(self, value: int) -> None:
        """Set feedback ID (register 7)."""
        self.write_register(7, value)

    def set_receive_id(self, value: int) -> None:
        """Set receive ID (register 8)."""
        self.write_register(8, value)

    def set_timeout_alarm(self, value: int) -> None:
        """Set timeout alarm time (register 9)."""
        self.write_register(9, value)

    def set_control_mode(self, value: int) -> None:
        """Set control mode (register 10)."""
        self.write_register(10, value)

    def set_position_mapping_range(self, value: float) -> None:
        """Set position mapping range (register 21)."""
        self.write_register(21, value)

    def set_speed_mapping_range(self, value: float) -> None:
        """Set speed mapping range (register 22)."""
        self.write_register(22, value)

    def set_torque_mapping_range(self, value: float) -> None:
        """Set torque mapping range (register 23)."""
        self.write_register(23, value)

    def set_current_loop_bandwidth(self, value: float) -> None:
        """Set current loop control bandwidth (register 24)."""
        self.write_register(24, value)

    def set_speed_loop_kp(self, value: float) -> None:
        """Set speed loop proportional gain Kp (register 25)."""
        self.write_register(25, value)

    def set_speed_loop_ki(self, value: float) -> None:
        """Set speed loop integral gain Ki (register 26)."""
        self.write_register(26, value)

    def set_position_loop_kp(self, value: float) -> None:
        """Set position loop proportional gain Kp (register 27)."""
        self.write_register(27, value)

    def set_position_loop_ki(self, value: float) -> None:
        """Set position loop integral gain Ki (register 28)."""
        self.write_register(28, value)

    def set_overvoltage_protection(self, value: float) -> None:
        """Set overvoltage protection value (register 29)."""
        self.write_register(29, value)

    def set_gear_efficiency(self, value: float) -> None:
        """Set gear torque efficiency (register 30)."""
        self.write_register(30, value)

    def set_speed_loop_damping(self, value: float) -> None:
        """Set speed loop damping coefficient (register 31)."""
        self.write_register(31, value)

    def set_speed_loop_filter_bandwidth(self, value: float) -> None:
        """Set speed loop filter bandwidth (register 32)."""
        self.write_register(32, value)

    def set_current_loop_enhancement(self, value: float) -> None:
        """Set current loop enhancement coefficient (register 33)."""
        self.write_register(33, value)

    def set_speed_loop_enhancement(self, value: float) -> None:
        """Set speed loop enhancement coefficient (register 34)."""
        self.write_register(34, value)

    def set_can_baud_rate(self, baud_rate_code: int) -> None:
        """
        Set CAN baud rate using register 35 (can_br).
        
        Args:
            baud_rate_code: Baud rate code (0=125K, 1=200K, 2=250K, 3=500K, 4=1M)
        
        Raises:
            ValueError: If baud_rate_code is not in valid range [0, 4]
        """
        if baud_rate_code not in CAN_BAUD_RATE_CODES:
            raise ValueError(f"Invalid baud rate code: {baud_rate_code}. Must be in {list(CAN_BAUD_RATE_CODES.keys())}")
        
        self.write_register(35, baud_rate_code)  # Register 35 is can_br
        self.store_parameters()  # Store to flash so it persists


