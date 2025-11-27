import time
from typing import Dict, Iterable, Optional, Callable

import can

from .motor import DaMiaoMotor


class DaMiaoController:
    """
    Simple multi-motor controller.

    - Owns a single CAN bus.
    - Manages multiple DaMiaoMotor instances on that bus.
    - Provides helper methods to:
        * enable/disable all motors
        * send commands to one or all motors
        * poll feedback non-blockingly
        * run a basic control loop callback at fixed frequency
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan") -> None:
        self.bus: can.Bus = can.interface.Bus(channel=channel, bustype=bustype)
        # Keyed by command CAN ID (motor_id)
        self.motors: Dict[int, DaMiaoMotor] = {}
        # Keyed by logical motor ID (embedded in feedback frame)
        self._motors_by_feedback: Dict[int, DaMiaoMotor] = {}

    # -----------------------
    # Motor management
    # -----------------------
    def add_motor(self, motor_id: int, feedback_id: int) -> DaMiaoMotor:
        if motor_id in self.motors:
            raise ValueError(f"Motor with ID {motor_id} already exists")

        motor = DaMiaoMotor(motor_id=motor_id, feedback_id=feedback_id, bus=self.bus)
        self.motors[motor_id] = motor
        # Bind by logical motor ID; feedback frames embed this ID in the first byte.
        self._motors_by_feedback[motor_id] = motor
        return motor

    def get_motor(self, motor_id: int) -> DaMiaoMotor:
        return self.motors[motor_id]

    def all_motors(self) -> Iterable[DaMiaoMotor]:
        return self.motors.values()

    # -----------------------
    # Enable / disable
    # -----------------------
    def enable_all(self) -> None:
        for m in self.all_motors():
            m.enable()

    def disable_all(self) -> None:
        for m in self.all_motors():
            m.disable()

    # -----------------------
    # Command helpers
    # -----------------------
    def send_cmd(
        self,
        motor_id: int,
        pos: float,
        vel: float,
        torq: float,
        kp: float = 0.0,
        kd: float = 0.0,
    ) -> None:
        self.get_motor(motor_id).send_cmd(pos=pos, vel=vel, torq=torq, kp=kp, kd=kd)

    def send_cmd_all(
        self,
        pos: float,
        vel: float,
        torq: float,
        kp: float = 0.0,
        kd: float = 0.0,
    ) -> None:
        for m in self.all_motors():
            m.send_cmd(pos=pos, vel=vel, torq=torq, kp=kp, kd=kd)

    # -----------------------
    # Feedback polling
    # -----------------------
    def poll_feedback(self) -> None:
        """
        Non-blocking read of all pending CAN frames on this bus, and dispatch
        feedback frames to the corresponding motors.
        """
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None:
                break

            if len(msg.data) != 8:
                continue

            # Feedback messages include the logical motor ID in the low 4 bits
            # of the first data byte. Use that to route feedback to the right motor.
            logical_id = msg.data[0] & 0x0F
            motor = self._motors_by_feedback.get(logical_id)
            if motor is None:
                continue

            motor.decode_sensor_feedback(bytes(msg.data))

    # -----------------------
    # Simple control loop
    # -----------------------
    def run_loop(
        self,
        freq_hz: float,
        step_fn: Callable[["DaMiaoController", float], None],
        print_feedback: bool = False,
    ) -> None:
        """
        Run a simple timing-based loop.

        step_fn(controller, t) is called at freq_hz for user logic.
        This allows you to compute and send commands inside step_fn.
        Feedback is polled continuously in between steps.
        """
        period = 1.0 / freq_hz
        next_send_time = time.perf_counter()

        try:
            while True:
                now = time.perf_counter()

                # Non-blocking feedback
                self.poll_feedback()

                # Call user step at fixed frequency
                if now >= next_send_time:
                    t = now
                    step_fn(self, t)
                    next_send_time += period
                    if now > next_send_time:
                        next_send_time = now + period

                # Optionally print latest feedback (just once per loop iteration)
                if print_feedback:
                    for m in self.all_motors():
                        if m.state:
                            state_code = m.state.get("state")
                            state_name = m.state.get("state_name")
                            pos = m.state.get("pos")
                            vel = m.state.get("vel")
                            torq = m.state.get("torq")
                            t_mos = m.state.get("t_mos")
                            t_rotor = m.state.get("t_rotor")
                            print(
                                f"ID 0x{m.motor_id:02X}: "
                                f"state={state_name} ({state_code}), "
                                f"pos={pos:.3f}, vel={vel:.3f}, torq={torq:.3f}, "
                                f"t_mos={t_mos:.1f}C, t_rotor={t_rotor:.1f}C"
                            )

                time.sleep(0.0001)
        except KeyboardInterrupt:
            self.disable_all()
            self.bus.shutdown()


