import time
import math

from damiao_motor import DaMiaoController


def main() -> None:
    """
    WARNING: This example will move the motor.
    Make sure the motor is mounted and operated in a safe condition
    (no loose clothing, secure the mechanism, keep clear of moving parts).

    Single-motor demo using the package API.

    - One motor with ID 0x01 (edit to match your setup)
    - Sends a position command with simple gains
    """
    confirm = input(
        "WARNING: This example will MOVE the motor.\n"
        "Ensure it is mounted and operated safely (no loose clothing, secure mechanism, clear of moving parts).\n"
        "Type 'yes' to continue: "
    ).strip().lower()
    if confirm != "yes":
        print("Aborting: user did not confirm safety.")
        return

    controller = DaMiaoController(channel="can0", bustype="socketcan")
    motor = controller.add_motor(motor_id=0x01, feedback_id=0x00)

    controller.enable_all()
    time.sleep(0.1)

    target_pos = 0.0
    kp = 20.0
    kd = 0.5

    def step(ctrl: DaMiaoController, t: float) -> None:
        target_pos = 1.0 * math.sin(2.0 * math.pi * 0.2 * t)
        motor.send_cmd(pos=target_pos, vel=0.0, torq=0.0, kp=kp, kd=kd)

    controller.run_loop(freq_hz=100.0, step_fn=step, print_feedback=True)


if __name__ == "__main__":
    main()


