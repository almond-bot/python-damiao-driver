import math
import time

from damiao_motor import DaMiaoController


def main() -> None:
    """
    WARNING: This example will move multiple motors.
    Make sure all motors are mounted and operated in safe conditions
    (no loose clothing, secure the mechanism, keep clear of moving parts).

    Multi-motor demo:
    - controller on can0
    - two motors with IDs 0x01 and 0x02 (edit to match your setup)
    - sine position command
    """
    confirm = input(
        "WARNING: This example will MOVE MULTIPLE motors.\n"
        "Ensure all are mounted and operated safely (no loose clothing, secure mechanism, clear of moving parts).\n"
        "Type 'yes' to continue: "
    ).strip().lower()
    if confirm != "yes":
        print("Aborting: user did not confirm safety.")
        return

    controller = DaMiaoController(channel="can0", bustype="socketcan")

    controller.add_motor(motor_id=0x01, feedback_id=0x00)
    controller.add_motor(motor_id=0x02, feedback_id=0x00)

    controller.enable_all()
    time.sleep(0.1)

    def step(ctrl: DaMiaoController, t: float) -> None:
        target_pos = 1.0 * math.sin(2.0 * math.pi * 0.2 * t)
        kp = 20.0
        kd = 0.5
        for motor in ctrl.all_motors():
            motor.send_cmd(pos=target_pos, vel=0.0, torq=0.0, kp=kp, kd=kd)

    controller.run_loop(freq_hz=100.0, step_fn=step, print_feedback=True)


if __name__ == "__main__":
    main()


