## damiao-motor

Python driver for DaMiao motors over CAN, with support for multiple motors on a single bus.

### Requirements

- Linux with a working CAN interface (e.g. SocketCAN on `can0`)
- Python 3.8+
- `python-can` (installed automatically when you install the package)

Make sure your CAN interface is configured (for example, with SocketCAN on Linux):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### Installation

From PyPI (once published):

```bash
pip install damiao-motor
```

For local / development installs from a clone of this repository:

```bash
pip install -e .
```

This installs the `damiao_motor` package in editable mode.

### Quick usage

**Safety note:** The examples below will move the motor. Make sure the motor is securely mounted, keep clear of moving parts, and follow your lab/robot safety guidelines.

Single/multi-motor examples are in the `examples/` directory. After installation you can run, for example:

```bash
python examples/multi_motor.py
```

Adjust motor IDs and gains in the example scripts to match your hardware.

A minimal single-motor example using the library API:

```python
import math
from damiao_motor import DaMiaoController

controller = DaMiaoController(channel="can0", bustype="socketcan")
motor = controller.add_motor(motor_id=0x01, feedback_id=0x00)

controller.enable_all()

def step(ctrl: DaMiaoController, t: float) -> None:
    target_pos = 1.0 * math.sin(2.0 * math.pi * 0.2 * t)
    motor.send_cmd(pos=target_pos, vel=0.0, torq=0.0, kp=20.0, kd=0.5)

controller.run_loop(freq_hz=100.0, step_fn=step, print_feedback=True)
```