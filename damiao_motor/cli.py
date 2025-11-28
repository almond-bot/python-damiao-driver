#!/usr/bin/env python3
"""
CLI tool to scan for connected DaMiao motors and test communication.
"""
import argparse
import subprocess
import sys
import time
from typing import Any, Dict, Set

import can

from .controller import DaMiaoController
from .motor import DaMiaoMotor, REGISTER_TABLE

# ANSI color codes
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"


def check_and_bring_up_can_interface(channel: str, bitrate: int = 1000000) -> bool:
    """
    Check if CAN interface is up, and bring it up if it's down.
    
    Args:
        channel: CAN channel name (e.g., "can0")
        bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Returns:
        True if interface is up (or successfully brought up), False otherwise
    """
    try:
        # Check if interface exists and is up
        result = subprocess.run(
            ["ip", "link", "show", channel],
            capture_output=True,
            text=True,
            check=False,
        )
        
        if result.returncode != 0:
            print(f"Interface {channel} does not exist. Creating it...")
            # Create and bring up the interface
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "up", "type", "can", "bitrate", str(bitrate)],
                check=True,
            )
            print(f"✓ Interface {channel} created and brought up (bitrate: {bitrate})")
            time.sleep(0.5)  # Give it a moment to initialize
            return True
        
        # Check if interface is UP
        if "state UP" in result.stdout or "state UNKNOWN" in result.stdout:
            # Interface exists and is up, but verify it's actually a CAN interface
            if "link/can" not in result.stdout:
                print(f"⚠ Warning: {channel} is up but not configured as CAN interface. Reconfiguring...")
                # Reconfigure it
                subprocess.run(
                    ["sudo", "ip", "link", "set", channel, "down"],
                    check=False,
                )
                subprocess.run(
                    ["sudo", "ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
                    check=True,
                )
                subprocess.run(
                    ["sudo", "ip", "link", "set", channel, "up"],
                    check=True,
                )
                print(f"✓ Interface {channel} reconfigured as CAN (bitrate: {bitrate})")
                time.sleep(0.5)
            return True
        elif "state DOWN" in result.stdout:
            print(f"Interface {channel} is down. Configuring and bringing it up...")
            # Set it down first (in case it needs reconfiguration)
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "down"],
                check=False,  # Don't fail if already down
            )
            # Configure and bring up the interface with specified bitrate
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
                check=True,
            )
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "up"],
                check=True,
            )
            print(f"✓ Interface {channel} configured and brought up (bitrate: {bitrate})")
            time.sleep(0.5)  # Give it a moment to initialize
            # Verify it's actually up
            verify = subprocess.run(
                ["ip", "link", "show", channel],
                capture_output=True,
                text=True,
                check=False,
            )
            if verify.returncode == 0 and "state UP" in verify.stdout:
                return True
            else:
                print(f"⚠ Warning: {channel} configuration may have failed")
                return False
        else:
            # Try to bring it up anyway with full configuration
            print(f"Interface {channel} state unclear. Attempting to configure and bring it up...")
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "down"],
                check=False,
            )
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
                check=False,
            )
            subprocess.run(
                ["sudo", "ip", "link", "set", channel, "up"],
                check=False,
            )
            time.sleep(0.5)
            return True
            
    except subprocess.CalledProcessError as e:
        print(f"⚠ Warning: Could not bring up {channel} automatically: {e}")
        print(f"  You may need to run manually: sudo ip link set {channel} up type can bitrate {bitrate}")
        return False
    except FileNotFoundError:
        print("⚠ Warning: 'ip' command not found. Cannot automatically bring up CAN interface.")
        print(f"  Please run manually: sudo ip link set {channel} up type can bitrate {bitrate}")
        return False
    except Exception as e:
        print(f"⚠ Warning: Unexpected error checking CAN interface: {e}")
        return False


def scan_motors(
    channel: str = "can0",
    bustype: str = "socketcan",
    motor_ids: list[int] | None = None,
    duration_s: float = 3.0,
    bitrate: int = 1000000,
    debug: bool = False,
) -> Set[int]:
    """
    Scan for connected motors by sending zero commands and listening for feedback.

    Args:
        channel: CAN channel (e.g., "can0")
        bustype: CAN bus type (e.g., "socketcan")
        motor_ids: List of motor IDs to test. If None, tests IDs 0x01-0x10.
        duration_s: How long to listen for responses (seconds)

    Returns:
        Set of motor IDs that responded with feedback.
    """
    if motor_ids is None:
        motor_ids = list(range(0x01, 0x11))  # Test IDs 1-16

    # Check and bring up CAN interface if needed (only for socketcan)
    if bustype == "socketcan":
        print(f"Checking CAN interface {channel}...")
        if not check_and_bring_up_can_interface(channel, bitrate=bitrate):
            print(f"⚠ Warning: Could not verify {channel} is ready. Continuing anyway...")
        else:
            # Verify interface is actually up and working
            verify_result = subprocess.run(
                ["ip", "link", "show", channel],
                capture_output=True,
                text=True,
                check=False,
            )
            if verify_result.returncode == 0 and "state UP" in verify_result.stdout:
                print(f"✓ CAN interface {channel} is ready")
            else:
                print(f"⚠ Warning: {channel} may not be properly configured")

    controller = DaMiaoController(channel=channel, bustype=bustype)
    
    # Flush any pending messages from the bus
    print("Flushing CAN bus buffer...")
    flushed_count = controller.flush_bus()
    if flushed_count > 0:
        print(f"  Flushed {flushed_count} pending message(s) from bus")
    else:
        print("  Bus buffer is clean")
    
    motors: dict[int, DaMiaoMotor] = {}

    # Create motor instances for all IDs we want to test
    for motor_id in motor_ids:
        try:
            motor = controller.add_motor(motor_id=motor_id, feedback_id=0x00)
            motors[motor_id] = motor
        except ValueError:
            # Motor already exists, skip
            pass

    # Send zero command to all motors
    print(f"Sending zero command (pos=0, vel=0, torq=0, kp=0, kd=0) to {len(motors)} potential motor IDs...")
    try:
        for motor in motors.values():
            motor.send_cmd(target_position=0.0, target_velocity=0.0, stiffness=0.0, damping=0.0, feedforward_torque=0.0)
    except Exception as e:
        error_str = str(e)
        if "Error Code 105" in error_str or "No buffer space available" in error_str or "[Errno 105]" in error_str:
            print(f"\n⚠ [ERROR CODE 105] No buffer space available when sending commands")
            print(f"  Original error: {e}")
            print(f"\n  This error typically indicates:")
            print(f"    - No CAN device (motor) is connected to the bus")
            print(f"    - Motor(s) are not powered on")
            print(f"    - CAN interface hardware issue")
            print(f"  Please check:")
            print(f"    1. Motor(s) are properly connected to the CAN bus")
            print(f"    2. Motor(s) are powered on")
            print(f"    3. CAN interface hardware is working correctly")
            print(f"    4. CAN bus termination resistors (120Ω) are installed at both ends of the bus")
            # Clean up and exit gracefully
            try:
                controller.bus.shutdown()
            except:
                pass
            sys.exit(1)
        else:
            raise

    # Listen for feedback
    print(f"Listening for responses for {duration_s} seconds...")
    start_time = time.perf_counter()
    responded_ids: Set[int] = set()
    debug_messages = []  # Collect debug messages if debug mode is enabled
    # Track seen motor IDs and arbitration IDs for conflict detection
    seen_motor_ids: Set[int] = set()  # Track decoded motor IDs (logical_id)
    seen_arbitration_ids: Set[int] = set()  # Track arbitration IDs
    # Collect conflicts to group them at the end
    conflicted_motor_ids: Set[int] = set()  # Motor IDs that appeared multiple times
    conflicted_arbitration_ids: Set[int] = set()  # Arbitration IDs that appeared multiple times
    # Collect motor register information for table display
    motor_registers: Dict[int, Dict[int, float | int]] = {}  # motor_id -> {rid -> value}

    while time.perf_counter() - start_time < duration_s:
        # Debug mode: collect raw messages
        if debug:
            # Read and collect raw messages, then process normally
            while True:
                msg = controller.bus.recv(timeout=0)
                if msg is None:
                    break
                data_hex = " ".join(f"{b:02X}" for b in msg.data)
                debug_messages.append(f"  0x{msg.arbitration_id:03X} [{data_hex}]")
                # Process the message manually for debug mode
                if len(msg.data) == 8:
                    logical_id = msg.data[0] & 0x0F
                    arb_id = msg.arbitration_id
                    
                    # Check for motor ID conflict (same decoded motor ID seen twice)
                    if logical_id in seen_motor_ids:
                        conflicted_motor_ids.add(logical_id)
                    
                    # Check for arbitration ID conflict (same arbitration ID seen twice)
                    if arb_id in seen_arbitration_ids:
                        conflicted_arbitration_ids.add(arb_id)
                    
                    seen_motor_ids.add(logical_id)
                    seen_arbitration_ids.add(arb_id)
                    
                    motor = controller._motors_by_feedback.get(logical_id)
                    if motor is not None:
                        motor.decode_sensor_feedback(bytes(msg.data), arbitration_id=arb_id)
        else:
            # Normal mode: read messages, check conflicts, then process
            while True:
                msg = controller.bus.recv(timeout=0)
                if msg is None:
                    break
                
                if len(msg.data) == 8:
                    logical_id = msg.data[0] & 0x0F
                    arb_id = msg.arbitration_id
                    
                    # Check for motor ID conflict (same decoded motor ID seen twice)
                    if logical_id in seen_motor_ids:
                        conflicted_motor_ids.add(logical_id)
                    
                    # Check for arbitration ID conflict (same arbitration ID seen twice)
                    if arb_id in seen_arbitration_ids:
                        conflicted_arbitration_ids.add(arb_id)
                    
                    seen_motor_ids.add(logical_id)
                    seen_arbitration_ids.add(arb_id)
                    
                    # Process through controller
                    motor = controller._motors_by_feedback.get(logical_id)
                    if motor is not None:
                        motor.decode_sensor_feedback(bytes(msg.data), arbitration_id=arb_id)

        # Check which motors have received feedback
        for motor_id, motor in motors.items():
            if motor.state and motor.state.get("can_id") is not None:
                # Print once per motor when first detected
                if motor_id not in responded_ids:
                    state_name = motor.state.get("state_name", "UNKNOWN")
                    pos = motor.state.get("pos", 0.0)
                    arb_id = motor.state.get("arbitration_id")
                    if arb_id is not None:
                        print(f"  ✓ Motor ID 0x{motor_id:02X} responded (arbitration_id: 0x{arb_id:03X}, state: {state_name}, pos: {pos:.3f})")
                    else:
                        print(f"  ✓ Motor ID 0x{motor_id:02X} responded (state: {state_name}, pos: {pos:.3f})")
                
                responded_ids.add(motor_id)

        time.sleep(0.01)

    # Print conflicts (grouped)
    if conflicted_motor_ids:
        print()
        print(f"{RED}{'=' * 60}{RESET}")
        print(f"{RED}[ERROR] Motor ID Conflicts Detected{RESET}")
        print(f"  Multiple motors responded with the same motor ID.")
        print(f"  This indicates multiple motors are configured with the same motor ID.")
        print(f"  Conflicted Motor IDs: {', '.join(f'0x{mid:02X}' for mid in sorted(conflicted_motor_ids))}")
        print(f"{RED}{'=' * 60}{RESET}")
    
    if conflicted_arbitration_ids:
        print()
        print(f"{YELLOW}{'=' * 60}{RESET}")
        print(f"{YELLOW}[WARNING] Arbitration ID Conflicts Detected{RESET}")
        print(f"  Same arbitration ID seen multiple times.")
        print(f"  This may indicate a CAN bus configuration issue.")
        print(f"  Conflicted Arbitration IDs: {', '.join(f'0x{aid:03X}' for aid in sorted(conflicted_arbitration_ids))}")
        print(f"{YELLOW}{'=' * 60}{RESET}")

    # Read all registers from detected motors if no motor ID conflicts
    if not conflicted_motor_ids and responded_ids:
        print()
        print("Reading register parameters from detected motors...")
        for motor_id in sorted(responded_ids):
            motor = motors.get(motor_id)
            if motor is not None:
                try:
                    registers = motor.read_all_registers(timeout=0.05)
                    motor_registers[motor_id] = registers
                except Exception as e:
                    print(f"  ⚠ Failed to read registers from motor 0x{motor_id:02X}: {e}")

    # Print motor register table if no motor ID conflicts
    if not conflicted_motor_ids and motor_registers:
        print()
        print("=" * 120)
        print(f"{GREEN}Detected Motors - Register Parameters{RESET}")
        print("=" * 120)
        
        # Group registers by motor
        for motor_id in sorted(motor_registers.keys()):
            registers = motor_registers[motor_id]
            print()
            print(f"{GREEN}Motor ID: 0x{motor_id:02X} ({motor_id}){RESET}")
            print("-" * 120)
            print(f"{'RID':<6} {'Variable':<20} {'Description':<35} {'Value':<20} {'Type':<10} {'Access':<8}")
            print("-" * 120)
            
            for rid in sorted(registers.keys()):
                if rid not in REGISTER_TABLE:
                    continue
                
                reg_info = REGISTER_TABLE[rid]
                value = registers[rid]
                
                # Format value based on type
                if isinstance(value, str) and value.startswith("ERROR"):
                    value_str = value
                elif reg_info.data_type == "float":
                    value_str = f"{float(value):.6f}"
                else:
                    value_str = str(int(value))
                
                # Truncate long descriptions
                desc = reg_info.description[:33] + ".." if len(reg_info.description) > 35 else reg_info.description
                
                print(
                    f"{rid:<6} "
                    f"{reg_info.variable:<20} "
                    f"{desc:<35} "
                    f"{value_str:<20} "
                    f"{reg_info.data_type:<10} "
                    f"{reg_info.access:<8}"
                )
        
        print("=" * 120)

    # Print debug messages in a separate section
    if debug and debug_messages:
        print()
        print("=" * 60)
        print("DEBUG: Raw CAN Messages")
        for msg in debug_messages:
            print(msg)

    # Cleanup
    try:
        controller.bus.shutdown()
    except:
        pass

    return responded_ids


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scan for connected DaMiao motors on CAN bus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Scan default IDs (0x01-0x10) on can0
  damiao-scan

  # Scan specific motor IDs
  damiao-scan --ids 1 2 3

  # Use different CAN channel
  damiao-scan --channel can1

  # Listen longer for responses
  damiao-scan --duration 5.0
        """,
    )
    parser.add_argument(
        "--channel",
        type=str,
        default="can0",
        help="CAN channel (default: can0)",
    )
    parser.add_argument(
        "--bustype",
        type=str,
        default="socketcan",
        help="CAN bus type (default: socketcan)",
    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        metavar="ID",
        help="Motor IDs to test (e.g., --ids 1 2 3). If not specified, tests IDs 0x01-0x10.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.5,
        help="Duration to listen for responses in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1000000,
        help="CAN bitrate in bits per second (default: 1000000). Only used when bringing up interface.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print all raw CAN messages for debugging.",
    )

    args = parser.parse_args()

    print("=" * 60)
    print("DaMiao Motor Scanner")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Bus type: {args.bustype}")
    if args.ids:
        print(f"Testing motor IDs: {[hex(i) for i in args.ids]}")
    else:
        print("Testing motor IDs: 0x01-0x10 (default range)")
    print(f"Listen duration: {args.duration}s")
    if args.debug:
        print("Debug mode: ENABLED (printing all raw CAN messages)")
    print("=" * 60)
    print()

    try:
        responded = scan_motors(
            channel=args.channel,
            bustype=args.bustype,
            motor_ids=args.ids,
            duration_s=args.duration,
            bitrate=args.bitrate,
            debug=args.debug,
        )

        print()
        print("=" * 60)
        if responded:
            print(f"Found {len(responded)} motor(s):")
            for motor_id in sorted(responded):
                print(f"  • Motor ID: 0x{motor_id:02X} ({motor_id})")
        else:
            print("No motors responded.")
            print("Check:")
            print("  - CAN interface is up (e.g., sudo ip link set can0 up type can bitrate 1000000)")
            print("  - Motors are powered and connected")
            print("  - Motor IDs match the tested range")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        raise


if __name__ == "__main__":
    main()

