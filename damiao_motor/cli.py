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
            if debug:
                # Print sent command in debug mode
                cmd_data = motor.encode_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0)
                data_hex = " ".join(f"{b:02X}" for b in cmd_data)
                print(f"  [SENT] 0x{motor.motor_id:03X} [{data_hex}]")
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
        # Debug mode: collect and print raw messages immediately
        if debug:
            # Read and collect raw messages, then process normally
            while True:
                msg = controller.bus.recv(timeout=0)
                if msg is None:
                    break
                data_hex = " ".join(f"{b:02X}" for b in msg.data)
                debug_msg = f"  0x{msg.arbitration_id:03X} [{data_hex}]"
                debug_messages.append(debug_msg)
                # Print immediately in debug mode
                print(debug_msg)
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

    # Print debug summary if messages were collected
    if debug and debug_messages:
        print()
        print("=" * 60)
        print(f"DEBUG: Total {len(debug_messages)} raw CAN messages received")
        print("=" * 60)

    # Cleanup
    try:
        controller.bus.shutdown()
    except:
        pass

    return responded_ids


def cmd_scan(args) -> None:
    """
    Handle 'scan' subcommand.
    
    Scans for connected motors on the CAN bus by sending zero commands and listening for feedback.
    
    Args:
        args: Parsed command-line arguments containing:
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - ids: Optional list of motor IDs to test (default: 0x01-0x10)
            - duration: Duration to listen for responses in seconds (default: 0.5)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
            - debug: Print all raw CAN messages for debugging (default: False)
    
    Examples:
        ```bash
        # Scan default ID range (0x01-0x10)
        damiao scan
        
        # Scan specific motor IDs
        damiao scan --ids 1 2 3
        
        # Scan with longer listen duration
        damiao scan --duration 2.0
        
        # Scan with debug output
        damiao scan --debug
        ```
    """
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


def cmd_set_zero(args) -> None:
    """
    Handle 'set-zero-command' subcommand.
    
    Sends a zero command (pos=0, vel=0, torq=0, kp=0, kd=0) to a motor continuously.
    Loops until interrupted with Ctrl+C.
    
    Args:
        args: Parsed command-line arguments containing:
            - motor_id: Motor ID to send zero command to (required)
            - frequency: Command frequency in Hz (default: 100.0)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Examples:
        ```bash
        # Send zero command continuously
        damiao set-zero-command --id 1
        
        # With custom frequency
        damiao set-zero-command --id 1 --frequency 50.0
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Set Zero Command")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Motor ID: 0x{args.motor_id:02X} ({args.motor_id})")
    print("=" * 60)
    print()

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        motor = controller.add_motor(motor_id=args.motor_id, feedback_id=0x00)
        
        print(f"Sending zero command continuously (press Ctrl+C to stop)...")
        print(f"  Command: pos=0, vel=0, torq=0, kp=0, kd=0")
        print(f"  Frequency: {args.frequency} Hz")
        print()
        
        interval = 1.0 / args.frequency if args.frequency > 0 else 0.01
        
        try:
            while True:
                motor.set_zero_command()
                controller.poll_feedback()
                
                if motor.state:
                    state = motor.state
                    status_code = state.get("status_code", "N/A")
                    status_name = state.get("status", "UNKNOWN")
                    print(f"State: {status_code} ({status_name}) | "
                          f"Pos:{state.get('pos', 0.0): 8.3f} rad | "
                          f"Vel:{state.get('vel', 0.0): 8.3f} rad/s | "
                          f"Torq:{state.get('torq', 0.0): 8.3f} Nm | "
                          f"T_mos:{state.get('t_mos', 0.0):5.1f}°C | "
                          f"T_rotor:{state.get('t_rotor', 0.0):5.1f}°C")
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n\nStopped by user.")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        print("Shutting down controller...")
        controller.shutdown()


def cmd_set_motor_id(args) -> None:
    """
    Handle 'set-motor-id' subcommand.
    
    Changes the motor's receive ID (ESC_ID, register 8). This is the ID used to send commands to the motor.
    
    Args:
        args: Parsed command-line arguments containing:
            - current: Current motor ID (to connect to the motor) (required)
            - target: Target motor ID (new receive ID) (required)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Note:
        After changing the motor ID, you will need to use the new ID to communicate with the motor.
        The value is stored to flash memory after setting.
    
    Examples:
        ```bash
        # Change motor ID from 1 to 2
        damiao set-motor-id --current 1 --target 2
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Set Motor ID (Receive ID)")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Current Motor ID: 0x{args.current:02X} ({args.current})")
    print(f"Target Motor ID: 0x{args.target:02X} ({args.target})")
    print("=" * 60)
    print()

    if args.current == args.target:
        print("Current and target IDs are the same. No change needed.")
        return

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        # Use current ID to connect
        motor = controller.add_motor(motor_id=args.current, feedback_id=0x00)
        
        print(f"Reading current register values...")
        time.sleep(0.1)
        controller.poll_feedback()
        
        # Read current receive ID (register 8)
        try:
            current_receive_id = motor.read_register(8, timeout=1.0)
            print(f"Current Receive ID (register 8): {int(current_receive_id)} (0x{int(current_receive_id):02X})")
        except Exception as e:
            print(f"⚠ Warning: Could not read register 8: {e}")
            print("  Proceeding with write anyway...")
        
        print(f"Writing new Receive ID (register 8) = {args.target} (0x{args.target:02X})...")
        motor.write_register(8, args.target)
        
        # Store parameters to flash
        print("Storing parameters to flash...")
        try:
            motor.store_parameters()
            print("✓ Parameters stored to flash")
        except Exception as e:
            print(f"⚠ Warning: Could not store parameters: {e}")
        
        print()
        print(f"✓ Motor ID changed from 0x{args.current:02X} to 0x{args.target:02X}")
        print(f"  Note: You may need to reconnect using the new ID: 0x{args.target:02X}")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        controller.shutdown()


def cmd_set_zero_position(args) -> None:
    """
    Handle 'set-zero-position' subcommand.
    
    Sets the current output shaft position to zero (save position zero).
    
    Args:
        args: Parsed command-line arguments containing:
            - motor_id: Motor ID (required)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Examples:
        ```bash
        # Set current position to zero
        damiao set-zero-position --id 1
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Set Zero Position")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Motor ID: 0x{args.motor_id:02X} ({args.motor_id})")
    print("=" * 60)
    print()

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        motor = controller.add_motor(motor_id=args.motor_id, feedback_id=0x00)
        
        print(f"Setting current position to zero...")
        motor.set_zero_position()
        print(f"✓ Position zero set")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        controller.shutdown()


def cmd_set_can_timeout(args) -> None:
    """
    Handle 'set-can-timeout' subcommand.
    
    Sets the CAN timeout alarm time (register 9) in milliseconds.
    
    Args:
        args: Parsed command-line arguments containing:
            - motor_id: Motor ID (required)
            - timeout_ms: Timeout in milliseconds (required)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Note:
        Register 9 stores timeout in units of 50 microseconds: **1 register unit = 50 microseconds**.
        
        The timeout is internally converted from milliseconds to register units using:
        register_value = timeout_ms × 20
        
        Examples:
        - 1000 ms = 20,000 register units
        - 50 ms = 1,000 register units
        
        The value is stored to flash memory after setting.
    
    Examples:
        ```bash
        # Set CAN timeout to 1000 ms
        damiao set-can-timeout --id 1 --timeout 1000
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Set CAN Timeout")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Motor ID: 0x{args.motor_id:02X} ({args.motor_id})")
    print(f"Timeout: {args.timeout_ms} ms")
    print("=" * 60)
    print()

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        motor = controller.add_motor(motor_id=args.motor_id, feedback_id=0x00)
        
        print(f"Setting CAN timeout to {args.timeout_ms} ms (register 9)...")
        motor.set_can_timeout(args.timeout_ms)
        
        # Store parameters to flash
        print("Storing parameters to flash...")
        try:
            motor.store_parameters()
            print("✓ Parameters stored to flash")
        except Exception as e:
            print(f"⚠ Warning: Could not store parameters: {e}")
        
        print()
        print(f"✓ CAN timeout set to {args.timeout_ms} ms")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        controller.shutdown()


def cmd_send_cmd(args) -> None:
    """
    Handle unified 'send-cmd' subcommand.
    
    Sends command to motor with specified control mode. Loops continuously until Ctrl+C.
    Supports MIT, position_velocity, velocity, and force_position_hybrid control modes.
    
    Args:
        args: Parsed command-line arguments containing:
            - motor_id: Motor ID (required)
            - mode: Control mode - "MIT", "position_velocity", "velocity", or "force_position_hybrid" (default: MIT)
            - position: Desired position (radians) - for MIT, position_velocity, force_position_hybrid modes
            - velocity: Desired velocity (rad/s) - for MIT, position_velocity, velocity modes
            - stiffness: Stiffness (kp) for MIT mode (default: 0.0)
            - damping: Damping (kd) for MIT mode (default: 0.0)
            - feedforward_torque: Feedforward torque for MIT mode (default: 0.0)
            - velocity_limit: Velocity limit (rad/s, 0-100) for force_position_hybrid mode
            - current_limit: Torque current limit normalized (0.0-1.0) for force_position_hybrid mode
            - frequency: Command frequency in Hz (default: 100.0)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Examples:
        ```bash
        # MIT mode (default)
        damiao send-cmd --id 1 --mode MIT --position 1.5 --velocity 0.0 --stiffness 20.0
        
        # Position-Velocity mode
        damiao send-cmd --id 1 --mode position_velocity --position 1.5 --velocity 2.0
        
        # Velocity mode
        damiao send-cmd --id 1 --mode velocity --velocity 3.0
        
        # Force-Position Hybrid mode
        damiao send-cmd --id 1 --mode force_position_hybrid --position 1.5 --velocity-limit 50.0 --current-limit 0.8
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Send Command")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Motor ID: 0x{args.motor_id:02X} ({args.motor_id})")
    print(f"Control Mode: {args.mode}")
    if args.mode == "MIT":
        print(f"  Position: {args.position:.6f} rad")
        print(f"  Velocity: {args.velocity:.6f} rad/s")
        print(f"  Stiffness (kp): {args.stiffness:.6f}")
        print(f"  Damping (kd): {args.damping:.6f}")
        print(f"  Feedforward Torque: {args.feedforward_torque:.6f} Nm")
    elif args.mode == "position_velocity":
        print(f"  Position: {args.position:.6f} rad")
        print(f"  Velocity: {args.velocity:.6f} rad/s")
    elif args.mode == "velocity":
        print(f"  Velocity: {args.velocity:.6f} rad/s")
    elif args.mode == "force_position_hybrid":
        print(f"  Position: {args.position:.6f} rad")
        print(f"  Velocity Limit: {args.velocity_limit:.6f} rad/s")
        print(f"  Current Limit: {args.current_limit:.6f}")
    print(f"Frequency: {args.frequency} Hz")
    print("=" * 60)
    print()

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        motor = controller.add_motor(motor_id=args.motor_id, feedback_id=0x00)
        
        # Determine CAN ID based on mode
        can_id_map = {
            "MIT": args.motor_id,
            "position_velocity": 0x100 + args.motor_id,
            "velocity": 0x200 + args.motor_id,
            "force_position_hybrid": 0x300 + args.motor_id,
        }
        can_id = can_id_map.get(args.mode, args.motor_id)
        
        print(f"Sending command continuously (press Ctrl+C to stop)...")
        print(f"  CAN ID: 0x{can_id:03X}")
        print(f"  Mode: {args.mode}")
        print(f"  Frequency: {args.frequency} Hz")
        print()
        
        interval = 1.0 / args.frequency if args.frequency > 0 else 0.01
        
        try:
            while True:
                if args.mode == "MIT":
                    motor.send_cmd(
                        target_position=args.position,
                        target_velocity=args.velocity,
                        stiffness=args.stiffness,
                        damping=args.damping,
                        feedforward_torque=args.feedforward_torque,
                        control_mode="MIT"
                    )
                elif args.mode == "position_velocity":
                    motor.send_cmd(
                        target_position=args.position,
                        target_velocity=args.velocity,
                        control_mode="position_velocity"
                    )
                elif args.mode == "velocity":
                    motor.send_cmd(
                        target_velocity=args.velocity,
                        control_mode="velocity"
                    )
                elif args.mode == "force_position_hybrid":
                    motor.send_cmd(
                        target_position=args.position,
                        velocity_limit=args.velocity_limit,
                        current_limit=args.current_limit,
                        control_mode="force_position_hybrid"
                    )
                
                controller.poll_feedback()
                
                if motor.state:
                    state = motor.state
                    status_code = state.get("status_code", "N/A")
                    status_name = state.get("status", "UNKNOWN")
                    print(f"State: {status_code} ({status_name}) | "
                          f"Pos:{state.get('pos', 0.0): 8.3f} rad | "
                          f"Vel:{state.get('vel', 0.0): 8.3f} rad/s | "
                          f"Torq:{state.get('torq', 0.0): 8.3f} Nm | "
                          f"T_mos:{state.get('t_mos', 0.0):5.1f}°C | "
                          f"T_rotor:{state.get('t_rotor', 0.0):5.1f}°C")
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n\nStopped by user.")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        print("Shutting down controller...")
        controller.shutdown()


def cmd_set_feedback_id(args) -> None:
    """
    Handle 'set-feedback-id' subcommand.
    
    Changes the motor's feedback ID (MST_ID, register 7). This is the ID used to identify feedback messages from the motor.
    
    Args:
        args: Parsed command-line arguments containing:
            - current: Current motor ID (to connect to the motor) (required)
            - target: Target feedback ID (new MST_ID) (required)
            - channel: CAN channel (default: can0)
            - bustype: CAN bus type (default: socketcan)
            - bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Note:
        The motor will now respond with feedback using the new feedback ID.
        The value is stored to flash memory after setting.
    
    Examples:
        ```bash
        # Change feedback ID to 3 (using motor ID 1 to connect)
        damiao set-feedback-id --current 1 --target 3
        ```
    """
    print("=" * 60)
    print("DaMiao Motor - Set Feedback ID (MST_ID)")
    print("=" * 60)
    print(f"CAN channel: {args.channel}")
    print(f"Current Motor ID: 0x{args.current:02X} ({args.current})")
    print(f"Target Feedback ID: 0x{args.target:02X} ({args.target})")
    print("=" * 60)
    print()

    # Check and bring up CAN interface if needed
    if args.bustype == "socketcan":
        if not check_and_bring_up_can_interface(args.channel, bitrate=args.bitrate):
            print(f"⚠ Warning: Could not verify {args.channel} is ready. Continuing anyway...")

    controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
    
    try:
        # Use current motor ID to connect
        motor = controller.add_motor(motor_id=args.current, feedback_id=0x00)
        
        print(f"Reading current register values...")
        time.sleep(0.1)
        controller.poll_feedback()
        
        # Read current feedback ID (register 7)
        try:
            current_feedback_id = motor.read_register(7, timeout=1.0)
            print(f"Current Feedback ID (register 7): {int(current_feedback_id)} (0x{int(current_feedback_id):02X})")
        except Exception as e:
            print(f"⚠ Warning: Could not read register 7: {e}")
            print("  Proceeding with write anyway...")
        
        print(f"Writing new Feedback ID (register 7) = {args.target} (0x{args.target:02X})...")
        motor.write_register(7, args.target)
        
        # Store parameters to flash
        print("Storing parameters to flash...")
        try:
            motor.store_parameters()
            print("✓ Parameters stored to flash")
        except Exception as e:
            print(f"⚠ Warning: Could not store parameters: {e}")
        
        print()
        print(f"✓ Feedback ID changed to 0x{args.target:02X}")
        print(f"  Note: Motor will now respond with feedback ID 0x{args.target:02X}")
        
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        controller.shutdown()


def unified_main() -> None:
    """
    Unified CLI entry point with subcommands.
    
    Main entry point for the `damiao` command-line tool. Provides a unified interface
    for scanning, configuring, and controlling DaMiao motors over CAN bus.
    
    Available commands:
        - scan: Scan for connected motors
        - send-cmd: Send command to motor (all control modes)
        - set-zero-command: Send zero command continuously
        - set-zero-position: Set current position to zero
        - set-can-timeout: Set CAN timeout alarm time
        - set-motor-id: Change motor receive ID
        - set-feedback-id: Change motor feedback ID
    
    Global options (available for all commands):
        - --channel: CAN channel (default: can0)
        - --bustype: CAN bus type (default: socketcan)
        - --bitrate: CAN bitrate in bits per second (default: 1000000)
    
    Examples:
        ```bash
        # Scan for motors
        damiao scan
        
        # Send command in MIT mode
        damiao send-cmd --id 1 --mode MIT --position 1.5 --velocity 0.0 --stiffness 20.0
        
        # Set current position to zero
        damiao set-zero-position --id 1
        ```
    """
    parser = argparse.ArgumentParser(
        description="DaMiao Motor CLI Tool - Control and configure DaMiao motors over CAN bus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Scan for motors on default CAN channel (can0)
  damiao scan

  # Scan specific motor IDs with debug output
  damiao scan --ids 1 2 3 --debug

  # Send command in MIT mode
  damiao send-cmd --id 1 --mode MIT --position 1.5 --velocity 0.0 --stiffness 20.0

  # Send command in velocity mode
  damiao send-cmd --id 1 --mode velocity --velocity 3.0

  # Set current position to zero
  damiao set-zero-position --id 1

  # Set CAN timeout
  damiao set-can-timeout --id 1 --timeout 1000

  # Use different CAN channel (can be before or after command)
  damiao scan --channel can_leader_l
  damiao send-cmd --id 1 --mode MIT --channel can_leader_l

For more information about a specific command, use:
  damiao <command> --help
        """,
    )
    
    # Global arguments
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
        "--bitrate",
        type=int,
        default=1000000,
        help="CAN bitrate in bits per second (default: 1000000). Only used when bringing up interface.",
    )
    
    subparsers = parser.add_subparsers(
        dest="command",
        help="Available commands",
        required=True,
        metavar="COMMAND",
        title="Commands",
        description="Use 'damiao <command> --help' for more information about a specific command."
    )
    
    # Helper function to add global arguments to subcommands
    def add_global_args(subparser):
        """Add global arguments to a subcommand parser."""
        subparser.add_argument(
            "--channel",
            type=str,
            default="can0",
            help="CAN channel (default: can0)",
        )
        subparser.add_argument(
            "--bustype",
            type=str,
            default="socketcan",
            help="CAN bus type (default: socketcan)",
        )
        subparser.add_argument(
            "--bitrate",
            type=int,
            default=1000000,
            help="CAN bitrate in bits per second (default: 1000000). Only used when bringing up interface.",
        )
    
    # scan command
    scan_parser = subparsers.add_parser(
        "scan",
        help="Scan for connected motors on CAN bus",
        description="Scan for connected DaMiao motors by sending zero commands and listening for responses.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Scan default ID range (0x01-0x10)
  damiao scan

  # Scan specific motor IDs
  damiao scan --ids 1 2 3

  # Scan with longer listen duration
  damiao scan --duration 2.0

  # Scan with debug output (print all raw CAN messages)
  damiao scan --debug
        """
    )
    scan_parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        metavar="ID",
        help="Motor IDs to test (e.g., --ids 1 2 3). If not specified, tests IDs 0x01-0x10.",
    )
    scan_parser.add_argument(
        "--duration",
        type=float,
        default=0.5,
        help="Duration to listen for responses in seconds (default: 0.5)",
    )
    scan_parser.add_argument(
        "--debug",
        action="store_true",
        help="Print all raw CAN messages for debugging.",
    )
    add_global_args(scan_parser)
    scan_parser.set_defaults(func=cmd_scan)
    
    # set-zero-command (renamed from set-zero)
    zero_parser = subparsers.add_parser(
        "set-zero-command",
        help="Send zero command to a motor (pos=0, vel=0, torq=0, kp=0, kd=0)",
        description="Send a zero command continuously to a motor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Send zero command continuously (loops until Ctrl+C)
  damiao set-zero-command --id 1

  # With custom frequency
  damiao set-zero-command --id 1 --frequency 50.0
        """
    )
    zero_parser.add_argument(
        "--id",
        type=int,
        required=True,
        dest="motor_id",
        help="Motor ID to send zero command to",
    )
    zero_parser.add_argument(
        "--frequency",
        type=float,
        default=100.0,
        help="Command frequency in Hz (default: 100.0)",
    )
    add_global_args(zero_parser)
    zero_parser.set_defaults(func=cmd_set_zero)
    
    # set-zero-position command
    zero_pos_parser = subparsers.add_parser(
        "set-zero-position",
        help="Set current position to zero",
        description="Set the current output shaft position to zero (save position zero).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Set current position to zero
  damiao set-zero-position --id 1
        """
    )
    zero_pos_parser.add_argument(
        "--id",
        type=int,
        required=True,
        dest="motor_id",
        help="Motor ID",
    )
    add_global_args(zero_pos_parser)
    zero_pos_parser.set_defaults(func=cmd_set_zero_position)
    
    # set-can-timeout command
    timeout_parser = subparsers.add_parser(
        "set-can-timeout",
        help="Set CAN timeout alarm time (register 9)",
        description="Set the CAN timeout alarm time in milliseconds. Register 9 uses units of 50 microseconds (1 unit = 50us).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Set CAN timeout to 1000 ms
  damiao set-can-timeout --id 1 --timeout 1000
        """
    )
    timeout_parser.add_argument(
        "--id",
        type=int,
        required=True,
        dest="motor_id",
        help="Motor ID",
    )
    timeout_parser.add_argument(
        "--timeout",
        type=int,
        required=True,
        dest="timeout_ms",
        help="Timeout in milliseconds (ms)",
    )
    add_global_args(timeout_parser)
    timeout_parser.set_defaults(func=cmd_set_can_timeout)
    
    # set-motor-id command
    set_motor_id_parser = subparsers.add_parser(
        "set-motor-id",
        help="Set motor receive ID (register 8)",
        description="Change the motor's receive ID (ESC_ID, register 8). This is the ID used to send commands to the motor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Change motor ID from 1 to 2
  damiao set-motor-id --current 1 --target 2

Note: After changing the motor ID, you will need to use the new ID to communicate with the motor.
        """
    )
    set_motor_id_parser.add_argument(
        "--current",
        type=int,
        required=True,
        help="Current motor ID (to connect to the motor)",
    )
    set_motor_id_parser.add_argument(
        "--target",
        type=int,
        required=True,
        help="Target motor ID (new receive ID)",
    )
    add_global_args(set_motor_id_parser)
    set_motor_id_parser.set_defaults(func=cmd_set_motor_id)
    
    # set-feedback-id command
    set_feedback_id_parser = subparsers.add_parser(
        "set-feedback-id",
        help="Set motor feedback ID (register 7)",
        description="Change the motor's feedback ID (MST_ID, register 7). This is the ID used to identify feedback messages from the motor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Change feedback ID to 3 (using motor ID 1 to connect)
  damiao set-feedback-id --current 1 --target 3

Note: The motor will now respond with feedback using the new feedback ID.
        """
    )
    set_feedback_id_parser.add_argument(
        "--current",
        type=int,
        required=True,
        help="Current motor ID (to connect to the motor)",
    )
    set_feedback_id_parser.add_argument(
        "--target",
        type=int,
        required=True,
        help="Target feedback ID (new MST_ID)",
    )
    add_global_args(set_feedback_id_parser)
    set_feedback_id_parser.set_defaults(func=cmd_set_feedback_id)
    
    # send-cmd command (unified command for all modes)
    send_cmd_parser = subparsers.add_parser(
        "send-cmd",
        help="Send command to motor (unified command for all control modes)",
        description="Send command to motor with specified control mode. Loops continuously until Ctrl+C.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # MIT mode (default)
  damiao send-cmd --id 1 --mode MIT --position 1.5 --velocity 0.0 --stiffness 20.0 --damping 0.5

  # Position-Velocity mode
  damiao send-cmd --id 1 --mode position_velocity --position 1.5 --velocity 2.0

  # Velocity mode
  damiao send-cmd --id 1 --mode velocity --velocity 3.0

  # Force-Position Hybrid mode
  damiao send-cmd --id 1 --mode force_position_hybrid --position 1.5 --velocity-limit 50.0 --current-limit 0.8

  # With custom frequency
  damiao send-cmd --id 1 --mode MIT --position 1.5 --frequency 50.0
        """
    )
    send_cmd_parser.add_argument(
        "--id",
        type=int,
        required=True,
        dest="motor_id",
        help="Motor ID",
    )
    send_cmd_parser.add_argument(
        "--mode",
        type=str,
        default="MIT",
        choices=["MIT", "position_velocity", "velocity", "force_position_hybrid"],
        help="Control mode (default: MIT)",
    )
    send_cmd_parser.add_argument(
        "--position",
        type=float,
        default=0.0,
        help="Desired position (radians). Required for MIT, position_velocity, force_position_hybrid modes.",
    )
    send_cmd_parser.add_argument(
        "--velocity",
        type=float,
        default=0.0,
        help="Desired velocity (rad/s). Required for MIT, position_velocity, velocity modes.",
    )
    send_cmd_parser.add_argument(
        "--stiffness",
        type=float,
        default=0.0,
        dest="stiffness",
        help="Stiffness (kp) for MIT mode (default: 0.0)",
    )
    send_cmd_parser.add_argument(
        "--damping",
        type=float,
        default=0.0,
        dest="damping",
        help="Damping (kd) for MIT mode (default: 0.0)",
    )
    send_cmd_parser.add_argument(
        "--feedforward-torque",
        type=float,
        default=0.0,
        dest="feedforward_torque",
        help="Feedforward torque for MIT mode (default: 0.0)",
    )
    send_cmd_parser.add_argument(
        "--velocity-limit",
        type=float,
        default=0.0,
        dest="velocity_limit",
        help="Velocity limit (rad/s, 0-100) for force_position_hybrid mode",
    )
    send_cmd_parser.add_argument(
        "--current-limit",
        type=float,
        default=0.0,
        dest="current_limit",
        help="Torque current limit normalized (0.0-1.0) for force_position_hybrid mode",
    )
    send_cmd_parser.add_argument(
        "--frequency",
        type=float,
        default=100.0,
        help="Command frequency in Hz (default: 100.0)",
    )
    add_global_args(send_cmd_parser)
    send_cmd_parser.set_defaults(func=cmd_send_cmd)
    
    args = parser.parse_args()
    
    # Execute the appropriate command
    try:
        args.func(args)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError: {e}")
        sys.exit(1)


if __name__ == "__main__":
    unified_main()

