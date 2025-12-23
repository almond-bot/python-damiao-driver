# Web GUI

The `damiao-gui` command provides a web-based interface for viewing and editing DaMiao motor parameters.

## Installation

After installing the package, the `damiao-gui` command will be available in your PATH:

```bash
pip install damiao-motor
```

## Starting the GUI

To start the web GUI server:

```bash
damiao-gui
```

By default, the server runs on `http://127.0.0.1:5000`. Open this URL in your web browser.

## Command Options

```bash
damiao-gui [OPTIONS]
```

**Options:**

| Option | Type | Description |
|--------|------|-------------|
| `--host` | `STR` | Host to bind to (default: 127.0.0.1) |
| `--port` | `INT` | Port to bind to (default: 5000) |
| `--debug` | flag | Enable debug mode |
| `--production` | flag | Use production WSGI server (requires waitress) |

**Examples:**
```bash
# Start GUI on default host and port
damiao-gui

# Start GUI on a specific port
damiao-gui --port 8080

# Start GUI on all interfaces
damiao-gui --host 0.0.0.0

# Use production server
damiao-gui --production
```

## Features

- **Scan for Motors**: Automatically discover connected motors on the CAN bus
- **View Registers**: Read and display all motor registers
- **Edit Registers**: Modify writable register values
- **Real-time Updates**: View current motor state and parameters

## Using the GUI

1. **Connect to CAN Bus**: Click "Connect" and select your CAN channel (e.g., `can0`)
2. **Scan for Motors**: Click "Scan" to discover connected motors
3. **Select a Motor**: Choose a motor from the list to view its registers
4. **View/Edit Registers**: Browse the register table and modify values as needed
5. **Store Parameters**: Changes to critical registers (like motor ID) are automatically stored to flash

## Register Editing

When editing registers in the GUI:

- **Read-Only Registers**: Displayed in gray and cannot be modified
- **Writable Registers**: Can be edited directly in the table
- **Automatic Storage**: Critical registers (ID 7, 8) are automatically stored to flash after modification

## Safety Notes

!!! warning "Safety First"
    - Always verify register values before writing
    - Some register changes take effect immediately
    - Changes to motor ID (register 8) will require reconnection with the new ID
    - Test register changes in a safe environment before production use

