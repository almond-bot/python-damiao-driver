#!/usr/bin/env python3
"""
Web-based GUI for viewing and changing DaMiao motor parameters.
"""
import json
from typing import Dict, Any, Optional

from flask import Flask, render_template_string, jsonify, request

from .controller import DaMiaoController
from .motor import REGISTER_TABLE, RegisterInfo

app = Flask(__name__)

# Global controller instance (will be initialized when needed)
_controller: Optional[DaMiaoController] = None
_motors: Dict[int, Any] = {}


def init_controller(channel: str = "can0", bustype: str = "socketcan") -> None:
    """Initialize the CAN controller."""
    global _controller, _motors
    # Shutdown existing controller if any
    if _controller is not None:
        try:
            _controller.shutdown()
        except:
            pass
    _controller = DaMiaoController(channel=channel, bustype=bustype)
    _motors = {}


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DaMiao Motor Parameter Editor</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            background: #f5f5f5;
            padding: 20px;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            padding: 30px;
        }
        h1 {
            color: #333;
            margin-bottom: 10px;
        }
        .subtitle {
            color: #666;
            margin-bottom: 30px;
        }
        .section {
            margin-bottom: 30px;
        }
        .section-title {
            font-size: 18px;
            font-weight: 600;
            color: #333;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 2px solid #e0e0e0;
        }
        .motor-selector {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 500;
            transition: all 0.2s;
        }
        .btn-primary {
            background: #007bff;
            color: white;
        }
        .btn-primary:hover {
            background: #0056b3;
        }
        .btn-success {
            background: #28a745;
            color: white;
        }
        .btn-success:hover {
            background: #218838;
        }
        .btn-danger {
            background: #dc3545;
            color: white;
        }
        .btn-danger:hover {
            background: #c82333;
        }
        .btn-secondary {
            background: #6c757d;
            color: white;
        }
        .btn-secondary:hover {
            background: #5a6268;
        }
        .btn:disabled {
            opacity: 0.6;
            cursor: not-allowed;
        }
        .status {
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 15px;
            display: none;
        }
        .status.success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .status.error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .status.info {
            background: #d1ecf1;
            color: #0c5460;
            border: 1px solid #bee5eb;
        }
        .register-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .register-table th {
            background: #f8f9fa;
            padding: 12px;
            text-align: left;
            font-weight: 600;
            color: #495057;
            border-bottom: 2px solid #dee2e6;
            position: sticky;
            top: 0;
        }
        .register-table td {
            padding: 10px 12px;
            border-bottom: 1px solid #e9ecef;
        }
        .register-table tr:hover {
            background: #f8f9fa;
        }
        .register-table tr.read-only {
            background: #f8f9fa;
            opacity: 0.8;
        }
        .value-cell {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .value-display {
            flex: 1;
        }
        .value-edit {
            display: none;
            flex: 1;
        }
        .value-edit input {
            width: 100%;
            padding: 6px 10px;
            border: 1px solid #ced4da;
            border-radius: 4px;
            font-size: 14px;
        }
        .value-edit input:focus {
            outline: none;
            border-color: #007bff;
            box-shadow: 0 0 0 2px rgba(0,123,255,0.25);
        }
        .edit-btn, .save-btn, .cancel-btn {
            padding: 4px 12px;
            font-size: 12px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        .edit-btn {
            background: #ffc107;
            color: #000;
        }
        .edit-btn:hover {
            background: #e0a800;
        }
        .save-btn {
            background: #28a745;
            color: white;
        }
        .save-btn:hover {
            background: #218838;
        }
        .cancel-btn {
            background: #6c757d;
            color: white;
        }
        .cancel-btn:hover {
            background: #5a6268;
        }
        .loading {
            text-align: center;
            padding: 40px;
            color: #666;
        }
        .spinner {
            border: 3px solid #f3f3f3;
            border-top: 3px solid #007bff;
            border-radius: 50%;
            width: 30px;
            height: 30px;
            animation: spin 1s linear infinite;
            margin: 0 auto 10px;
        }
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        .config-section {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .config-section label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
            color: #495057;
        }
        .config-section input {
            width: 100%;
            padding: 8px;
            border: 1px solid #ced4da;
            border-radius: 4px;
            margin-bottom: 10px;
        }
        .value-edit select {
            width: 100%;
            padding: 6px 10px;
            border: 1px solid #ced4da;
            border-radius: 4px;
            font-size: 14px;
        }
        .value-edit select:focus {
            outline: none;
            border-color: #007bff;
            box-shadow: 0 0 0 2px rgba(0,123,255,0.25);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>DaMiao Motor Parameter Editor</h1>
        <p class="subtitle">View and modify motor register parameters via CAN bus</p>
        
        <div id="status" class="status"></div>
        
        <div class="section">
            <div class="section-title">Connection</div>
            <div class="config-section">
                <label for="can_channel">CAN Channel:</label>
                <input type="text" id="can_channel" value="can0" placeholder="can0">
                <button class="btn btn-primary" onclick="connect()">Connect</button>
                <button class="btn btn-secondary" onclick="disconnect()">Disconnect</button>
            </div>
        </div>
        
        <div class="section">
            <div class="section-title">Motor Selection</div>
            <div class="motor-selector">
                <button class="btn btn-primary" onclick="scanMotors()">Scan Motors</button>
                <button class="btn btn-success" onclick="refreshRegisters()" id="refreshBtn" disabled>Refresh Registers</button>
                <select id="motorSelect" onchange="loadMotorRegisters()" style="padding: 10px; border-radius: 5px; border: 1px solid #ced4da; min-width: 200px;" disabled>
                    <option value="">Select a motor...</option>
                </select>
            </div>
        </div>
        
        <div class="section">
            <div class="section-title">Register Parameters</div>
            <div id="registerTableContainer">
                <div class="loading">
                    <div class="spinner"></div>
                    <p>Select a motor to view registers</p>
                </div>
            </div>
        </div>
    </div>

    <script>
        let currentMotorId = null;
        let originalValues = {};

        function showStatus(message, type = 'info') {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = 'status ' + type;
            status.style.display = 'block';
            setTimeout(() => {
                status.style.display = 'none';
            }, 5000);
        }

        async function connect() {
            const channel = document.getElementById('can_channel').value;
            if (!channel) {
                showStatus('Please enter a CAN channel (e.g., can0)', 'error');
                return;
            }
            
            const connectBtn = event.target;
            const originalText = connectBtn.textContent;
            connectBtn.disabled = true;
            connectBtn.textContent = 'Connecting...';
            showStatus('Connecting to CAN bus...', 'info');
            
            try {
                const response = await fetch('/api/connect', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({channel: channel})
                });
                const data = await response.json();
                if (data.success) {
                    showStatus('Connected to CAN bus: ' + channel, 'success');
                } else {
                    showStatus('Connection failed: ' + data.error, 'error');
                }
            } catch (error) {
                console.error('Connect error:', error);
                showStatus('Error: ' + error.message, 'error');
            } finally {
                connectBtn.disabled = false;
                connectBtn.textContent = originalText;
            }
        }

        async function disconnect() {
            try {
                const response = await fetch('/api/disconnect', {method: 'POST'});
                const data = await response.json();
                if (data.success) {
                    showStatus('Disconnected', 'info');
                    document.getElementById('motorSelect').innerHTML = '<option value="">Select a motor...</option>';
                    document.getElementById('motorSelect').disabled = true;
                    document.getElementById('refreshBtn').disabled = true;
                    document.getElementById('registerTableContainer').innerHTML = 
                        '<div class="loading"><div class="spinner"></div><p>Select a motor to view registers</p></div>';
                }
            } catch (error) {
                showStatus('Error: ' + error.message, 'error');
            }
        }

        async function scanMotors(silent = false) {
            const scanBtn = event ? event.target : document.querySelector('button[onclick="scanMotors()"]');
            const originalText = scanBtn ? scanBtn.textContent : 'Scan Motors';
            if (scanBtn) {
                scanBtn.disabled = true;
                scanBtn.textContent = 'Scanning...';
            }
            if (!silent) {
                showStatus('Scanning for motors...', 'info');
            }
            
            try {
                const response = await fetch('/api/scan', {method: 'POST'});
                if (!response.ok) {
                    const errorText = await response.text();
                    throw new Error(`HTTP ${response.status}: ${errorText}`);
                }
                const data = await response.json();
                console.log('Scan response:', data);
                
                if (data.success) {
                    const select = document.getElementById('motorSelect');
                    select.innerHTML = '<option value="">Select a motor...</option>';
                    if (data.motors && data.motors.length > 0) {
                        data.motors.forEach(motor => {
                            const option = document.createElement('option');
                            option.value = motor.id;
                            option.textContent = `Motor 0x${motor.id.toString(16).toUpperCase().padStart(2, '0')} (${motor.id}) - Arb ID: 0x${motor.arb_id.toString(16).toUpperCase().padStart(3, '0')}`;
                            select.appendChild(option);
                        });
                        select.disabled = false;
                        document.getElementById('refreshBtn').disabled = false;
                        
                        // Restore previous selection if motor still exists
                        if (currentMotorId) {
                            const motorExists = data.motors.some(m => m.id === currentMotorId);
                            if (motorExists) {
                                select.value = currentMotorId;
                            } else {
                                currentMotorId = null;
                            }
                        }
                        
                        if (!silent) {
                            showStatus(`Found ${data.motors.length} motor(s)`, 'success');
                        }
                    } else {
                        if (!silent) {
                            showStatus('No motors found. Make sure motors are connected, powered, and CAN bus is configured.', 'info');
                        }
                    }
                } else {
                    if (!silent) {
                        showStatus('Scan failed: ' + (data.error || 'Unknown error'), 'error');
                    }
                }
            } catch (error) {
                console.error('Scan error:', error);
                if (!silent) {
                    showStatus('Error: ' + error.message, 'error');
                }
            } finally {
                if (scanBtn) {
                    scanBtn.disabled = false;
                    scanBtn.textContent = originalText;
                }
            }
        }

        async function loadMotorRegisters() {
            const select = document.getElementById('motorSelect');
            const motorId = parseInt(select.value);
            if (!motorId) {
                document.getElementById('registerTableContainer').innerHTML = 
                    '<div class="loading"><div class="spinner"></div><p>Select a motor to view registers</p></div>';
                return;
            }
            
            currentMotorId = motorId;
            showStatus('Loading registers...', 'info');
            
            try {
                const response = await fetch(`/api/motors/${motorId}/registers`);
                const data = await response.json();
                if (data.success) {
                    displayRegisters(data.registers);
                    originalValues = {...data.registers};
                    showStatus('Registers loaded', 'success');
                } else {
                    showStatus('Failed to load registers: ' + data.error, 'error');
                }
            } catch (error) {
                showStatus('Error: ' + error.message, 'error');
            }
        }

        function displayRegisters(registers) {
            const container = document.getElementById('registerTableContainer');
            let html = '<table class="register-table"><thead><tr>';
            html += '<th>RID</th><th>Variable</th><th>Description</th><th>Value</th><th>Type</th><th>Access</th><th>Action</th>';
            html += '</tr></thead><tbody>';
            
            const sortedRids = Object.keys(registers).map(Number).sort((a, b) => a - b);
            
            sortedRids.forEach(rid => {
                const value = registers[rid];
                const regInfo = window.registerTable[rid];
                if (!regInfo) return;
                
                const isReadOnly = regInfo.access === 'RO';
                
                // Special handling for hex registers (7, 8) and dropdown (35)
                let valueStr, inputHtml;
                
                if (rid === 35) {
                    // CAN baud rate dropdown
                    const baudRateOptions = [
                        {code: 0, label: '125K (0)'},
                        {code: 1, label: '200K (1)'},
                        {code: 2, label: '250K (2)'},
                        {code: 3, label: '500K (3)'},
                        {code: 4, label: '1M (4)'}
                    ];
                    const currentValue = typeof value === 'string' ? 0 : parseInt(value);
                    valueStr = baudRateOptions.find(opt => opt.code === currentValue)?.label || `Unknown (${currentValue})`;
                    
                    if (!isReadOnly) {
                        inputHtml = `<select id="input-${rid}">`;
                        baudRateOptions.forEach(opt => {
                            inputHtml += `<option value="${opt.code}" ${opt.code === currentValue ? 'selected' : ''}>${opt.label}</option>`;
                        });
                        inputHtml += `</select>`;
                    }
                } else if (rid === 7 || rid === 8) {
                    // Hex display for MST_ID and ESC_ID
                    const numValue = typeof value === 'string' ? 0 : parseInt(value);
                    valueStr = `0x${numValue.toString(16).toUpperCase().padStart(3, '0')} (${numValue})`;
                    
                    if (!isReadOnly) {
                        inputHtml = `<input type="text" id="input-${rid}" value="0x${numValue.toString(16).toUpperCase().padStart(3, '0')}" placeholder="0x000" style="font-family: monospace;">`;
                    }
                } else {
                    // Regular numeric display
                    valueStr = typeof value === 'string' ? value : 
                        (regInfo.data_type === 'float' ? parseFloat(value).toFixed(6) : String(parseInt(value)));
                    
                    if (!isReadOnly) {
                        inputHtml = `<input type="number" step="${regInfo.data_type === 'float' ? '0.000001' : '1'}" id="input-${rid}" value="${valueStr}">`;
                    }
                }
                
                html += `<tr class="${isReadOnly ? 'read-only' : ''}">`;
                html += `<td>${rid}</td>`;
                html += `<td>${regInfo.variable}</td>`;
                html += `<td>${regInfo.description}</td>`;
                html += `<td class="value-cell">`;
                html += `<span class="value-display" id="value-${rid}">${valueStr}</span>`;
                if (!isReadOnly) {
                    html += `<span class="value-edit" id="edit-${rid}">`;
                    html += inputHtml;
                    html += `</span>`;
                }
                html += `</td>`;
                html += `<td>${regInfo.data_type}</td>`;
                html += `<td>${regInfo.access}</td>`;
                html += `<td>`;
                if (!isReadOnly) {
                    html += `<button class="edit-btn" onclick="editRegister(${rid})" id="edit-btn-${rid}">Edit</button>`;
                    html += `<button class="save-btn" onclick="saveRegister(${rid})" id="save-btn-${rid}" style="display:none;">Save</button>`;
                    html += `<button class="cancel-btn" onclick="cancelEdit(${rid})" id="cancel-btn-${rid}" style="display:none;">Cancel</button>`;
                } else {
                    html += `<span style="color: #999;">Read Only</span>`;
                }
                html += `</td>`;
                html += `</tr>`;
            });
            
            html += '</tbody></table>';
            container.innerHTML = html;
        }

        function editRegister(rid) {
            document.getElementById(`value-${rid}`).style.display = 'none';
            document.getElementById(`edit-${rid}`).style.display = 'flex';
            document.getElementById(`edit-btn-${rid}`).style.display = 'none';
            document.getElementById(`save-btn-${rid}`).style.display = 'inline-block';
            document.getElementById(`cancel-btn-${rid}`).style.display = 'inline-block';
        }

        function cancelEdit(rid) {
            document.getElementById(`value-${rid}`).style.display = 'flex';
            document.getElementById(`edit-${rid}`).style.display = 'none';
            document.getElementById(`edit-btn-${rid}`).style.display = 'inline-block';
            document.getElementById(`save-btn-${rid}`).style.display = 'none';
            document.getElementById(`cancel-btn-${rid}`).style.display = 'none';
            
            // Restore original value with proper formatting
            const regInfo = window.registerTable[rid];
            const originalValue = originalValues[rid];
            const input = document.getElementById(`input-${rid}`);
            
            if (rid === 35) {
                input.value = originalValue;
            } else if (rid === 7 || rid === 8) {
                const numValue = typeof originalValue === 'string' ? 0 : parseInt(originalValue);
                input.value = `0x${numValue.toString(16).toUpperCase().padStart(3, '0')}`;
            } else {
                input.value = originalValue;
            }
        }

        async function saveRegister(rid) {
            const input = document.getElementById(`input-${rid}`);
            const regInfo = window.registerTable[rid];
            let newValue;
            
            // Special handling for different input types
            if (rid === 35) {
                // Dropdown for CAN baud rate
                newValue = parseInt(input.value);
            } else if (rid === 7 || rid === 8) {
                // Hex input for MST_ID and ESC_ID
                const inputValue = input.value.trim();
                if (inputValue.startsWith('0x') || inputValue.startsWith('0X')) {
                    newValue = parseInt(inputValue, 16);
                } else {
                    newValue = parseInt(inputValue, 16); // Try hex first
                    if (isNaN(newValue)) {
                        newValue = parseInt(inputValue, 10); // Fall back to decimal
                    }
                }
            } else {
                // Regular numeric input
                newValue = regInfo.data_type === 'float' ? parseFloat(input.value) : parseInt(input.value);
            }
            
            if (isNaN(newValue)) {
                showStatus('Invalid value. Please enter a valid number.', 'error');
                return;
            }
            
            showStatus('Saving register...', 'info');
            
            try {
                const response = await fetch(`/api/motors/${currentMotorId}/registers/${rid}`, {
                    method: 'PUT',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({value: newValue})
                });
                
                // Check if response is OK before parsing JSON
                if (!response.ok) {
                    const errorText = await response.text();
                    let errorMsg = `HTTP ${response.status}: ${response.statusText}`;
                    try {
                        const errorJson = JSON.parse(errorText);
                        errorMsg = errorJson.error || errorMsg;
                    } catch (e) {
                        errorMsg = errorText || errorMsg;
                    }
                    throw new Error(errorMsg);
                }
                
                const data = await response.json();
                if (data.success) {
                    cancelEdit(rid);
                    
                    // If we changed register 7 (feedback_id) or 8 (motor_id), update currentMotorId
                    if (data.updated_ids) {
                        if (data.updated_ids.motor_id !== undefined) {
                            // Motor ID changed, update our reference
                            const oldMotorId = currentMotorId;
                            currentMotorId = data.updated_ids.motor_id;
                            showStatus(`Register saved. Motor ID changed from ${oldMotorId} to ${currentMotorId}. Rescanning...`, 'info');
                        } else if (data.updated_ids.feedback_id !== undefined) {
                            showStatus(`Register saved. Feedback ID changed to ${data.updated_ids.feedback_id}. Rescanning...`, 'info');
                        }
                    } else {
                        showStatus('Register saved. Rescanning motors and reloading registers...', 'info');
                    }
                    
                    // Rescan motors to find motor with new IDs
                    await scanMotors(true);
                    
                    // Update motor selection if motor_id changed
                    if (data.updated_ids && data.updated_ids.motor_id !== undefined) {
                        const select = document.getElementById('motorSelect');
                        select.value = currentMotorId;
                    }
                    
                    // Reload registers for the current motor
                    if (currentMotorId) {
                        await loadMotorRegisters();
                    }
                    
                    showStatus('Register saved and data refreshed', 'success');
                } else {
                    showStatus('Failed to save: ' + (data.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                console.error('Save register error:', error);
                showStatus('Error: ' + error.message, 'error');
            }
        }

        async function refreshRegisters() {
            if (currentMotorId) {
                await loadMotorRegisters();
            }
        }

        // Load register table on page load
        window.addEventListener('DOMContentLoaded', async () => {
            try {
                const response = await fetch('/api/register-table');
                const data = await response.json();
                window.registerTable = {};
                data.registers.forEach(reg => {
                    window.registerTable[reg.rid] = reg;
                });
            } catch (error) {
                console.error('Failed to load register table:', error);
            }
        });
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    """Serve the main GUI page."""
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/register-table', methods=['GET'])
def get_register_table():
    """Get the register table information."""
    registers = []
    for rid, reg_info in REGISTER_TABLE.items():
        registers.append({
            'rid': reg_info.rid,
            'variable': reg_info.variable,
            'description': reg_info.description,
            'access': reg_info.access,
            'range_str': reg_info.range_str,
            'data_type': reg_info.data_type,
        })
    return jsonify({'success': True, 'registers': registers})


@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to CAN bus."""
    try:
        data = request.json
        channel = data.get('channel', 'can0')
        init_controller(channel=channel)
        return jsonify({'success': True})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from CAN bus."""
    global _controller, _motors
    try:
        if _controller:
            _controller.shutdown()  # Use shutdown() which properly stops polling
        _controller = None
        _motors = {}
        return jsonify({'success': True})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/scan', methods=['POST'])
def scan():
    """Scan for motors."""
    global _controller, _motors
    try:
        if _controller is None:
            return jsonify({'success': False, 'error': 'Not connected. Please connect first.'}), 400
        
        # Clear existing motors
        _motors = {}
        _controller.motors = {}
        _controller._motors_by_feedback = {}
        
        # Flush bus
        _controller.flush_bus()
        
        # Send zero commands to potential motor IDs (0x01-0x10)
        motors_found = []
        for motor_id in range(0x01, 0x11):
            try:
                motor = _controller.add_motor(motor_id=motor_id, feedback_id=0x00)
                motor.send_cmd(target_position=0.0, target_velocity=0.0, stiffness=0.0, damping=0.0, feedforward_torque=0.0)
            except ValueError:
                pass  # Motor already exists
            except Exception as e:
                # Log but continue
                print(f"Warning: Failed to send command to motor {motor_id}: {e}")
        
        # Listen for responses
        import time
        start_time = time.perf_counter()
        responded = set()
        
        while time.perf_counter() - start_time < 0.5:
            _controller.poll_feedback()
            for motor_id, motor in _controller.motors.items():
                if motor.state and motor.state.get("can_id") is not None:
                    if motor_id not in responded:
                        arb_id = motor.state.get("arbitration_id")
                        motors_found.append({
                            'id': motor_id,
                            'arb_id': arb_id if arb_id is not None else 0,
                        })
                        responded.add(motor_id)
            time.sleep(0.01)
        
        _motors = {m['id']: _controller.motors[m['id']] for m in motors_found}
        
        return jsonify({'success': True, 'motors': motors_found})
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Scan error: {error_msg}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/motors/<int:motor_id>/registers', methods=['GET'])
def get_registers(motor_id: int):
    """Get all registers for a motor."""
    global _motors
    try:
        if motor_id not in _motors:
            return jsonify({'success': False, 'error': f'Motor {motor_id} not found'}), 404
        
        motor = _motors[motor_id]
        registers = motor.read_all_registers(timeout=0.05)
        
        # Filter out error strings
        clean_registers = {}
        for rid, value in registers.items():
            if not isinstance(value, str) or not value.startswith("ERROR"):
                clean_registers[rid] = value
        
        return jsonify({'success': True, 'registers': clean_registers})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/motors/<int:motor_id>/registers/<int:rid>', methods=['PUT'])
def set_register(motor_id: int, rid: int):
    """Set a register value."""
    global _controller, _motors
    try:
        if motor_id not in _motors:
            return jsonify({'success': False, 'error': f'Motor {motor_id} not found'}), 404
        
        if not request.is_json:
            return jsonify({'success': False, 'error': 'Content-Type must be application/json'}), 400
        
        data = request.get_json()
        if data is None:
            return jsonify({'success': False, 'error': 'Invalid JSON in request body'}), 400
        
        value = data.get('value')
        
        if value is None:
            return jsonify({'success': False, 'error': 'Value is required'}), 400
        
        motor = _motors[motor_id]
        motor.write_register(rid, value)
        
        # If we changed register 7 (MST_ID/feedback_id) or 8 (ESC_ID/receive_id),
        # we need to update the motor's IDs and controller mappings
        updated_ids = {}
        if rid == 7:  # MST_ID - feedback ID
            new_feedback_id = int(value)
            old_feedback_id = motor.feedback_id
            motor.feedback_id = new_feedback_id
            
            # Update controller's feedback mapping
            if old_feedback_id in _controller._motors_by_feedback:
                del _controller._motors_by_feedback[old_feedback_id]
            _controller._motors_by_feedback[new_feedback_id] = motor
            
            updated_ids['feedback_id'] = new_feedback_id
            
        elif rid == 8:  # ESC_ID - receive/command ID
            new_motor_id = int(value)
            old_motor_id = motor.motor_id
            
            # Update motor's motor_id
            motor.motor_id = new_motor_id
            
            # Update controller's motor mapping
            if old_motor_id in _controller.motors:
                del _controller.motors[old_motor_id]
            _controller.motors[new_motor_id] = motor
            
            # Update _motors dict
            if old_motor_id in _motors:
                _motors[new_motor_id] = _motors.pop(old_motor_id)
            
            updated_ids['motor_id'] = new_motor_id
        
        # Store parameters to flash if it's a critical register
        if rid in [7, 8]:
            try:
                motor.store_parameters()
            except:
                pass  # Ignore errors, motor might not support it
        
        return jsonify({'success': True, 'updated_ids': updated_ids})
    except KeyError as e:
        return jsonify({'success': False, 'error': f'Register {rid} not found in register table'}), 400
    except ValueError as e:
        return jsonify({'success': False, 'error': f'Invalid value: {str(e)}'}), 400
    except TimeoutError as e:
        return jsonify({'success': False, 'error': f'Timeout writing register: {str(e)}'}), 500
    except Exception as e:
        import traceback
        error_msg = f"{str(e)}\n{traceback.format_exc()}"
        print(f"Set register error: {error_msg}")
        return jsonify({'success': False, 'error': str(e)}), 500


def main():
    """Run the web GUI server."""
    import argparse
    import warnings
    parser = argparse.ArgumentParser(description="Web-based GUI for DaMiao motor parameters")
    parser.add_argument('--host', default='127.0.0.1', help='Host to bind to (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to (default: 5000)')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    parser.add_argument('--production', action='store_true', help='Use production WSGI server (requires waitress)')
    args = parser.parse_args()
    
    print(f"Starting DaMiao Motor Parameter Editor...")
    print(f"Open http://{args.host}:{args.port} in your browser")
    
    if args.production:
        try:
            from waitress import serve
            print("Using Waitress production server")
            serve(app, host=args.host, port=args.port)
        except ImportError:
            print("Warning: waitress not installed. Install with: pip install waitress")
            print("Falling back to development server...")
            # Suppress the warning for development server
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            app.run(host=args.host, port=args.port, debug=args.debug)
    else:
        # Suppress the development server warning for local use
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        warnings.filterwarnings('ignore', message='.*development server.*')
        app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()

