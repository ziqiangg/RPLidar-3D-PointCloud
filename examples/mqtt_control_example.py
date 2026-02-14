"""
Example: Controlling an edge device RPLidar via MQTT

This script demonstrates how to send scan commands to an edge device
running the RPLidar scanner. The viewer application receives status updates
and scan results from the edge device.

Usage:
    python examples/mqtt_control_example.py
"""

import paho.mqtt.client as mqtt
import json
import time

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "rplidar/scan/command"   # Send commands to edge device
MQTT_TOPIC_STATUS = "rplidar/scan/status"     # Receive status from edge device
MQTT_TOPIC_RESULT = "rplidar/scan/result"     # Receive results from edge device


def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker."""
    if rc == 0:
        print(f"✓ Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        # Subscribe to status topics from edge device
        client.subscribe(MQTT_TOPIC_STATUS)
        client.subscribe(MQTT_TOPIC_RESULT)
        print(f"✓ Subscribed to status topics")
    else:
        print(f"✗ Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Callback when a message is received."""
    try:
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == MQTT_TOPIC_STATUS:
            status = payload.get("status", "")
            message = payload.get("message", "")
            print(f"[EDGE STATUS] {status}: {message}")
        
        elif msg.topic == MQTT_TOPIC_RESULT:
            scan_type = payload.get("scan_type", "")
            file_path = payload.get("file_path", "")
            success = payload.get("success", True)
            if success:
                print(f"[SCAN RESULT] {scan_type.upper()} scan completed: {file_path}")
            else:
                print(f"[SCAN RESULT] {scan_type.upper()} scan failed")
    
    except json.JSONDecodeError:
        print(f"[ERROR] Invalid JSON in message: {msg.payload}")


def send_scan_command(client, scan_type="2d", params=None):
    """
    Send a scan command to the edge device.
    
    Args:
        client: MQTT client instance
        scan_type: "2d" or "3d"
        params: Additional parameters (port, etc.)
    """
    command = {
        "scan_type": scan_type,
        "params": params or {}
    }
    
    payload = json.dumps(command)
    result = client.publish(MQTT_TOPIC_COMMAND, payload, qos=1)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"✓ Sent command to edge device: {scan_type.upper()} scan")
    else:
        print(f"✗ Failed to send command (error code: {result.rc})")
    
    return result.rc == mqtt.MQTT_ERR_SUCCESS


def main():
    """Main demonstration function."""
    print("="*70)
    print("RPLidar Edge Device MQTT Control Example")
    print("="*70)
    print()
    print("This script sends scan commands to an RPLidar edge device via MQTT.")
    print("The viewer application will receive status updates and scan results.")
    print()
    print("Make sure:")
    print("  1. MQTT broker is running (mosquitto)")
    print("  2. Edge device is running and connected to MQTT")
    print("  3. Viewer is running: python viewer/app.py")
    print()
    print("="*70)
    print()
    
    # Create MQTT client
    client = mqtt.Client(client_id="mqtt_control_example")
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        # Connect to broker
        print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Send a 2D scan command to edge device
        print()
        print("Sending 2D scan command to edge device...")
        send_scan_command(client, scan_type="2d")
        
        # Wait for scan to complete on edge device
        print()
        print("Waiting for edge device to complete scan...")
        print("(Watch the status messages above)")
        print()
        time.sleep(20)  # Wait 20 seconds for scan to complete
        
        # Optionally send a 3D scan command
        # Uncomment to test 3D scanning:
        # print()
        # print("Sending 3D scan command...")
        # send_scan_command(client, scan_type="3d")
        # time.sleep(30)
        
        print()
        print("="*70)
        print("Example complete!")
        print("="*70)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\n\nError: {e}")
    
    finally:
        # Cleanup
        print("\nDisconnecting...")
        client.loop_stop()
        client.disconnect()
        print("Done!")


def interactive_mode():
    """Interactive mode for sending custom commands."""
    print("="*70)
    print("RPLidar MQTT Interactive Control")
    print("="*70)
    print()
    
    client = mqtt.Client(client_id="mqtt_interactive")
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print()
        print("Commands:")
        print("  2d [port]  - Start 2D scan (port optional)")
        print("  3d [port]  - Start 3D scan (port optional)")
        print("  quit       - Exit")
        print()
        
        while True:
            try:
                cmd = input(">>> ").strip().lower()
                
                if cmd == "quit" or cmd == "exit" or cmd == "q":
                    break
                
                parts = cmd.split()
                if not parts:
                    continue
                
                scan_type = parts[0]
                port = parts[1] if len(parts) > 1 else None
                
                if scan_type in ["2d", "3d"]:
                    send_scan_command(client, scan_type, port)
                else:
                    print("Unknown command. Use '2d', '3d', or 'quit'")
            
            except EOFError:
                break
        
    except KeyboardInterrupt:
        print("\n")
    
    finally:
        client.loop_stop()
        client.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "-i":
        # Interactive mode
        interactive_mode()
    else:
        # Automated example
        main()
