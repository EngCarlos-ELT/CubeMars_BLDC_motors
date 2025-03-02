# pip install pyserial
# pip install python-can
import can
import time
import platform

def main():
    # Detects the operating system
    system = platform.system()

    if system == "Windows":
        # Configuration for Windows
        can_interface = 'slcan'  # Use 'slcan' for Canable devices on Windows
        channel = 'COM5'  # Replace with your COM port (check in Device Manager)
        bitrate = 1000000  # Bitrate (1 Mbps)
    elif system == "Linux":
        # Configuration for Linux
        can_interface = 'socketcan'  # Use 'socketcan' for Canable devices on Linux
        channel = 'can0'  # Replace with the CAN device name (e.g., can0, can1, etc.)
        bitrate = None  # On Linux, the bitrate is configured outside the script (e.g., with `ip link set can0 up type can bitrate 1000000`)
    else:
        print(f"Unsupported operating system: {system}")
        return

    try:
        # Creates a CAN bus instance
        bus = can.interface.Bus(channel=channel, interface=can_interface, bitrate=bitrate)
        print(f"Connected to CAN interface: {can_interface} on channel {channel}")

        # Sends a CAN message
        message = can.Message(arbitration_id=0x123, data=[0x15, 0x21, 0x33, 0x44, 0x55, 0x66, 0x78, 0x87], is_extended_id=False)
        bus.send(message)
        print(f"Message sent: {message}")

        # Receives CAN messages
        print("Waiting for CAN messages...")
        while True:
            msg = bus.recv(timeout=1)  # Waits for a message (timeout in seconds)
            if msg:
                print(f"Message received: {msg}")
            else:
                print("No message received, waiting...")

    except can.CanError as e:
        print(f"CAN error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Shuts down the CAN bus
        if 'bus' in locals():
            bus.shutdown()
        print("CAN interface closed.")

if __name__ == "__main__":
    main()
