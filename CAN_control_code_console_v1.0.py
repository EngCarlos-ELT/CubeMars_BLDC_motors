# -*- coding: utf-8 -*-
# CubeMars AK80-64 - CAN Control (MIT / SERVO Modes)
# Requirements:
# pip install python-can

import can
import struct
import time
import platform

# --- AK80-64 motor constants ---
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -8.0, 8.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -144.0, 144.0

# --- Automatic CAN IDs per mode ---
MIT_ID = 1        # Default for MIT firmware
SERVO_ID = 104    # Default for Servo firmware


# --- Motor state container ---
class MotorState:
    def __init__(self):
        self.p_in = 0.0
        self.v_in = 0.0
        self.kp_in = 0.0
        self.kd_in = 0.50
        self.t_in = 0.0
        self.p_out = 0.0
        self.v_out = 0.0
        self.t_out = 0.0


# --- Helper functions ---
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return (float(x_int) * span / ((1 << bits) - 1)) + x_min


# --- MIT Mode control functions ---
def mit_enter_mode(bus, motor_id):
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False)
    bus.send(msg)
    print(f"→ [MIT] Entering MIT mode (ID={motor_id})")
    time.sleep(0.1)


def mit_exit_mode(bus, motor_id):
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False)
    bus.send(msg)
    print(f"→ [MIT] Exiting MIT mode (ID={motor_id})")
    time.sleep(0.1)


def mit_zero_position(bus, motor_id):
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFE], is_extended_id=False)
    bus.send(msg)
    print(f"→ [MIT] Zeroing position (ID={motor_id})")
    time.sleep(0.1)


def mit_pack_cmd(bus, motor_state, motor_id):
    """Pack and send a command in MIT format"""
    # Clamp input values
    p_des = max(min(motor_state.p_in, P_MAX), P_MIN)
    v_des = max(min(motor_state.v_in, V_MAX), V_MIN)
    kp = max(min(motor_state.kp_in, KP_MAX), KP_MIN)
    kd = max(min(motor_state.kd_in, KD_MAX), KD_MIN)
    t_ff = max(min(motor_state.t_in, T_MAX), T_MIN)

    # Convert floats to integers
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    # Pack into 8 bytes
    buf = bytearray(8)
    buf[0] = p_int >> 8
    buf[1] = p_int & 0xFF
    buf[2] = v_int >> 4
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    buf[4] = kp_int & 0xFF
    buf[5] = kd_int >> 4
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    buf[7] = t_int & 0xFF

    msg = can.Message(arbitration_id=motor_id, data=buf, is_extended_id=False)
    bus.send(msg)
    print(f"→ [MIT] P={p_des:.2f} rad, KP={kp:.1f}, KD={kd:.2f}, T={t_ff:.2f}")


# --- SERVO Mode function ---
def servo_send_command(bus, pos_deg, speed_erpm, accel_erpm_s2, motor_id):
    """Send a position-velocity command using the Servo mode"""
    POS_SPEED_LOOP_MODE = 6
    can_id = (POS_SPEED_LOOP_MODE << 8) | motor_id  # Extended ID format

    pos_val = int(pos_deg * 10000)
    speed_val = int(speed_erpm / 10)
    accel_val = int(accel_erpm_s2 / 10)

    data = struct.pack('>ihh', pos_val, speed_val, accel_val)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    bus.send(msg)
    print(f"→ [SERVO] Pos={pos_deg:.1f}°, Vel={speed_erpm} eRPM, Accel={accel_erpm_s2} eRPM/s²")


# --- CAN interface detection ---
def detect_can_interface():
    system = platform.system()
    if system == 'Windows':
        interface = 'slcan'
        channel = 'COM9'
    elif system == 'Linux':
        interface = 'slcan'
        channel = '/dev/ttyACM0'
        print("Ensure CAN interface is up: sudo ip link set can0 up type can bitrate 1000000")
    else:
        interface = 'slcan'
        channel = 'COM9'
    return interface, channel


# --- Menu and user input ---
def print_menu():
    print("\n=== CubeMars AK80-64 Control ===")
    print("1. MIT Mode - Enter")
    print("2. MIT Mode - Send Command")
    print("3. MIT Mode - Exit")
    print("4. SERVO Mode - Test 90° and Return")
    print("q. Quit")
    print("Select an option: ", end='')


def get_float_input(prompt, min_val, max_val):
    while True:
        try:
            val = float(input(f"{prompt} ({min_val} to {max_val}): "))
            if min_val <= val <= max_val:
                return val
            print("Value out of range.")
        except ValueError:
            print("Invalid input. Please enter a number.")


# --- Main program ---
def main():
    can_interface, channel = detect_can_interface()
    bitrate = 1000000
    motor_state = MotorState()

    try:
        bus = can.interface.Bus(channel=channel, interface=can_interface, bitrate=bitrate)
        print(f"✅ Connected to CAN interface {can_interface} ({channel})")

        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            # --- MIT Mode ---
            elif choice == '1':
                mit_enter_mode(bus, MIT_ID)

            elif choice == '2':
                print("\nEnter MIT Control Parameters:")
                motor_state.p_in = get_float_input("Position (rad)", P_MIN, P_MAX)
                motor_state.v_in = get_float_input("Velocity (rad/s)", V_MIN, V_MAX)
                motor_state.kp_in = get_float_input("Kp", KP_MIN, KP_MAX)
                motor_state.kd_in = get_float_input("Kd", KD_MIN, KD_MAX)
                motor_state.t_in = get_float_input("Torque (Nm)", T_MIN, T_MAX)
                mit_pack_cmd(bus, motor_state, MIT_ID)

            elif choice == '3':
                mit_exit_mode(bus, MIT_ID)

            # --- SERVO Mode ---
            elif choice == '4':
                print("→ Running SERVO mode test (ID=104)")
                target_speed = 5000
                target_accel = 10000
                servo_send_command(bus, 90.0, target_speed, target_accel, SERVO_ID)
                time.sleep(3)
                servo_send_command(bus, 0.0, target_speed, target_accel, SERVO_ID)
                time.sleep(3)
                print("✅ SERVO mode test complete.")

            else:
                print("Invalid option.")

            time.sleep(0.1)

    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            mit_exit_mode(bus, MIT_ID)
            bus.shutdown()
        except:
            pass
        print("🛑 CAN interface closed.")


if __name__ == "__main__":
    main()
