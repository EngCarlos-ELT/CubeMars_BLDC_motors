IMPORTANT! To run these codes, the following dependencies are necessary:

pip install python-can

pip install pyserial

pip install kivy

On windows the USB/CAN interface can be available in the 'COM' ports list(Ex. COM5).

On Linux can be available as slcan interface, to find the connection you can use the command: ls /dev/tty* (ex. /dev/ttyACM0). The command lsusb can help to find the list of connected usb devices. Another option is: sudo dmesg | grep tty Run this command to identify the serial port associated with the interface. To run de software on linux execute the sudo main.py in the project folder.



-can_usb_test_code.py This Python code configures and interacts with a CAN (Controller Area Network) interface, adapting to the operating system (Windows or Linux) to set the appropriate CAN interface, channel, and bitrate. It initializes a CAN bus connection, sends a predefined CAN message, and continuously listens for incoming messages, printing them to the console. The script handles errors (e.g., CAN errors) and allows graceful termination with a keyboard interrupt, ensuring the CAN interface is properly shut down before exiting. It is designed for testing and communication with CAN devices, such as those used in embedded systems or automotive applications.

-CAN_control_code_console_v1.0.py This Python code provides a command-line interface to control a CubeMars AK80-64 motor using a CAN (Controller Area Network) interface. It defines motor control parameters (e.g., position, velocity, torque) and includes functions to send commands, enter/exit MIT mode, zero the motor position, and read motor status. The script automatically detects the operating system to configure the appropriate CAN interface and channel. It uses a menu-driven approach, allowing the user to interactively input control parameters and execute commands. The motor's status, including position, velocity, and torque, can be read and displayed in real time. This script is designed for embedded systems or robotics applications, offering a flexible and interactive way to control the motor. The original code was written by Cocorock (Victor Ferman).

-CAN_control_code_kivy_v1.0.py This Python code is a GUI-based application for controlling a CubeMars AK80-64 motor using a CAN (Controller Area Network) interface, built with the Kivy framework. It provides a user-friendly interface with sliders for adjusting motor parameters (position, velocity, torque, etc.), buttons for entering/exiting MIT mode, zeroing the motor position, and sending control commands. The app automatically detects the operating system to configure the appropriate CAN interface and channel. It also includes real-time status updates for motor position, velocity, and torque, displayed in the GUI. The application supports continuous control with adjustable intervals and uses threading to handle CAN communication without blocking the GUI. This script is designed for robotics or embedded systems applications, offering an intuitive way to interact with and control the motor.
