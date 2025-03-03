IMPORTANT! To run these codes, the following dependencies are necessary:

pip install python-can

pip install pyserial

pip install kivy

On windows the USB/CAN interface can be available in the 'COM' ports list(Ex. COM5).

On Linux can be available as slcan interface, to find the connection you can use the command: ls /dev/tty* (ex. /dev/ttyACM0). The command lsusb can help to find the list of connected usb devices. Another option is: sudo dmesg | grep tty Run this command to identify the serial port associated with the interface. To run de software on linux execute the sudo main.py in the project folder.
