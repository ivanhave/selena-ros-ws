CAN CONFIGURATION NOTES (You have a pdf with this info in this folder. Open it separately, won
t work to open it in VS Code)

This document serves as a persistent record of the SocketCAN automation setup for the
Selena robot hardware interface. This prevents the "magic" of the automatic connection
from being lost during future development.

1. Location of the Configuration

The automation is handled via a udev rule. This rule triggers as soon as the PEAK-System USB-to-CAN adapter is plugged in.

File path: /etc/udev/rules.d/80-can.rules

2. Current Configuration Rule
The following line is what initializes the can0 interface at the required bitrate:

SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add", RUN+="/sbin/ip link set %k up type can bitrate 500000"

3. How to Edit the Rule

To modify settings (like increasing the bitrate to 1,000,000 for 4 motors), open the file via terminal using gedit with root privileges:

sudo gedit /etc/udev/rules.d/80-can.rules

4. Applying Changes
After saving the file in gedit, you must reload the udev rules for the changes to take effect immediately:

sudo udevadm control --reload-rules
sudo udevadm trigger

5. Scaling to Full Gantry (4 Motors)

Optimization Tip: When moving from one motor to the full 4-motor agricultural gantry,
the CAN bus traffic will increase. If you experience dropped frames or latency, update
the rule to include a larger transmit queue:
...type can bitrate 500000 txqueuelen 1000 (ALREADY DONE)

6. Hardware Verification

To verify the interface state and details manually, use:

ip -details link show can0

Expected adapter type: pcan_usb (PEAK-System).

7. Terminal commmands in case it doesn't start automatically:

    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 txqueuelen 1000
    sudo ip link set can0 up