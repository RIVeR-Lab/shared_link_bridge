# Installation

1. Clone this repo

2. Install joy package (for controller support)
```
sudo apt install ros-humble-joy
```
3. Install the xterm package (for gui generation)
```bash
sudo apt install xterm
```

# Usage

Utilize the launch file and run the following command 

```bash
ros2 launch shared_link_bridge bringup.launch.py
```

### Controller Controls

The controller_teleop.py inside of the launch file will generate a gui using xterm. Control using the DPad, and press `A` to toggle the connect and teleop on buttons. The right DPad will then scroll for the shifting options. The right of the screen will provide real time updates on the values, including the frequency of publishes from /joy and /kairos_values. 

The right bumber is the deadman's switch and pressing that allows you to use the left joystick at the throttle and brake, and the right for steering.

__Additional Note:__ There are different scalers used for if the joystick is pressed or not, currently they are the same, but you can change that in `controller_teleop.py` to be different numbers.

# Debugging

### The controls not changing gui 
Spamming buttons occassionally works. Otherwise check that joy_node is running properly by ensuring /joy is publishing with a frequency on the right pane, or double check with `ros topic echo /joy`