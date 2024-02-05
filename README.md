# Imenco Pan/Tilt ROS2 Driver
![alt text](docs/media/logo.png)

## About

This driver provides a means of controlling an Imenco Pan and Tilt device using standard ROS2 messages.  

@note This package currently only implements UDP communications through a UDP-serial converter like a MOXA.   Direct serial communication is not yet implemented.  

## Installation

This package is currently only available through source build.

Change to your workspace directory
```bash
cd <your_workspace>
```

Clone the repo
```bash
git clone https://github.com/SeawardScience/imenco_pt.git src
```

get dependencies and build
```bash
rosdep install --from-paths src -y --ignore-src
colcon build
```

## Getting Started

## Topics
This node only subscribes to one topic `/joy_console/joy` for joystick input.   The topic can be changed by using the `joy_topic` parameter

## Parameters

```yaml
/imenco_pt:
  ros__parameters:
    dst_ip: 10.0.0.31                 # the IP address of the pan and tilt module (for communicaton over a MOXA)
    port: 4016                        # the port to send messages to (if using a MOXA)
    from_addr: 1                      # the value to fill the "from" address field in the packet header
    to_addr: 3                        # the value to fill the "to" address field in the packet header
    joy_topic: joy_console/joy        # the joystick topic you want to control the P/T with
    max_joy_age: 0.5                  # how old can a joystick messsage be before we center it
    pan_axis: 1                       # the joystick axis you want for panning
    pan_gain: -0.1                    # a multiplier for the pan axis  negative value to invert axis
    tilt_axis: 2                      # the joystick axis you want for tilting
    tilt_gain: 0.1                    # a multiplier for the tilt axis  negative value to invert axiss
```


## Credits

This project was developed by Kristopher Krasnosky of [Seaward Science](http://seaward.science) with funding from [Pelagic Research Service](https://pelagic-services.com) 
