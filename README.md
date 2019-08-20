# bipropellant-hoverboard-api

C++ Wrapper around the [bipropellant hoverboard UART protocol](https://github.com/bipropellant/bipropellant-protocol).
An Arduino compatible branch is atumatically generated: master_arduino. (This branch removes the submodule dependency.)

Feel free to write Pull Requests for added features and examples.

For now, only functions are implemented which were needed by the Author. Many more are possible.

See https://github.com/bipropellant/bipropellant-protocol/blob/fe96935cb2f550110ffaaa6d74852bedd79e25c7/protocol.c#L266-L307 for possible information which could be accessed via this protocol.

## Implemented Features
* Setting Motor PWM
* Reading Motor Speeds
* Reading electrical Parameters (Voltage, Current..)
* Subscribe to automatic updates of values
* Schedule recurring transmission of values
* Register callback function when values are received

See https://github.com/bipropellant/bipropellant-hoverboard-api/blob/master/src/HoverboardAPI.cpp for detailed information of implemented functions.

## Additions by Alex Makarov
More ROS-friendly setup:
* Adapted for using on 64-bit ARM platform
* Removed Arduino dependencies
* Added methods to set speed and PID control
