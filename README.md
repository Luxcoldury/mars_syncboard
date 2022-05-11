# ROS driver for SyncBoard

## How to setup

1. Install [pigpio](http://abyz.me.uk/rpi/pigpio/download.html)
2. Clone this repo to `catkin_ws/src`
3. `catkin_make`

## How to Run the node

*To manipulate GPIOs of raspberry pi, root privileges are required.*

0. `roscore`
1. `sudo su`
2. `cd` to `catkin_ws`
3. `source devel/setup.bash`
4. `rosrun mars_syncboard syncboard`

## Services advertised

### Service `toggle_trigger`

To start or stop triggering on Line 1,2,8~17.

Request and response of `toggle_trigger` in *Message Description Language* :

```
bool start_trigger
---
bool triggering
```

* `bool start_trigger` : `true` to start, `false` to stop

In normal use cases, lines are configured using `config_line` before calling `toggle_trigger` to start and finally stop triggering.

Line PPS and Line GPS is always on.

### Service `config_line`

To configure the signal output on Line 1,2,8~17.

Request and response of `config_line` in *Message Description Language* :

```
uint8 line_num
bool enabled
uint8 trigger_type
float32 freq
uint32 offset_us
uint8 duty_cycle_percent
---
bool succeeded
string msg
```

* `uint8 line_num` : The index of the line to be configured.
* `bool enabled` : If it is `true`, the line will start triggering after `toggle_trigger true`

* `uint8 trigger_type` : `0` for Rising edge, `1` for Falling edge, `2` for Either edge
* `float32 freq` : Frequency in Hz. Should be in (0,1000]. When the frequency is lesser than 1Hz, SyncBoard will try to trigger the line every $\lfloor 1/freq \rfloor$ seconds.
* `uint32 offset_us` : Offset/delay of the signal, in microsecond ($10^{-6}$ second)

* `uint8 duty_cycle_percent` : Duty cycle of the signal, in percentage. If `freq` is less than 1Hz, the width of the signal is `duty_cycle_percent`% of 1 second.

### Service `toggle_button_led`

To control the LED on the external button.

Request and response of `toggle_button_led` in *Message Description Language* :

```
uint8 mode
---
uint8 mode
```

* `uint8 mode` : `0` for off, `1` for always on, `2` for blinking every 1 second

## Topics advertised

### Topic `line/x` (in which `x` is the line index)

The exact `time` which Line `x` triggered (or is going to trigger) at is published through this topic.

The messages are published every 1 second, not in real-time.

`x` resides in 1,2,8~17.

### Topic `button`

`String` Message `Pressed` / `Released` will be published when the button is pressed / released.