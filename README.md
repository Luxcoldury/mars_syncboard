# ROS driver for SyncBoard

## How to setup

0. Get and build the service definition package [mars_syncboard_srvs](https://github.com/Luxcoldury/mars_syncboard_srvs)

1. Clone this repo to `catkin_ws/src`

2. Install the modified version of pigpio.

   ```bash
   cd mars_syncboard/vendors/pigpio
   sudo make install
   ```

3. `cd` to `catkin_ws` then `catkin_make`

## How to run the node

*To manipulate GPIOs of raspberry pi, root privileges are required.*

0. `roscore`
1. `sudo su`
2. `cd` to `catkin_ws`
3. `source devel/setup.bash`
4. `rosrun mars_syncboard syncboard`

Or if you prefer to use `roslaunch`.

1. `sudo su`
2. `cd` to `catkin_ws`
3. `source devel/setup.bash`
4. `roslaunch mars_syncboard example.launch`

Then you can play with Syncboard by either calling the services or setting params.

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

### Service `config_gps`

To configure the signal output on Line GPS.

Request and response of `config_gps` in *Message Description Language* :

```
uint32 baud
uint32 offset_us
bool inverted
---
bool succeeded
string msg
```

* `uint32 baud` : The baud rate of the GPS time signal. Should be (9600, 14400, 19200, 38400, 56000, 57600, 115200). The default baud rate is 9600.
* `uint32 offset_us` :  Offset/delay of the GPRMC message compared to the PPS rising edge, in microsecond ($10^{-6}$ second). The default offset is 100ms (100*1000 microseconds).
* `bool inverted` : If it is `true`, the level of the signal will be inverted. The default setting is `false`.

### Service `toggle_button_led`

To control the LED on the external button.

Request and response of `toggle_button_led` in *Message Description Language* :

```
uint8 mode
---
uint8 mode
```

* `uint8 mode` : `0` for off, `1` for always on, `2` for blinking every 1 second

## Params advertised

ROS Params can be helpful if you want to monitor the triggering status of each line or to configure multiple lines all at once (e.g. in a launch file). However, it is NOT recommended to configure lines on the fly using `rosparam set` since it gives you no error message if your new combination of params doesn't make sense.

Syncboard reads the params once per second and will try to configure the lines according to the params. No matter the configuration succeeded or not, Syncboard will then update the params to reflect the actually configuration of the line. Which means (1) if you want to alter more than one param, better set them at once using `rosparam load`, (2) if the new configuration doesn't make sense, old params will be restored, (3) even if the new configuration does make sense and be applied, the updated params (and the actual triggering setting) may not be identical to your inputs due to rounding.

### Param `line/<line_num>`

There are 5 ROS param for each line: `line/<line_num>/enabled`, `line/<line_num>/trigger_type`, `line/<line_num>/freq`, `line/<line_num>/offset_us`, `line/<line_num>/duty_cycle_percent`.  The definition is identical to the parameters of service `config_line`.

`<line_num>` resides in 1,2,8~17.

The params will not be created (and thus appear in `rosparam list`) until the line is explicitly configured either by setting the params or using `config_line` service.

### Param `gps`

There are 3 ROS param for GPRMC: `gps/baud`, `gps/inverted`, `gps/offset_us`.  The definition is identical to the parameters of service `config_gps`.

### Param `triggering`

The definition is identical to the parameter of service `toggle_trigger`.

### Param `led`

The definition is identical to the parameter of service `toggle_button_led`.

## Topics advertised

### Topic `line/<line_num>`

The exact `time` which Line `<line_num>` triggered (or is going to trigger) at is published through this topic.

The messages are published once every 1 second, not in real-time.

`<line_num>` resides in 1,2,8~17.

### Topic `button`

`String` Message `Pressed` / `Released` will be published when the button is pressed / released.