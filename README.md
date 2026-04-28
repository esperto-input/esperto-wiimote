# Esperto Wiimote

> [!IMPORTANT]
> This software requires an updated/fixed Wiimote kernel driver found [here](https://github.com/dkosmari/hid-wiimote-plus).
> 
> After installation, run `sudo sed -i '/ATTR{inhibited}="1"/s/^/#/' /etc/udev/rules.d/99-wiimote.rules` to un-inhibit IR and accelerometer events.

Based on the [esperto](https://github.com/KayJay7/esperto) input system, `esperto-wiimote` is an advanced Wiimote mapper. It features precise accelerometer calibration,
good IR tracking, and powerful remapping options, with combos support. It adds virtually no latency, mostly dominated by the kernel's evdev/uinput api itself.

This program features a port++ of Hector Martin's 2-point IR tracking [algorithm](https://gist.github.com/marcan/c7ca900d5191610957c478bbdbb516c0), [bundled](https://github.com/KayJay7/esperto-wiimote/tree/master/reference) for reference.

Available on [crates.io](https://crates.io/crates/esperto-wiimote).

## Configuration options

The configuration file is specified with `--config <FILE>`. If unspecified,
the program will load a built-in media-remote default available at [`src/default.yaml`](https://github.com/KayJay7/esperto-wiimote/blob/master/src/default.yaml).

Here's a cnfiguration example with descriptions:

```yaml
# Whether to grab the devices, so other software won't use them 
grab: false
# Command to run when the first Wiimote connects, as list of arguments
# Useful for automation like powering on the sensor bar
on_connect: [ "echo", "hello", "world" ]
# Command to run when thelast Wiimote disconnected, as list of arguments
# Useful for automation like powering off the sensor bar
on_disconnect: [ "echo", "goodbye", "world" ]
# Options for the output device slot. For each Wiimote there are four available slots, from `slot1` to `slot4`.
# Slots are useful to isolate mouse/keyboard/joysticks inputs
slots:
  slot1:
    # Name of the wirtual device
    name: "Esperto Wiimote mouse"
  slot2:
    name: "Esperto Wiimote keyboard"
    # Whether the device should autorepeat held keys, this might get overruled by the compositor
    repeating: true
# Accelerometer calibration data generated with the `esperto-wiimote calibration` command
accelerometer_calibration: [
  1.0014056, -0.011690015, 0.009826779,
  -0.02909683, 1.013545, 0.005751312,
  -0.010004735, -0.012316463, 0.99458194,
  28.481918, 33.827446, 30.468597 ]
# Margins of the mapped area. True margins are 0-4095 (regardless of screen resolution), but you will 
# want to exceed those to account for the aspect ratio, and the IR sensor not reaching all the way to the margins.
# The same scaling will be applied to all axes
screen_limits:
  north: -2000
  south: 6569
  west: -300
  east: 4395
# Center an axis when it gets enabled 
centering:
  # Axes can be specified by name...
  !Axis ABS_X: 2048
  # Or by their real integer id
  !CustomAxis 1: 2048
# Park an axis when it gets disabled
parking:
  !CustomAxis 0: 4095
  !Axis ABS_Y: 4095
# The physical dimensions of your sensorbar
sensor_bar:
  width: 19.5
  cluster_width: 4.5
  cluster_height: 1.0
  pixel_width: 256.0
# The smoothing parameters can be tuned
smoothing:
  radius: 30.0
  speed: 0.15
  deadzone: 8.0
# esperto combos configuration, check https://github.com/KayJay7/esperto for more information
esperto:
  modifiers:
    - name: pointer
      keys: [ B ]
  actions:
    - key: A
      action:
        # Actions must specify their output slot
        slot: Slot2
        # Keys can be specified by name...
        code: !Key KEY_SPACE
      modified:
        - modifier: pointer
          action:
            slot: Slot1
            # Or by their integer id
            code: !CustomKey 0x110
```

The full list of available Wiimote keys is:

* `A`
* `B`
* `Up`
* `Down`
* `Left`
* `Right`
* `Plus`
* `Minus`
* `Home`
* `Btn1`
* `Btn2`

And axis:

* `IRAbsX`
* `IRAbsY`

The full list of available output codes is found [here](https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h). In case a scancode name is not recognised, the numeric id will still work.

## Calibration

An accelerometer calibration wizard can be started by running `esperto-wiimote calibration`.

You will need to place the Wiimote on its 6 sides as prompted, and wait for the tool to gather enough readings. The output will be added to the config file (if provided), otherwise printed on screen.
Some advanced overrides can be provided as arguments if necessary.

We highly encourage to calibrate your Wiimote before using this program, otherwise the accelerometer might be quite unreliable. It is better to calibrate after the remote has reached its normal operating temperature.
Remember that the sensor readings will drift slightly over time and with temperature changes, but recalibration likely won't be necessary, as the software accounts for drift.

## Roadmap

- [x] Full pointer and buttons support
- [x] Accelerometer calibration
- [x] Full combo support
- [x] Multi-device support
- [ ] KeyCombos as output events
- [ ] Wiimote extensions (nunchuck, etc.)
