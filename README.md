# Esperto Wiimote

Based on the [esperto](https://github.com/KayJay7/esperto) input system, `esperto-wiimote` is probably the
most advanced Wiimote mapping available. It features precise accelerometer calibration,
advanced IR tracking, and full remapping options, with full "key combos" support. And all of that
at virtually no additional latency, mostly only limited by speed of the kernel's evdev/uinput api itself.

The IR tracking algorithm is a rewrite [this](https://gist.github.com/marcan/c7ca900d5191610957c478bbdbb516c0)
algorithm from Hector Martin, included for reference [in this repository](https://github.com/KayJay7/esperto-wiimote/tree/master/reference).

This software requires a custom Wiimote kernel driver found [here](https://github.com/dkosmari/hid-wiimote-plus).

Available on [crates.io](https://crates.io/)!

## Configuration options

The configuration file is specified with the `--config <FILE>` command line argument. If unspecified,
the program will follow a media-oriented-remote default available at [`src/default.yaml`](https://github.com/KayJay7/esperto-wiimote/blob/master/src/default.yaml) in this repo.

Here an example of configuration with descriptions of the available options.

```yaml
# Whether to grab the devices, so other software won't try to use them 
grab: false
# Command to run when the first device is connected, as list of arguments
# Useful for turning on the sensor bar
on_connect: [ "echo", "hello", "world" ]
# Command to run when all devices are disconnected, as list of arguments
# Useful for turning off the sensor bar
on_disconnect: [ "echo", "goodbye", "world" ]
# Options for the output device slot. There's four slots from `slot1` to `slot2` for each Wiimote
# slots are useful to separate mouse inputs from keyboard inputs from joysticks
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
# Center an axis value when it appears (resets when it disappears)
centering:
  # Axes can be specified by name...
  !Axis ABS_X: 2048
  # Or by their real integer id
  !CustomAxis 1: 2048
# Park an axis when it disappears
parking:
  !CustomAxis 0: 4095
  !Axis ABS_Y: 4095
# The size of the sensorbar can be changed if using a custom one
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
        # Keys can also be specified by names...
        code: !Key KEY_SPACE
      modified:
        - modifier: pointer
          action:
            slot: Slot1
            # Or by their real integer id
            code: !CustomKey 0x110
```

The full list of available Wiimote inputs is: `A, B, Up, Down, Left, Right, Plus, Minus, Home, Btn1, Btn2` for keys,
and `IRAbsX, IRAbsY` for axes. The full list of available output codes is found [here](https://github.com/torvalds/linux/blob/master/include/uapi/linux/input-event-codes.h),
even if a scancode name is not recognised, the numeric id will still work.

## Calibration

An accelerometer calibration wizard can be started by running `esperto-wiimote calibration`, it will require
to place the Wiimote on its 6 sides and let it still for a while. After that it will compute some data
and write it to the config file (if provided as argument) or print it on screen to be copied on the config.
Some tuning options can be provided as arguments if necessary.

It is highly encouraged to calibrate the sensor before using the software, as the accelerometer will be entirely
unreliable otherwise. It is suggested to calibrate after the remote has reached its normal operating temperature.
Remember that the sensor readings will drift slightly overtime and with temperature changes, but
recalibration won't likely be necessary, as the software accounts for drift.

## Roadmap

- [x] Full pointer and buttons support
- [x] Accelerometer calibration
- [x] Full combo support
- [x] Multi-device support
- [ ] KeyCombos as output events
- [ ] Wiimote extensions (nunchuck, etc.)