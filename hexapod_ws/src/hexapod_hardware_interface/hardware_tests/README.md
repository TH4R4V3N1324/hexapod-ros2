# Hardware tests

Small programs to check each piece of the robot's hardware on its own, without ROS
running. Each one prints what it finds, then a list of `PASS` / `WARN` / `FAIL`
checks with hints on what to look at, and exits non-zero if anything failed.

| Test | Hardware | Runs in |
|---|---|---|
| `test_servos` | ST3215 bus servos | container |
| `test_power` | INA226 power monitors (left 0x40, right 0x41) | container |
| `test_imu` | ISM330DHCX IMU (0x6A) | container |
| `test_lidar` | RPLIDAR A2M12 | container (or the Pi, with `python3-serial`) |
| `test_camera.sh` | Raspberry Pi Camera Module 3 | the Pi |
| `test_tof.py` | Arducam ToF camera (B0410) | the Pi |

Every test takes `--help`.

## Running them

The servo, power, IMU and lidar tests are built into the robot's image. Open a
shell in the running container on the Pi:

```bash
ssh raven@hexapod
docker exec -it hexapod_ros2 bash
ros2 run hexapod_hardware_interface test_servos scan
```

Only one program can use the servo bus or the lidar at a time. The container
runs `hexapod_fake.launch.py` (mock hardware) by default, so they're free. If
it runs `hexapod_robot.launch.py`, stop it and use a one-off container instead:

```bash
cd ~/hexapod-ros2/hexapod_ws
docker compose stop
docker compose run --rm hexapod bash
```

The I2C tests can run alongside the robot, though `test_imu` resets the IMU,
so the IMU node logs a read failure and reinitialises.

The camera tests run on the Pi itself, outside the container, because the
Pi 5's cameras need Raspberry Pi OS's own libcamera:

```bash
~/hexapod-ros2/hexapod_ws/src/hexapod_hardware_interface/hardware_tests/test_camera.sh
```

## Suggested order for a first bring-up

1. **Power:** `test_power`. Both monitors found, battery voltage sensible,
   current positive (if it reads negative, the shunt is wired backwards).
2. **Servo IDs:** new servos all come as ID 1. Connect them **one at a time**,
   then `test_servos scan --all` to find it and `test_servos set-id 1 <leg*10+joint>`
   to give it its ID (e.g. 23 for leg 2's tibia).
3. **Servos:** `test_servos scan` should list all 18 with sensible voltage and
   temperature.
4. **Directions:** `test_servos wiggle <id>` on each servo. It moves
   +10°, back, -10°, back, so you can see which joint it drives and which way
   positive is. Joints that turn the wrong way for the URDF need `direction` -1
   in `hexapod.ros2_control.xacro`.
5. **Centres:** every joint at 0 is the leg laid out flat: coxa straight out,
   femur level, and the tibia in line with the femur. The tibia is curved, so
   line up the foot ball's centre with the femur, not the tibia's body.
   `test_servos centre <id>` turns torque off; put the joint in that pose and
   press Enter, and the servo stores it as its centre. The `offset` in
   `hexapod.ros2_control.xacro` can then stay 0. `test_servos offset <id> <steps>`
   fine-tunes a centre afterwards.
6. **Limits (optional):** after centring, `test_servos limits <id> min` and
   `limits <id> max` record each end by hand (or give degrees), so the servo
   itself refuses to drive the joint into the body or its stops.
7. **IMU:** `test_imu` with the robot still and level checks gravity reads on
   +Z, as `imu_link` expects. While it streams, raise the front (X goes
   positive), raise the left (Y goes positive) and turn it anticlockwise
   (gyro Z goes positive).
8. **Lidar:** `test_lidar` reads the lidar's info and health, scans for 5 s and
   prints the nearest object per 45° sector. Hold a hand close to it to see
   which sector it lands in.
9. **Cameras:** `test_camera.sh` and `test_tof.py`, below.

## Servos

```
test_servos ports                         list the serial ports
test_servos scan [--all] [--all-bauds]    ping the 18 leg servos (or every ID, at every baud rate)
test_servos ping <id>                     check one servo replies
test_servos info <id>                     every register, decoded
test_servos regs                          the register names read and write take
test_servos read <id> <reg|addr> [len]    read a register by name, or raw bytes
test_servos write <id> <reg|addr> <value> write a register by name (EEPROM unlocked for you), or raw bytes

test_servos set-id <old> <new>            change a servo's ID
test_servos set-baud <id> <rate> --yes    change a servo's baud rate
test_servos mode <id> <0-3>               position (0, what the robot needs), wheel, PWM, step (1-3 need --yes)
test_servos reset <id> --yes              factory reset (ID 1, no offset or limits)

test_servos centre <id>                   by hand: the pose it's in when you press Enter reads 0
test_servos offset <id> [steps]           show or set the stored position offset
test_servos limits <id>                   show the angle limits
test_servos limits <id> min|max [degrees] set one limit, typed or by hand
test_servos limits <id> <min> <max>       set both limits
test_servos limits <id> clear             remove the limits

test_servos torque <id> on|off            hold where it is, or go limp
test_servos limp [id ...]                 torque off (default: every servo). The legs will drop!
test_servos pos [id ...]                  where each servo is
test_servos move <id> <degrees>           move to an angle from centre (--steps for 0-4095) and hold
test_servos wiggle <id> [degrees]         +deg, back, -deg, back, then hold
test_servos sweep <id> [cycles] [min max] back and forth, then the largest error, move time, load, current
test_servos wheel <id> <°/s> [s] --yes    spin in wheel mode (only for a servo not fitted to a leg)
test_servos monitor [id ...]              live position, load, current, voltage, temperature (--rate <Hz>)
test_servos selftest <id>                 replies, 50 reads, voltage, temperature, errors, ±10° move
```

Options: `--port /dev/ttyAMA0`, `--baud 1000000`, `--echo` (if the bus adapter
loops TX back to RX), `--speed <°/s>` for `move`, `wiggle`, `sweep` and
`selftest` (default 30), `--debug` to print every packet sent and byte received.
Angles are degrees from the servo's centre (2048 steps).

`centre`, `offset`, `limits`, `set-id`, `set-baud`, `mode` and `reset` write the
servo's EEPROM, so they're kept through a power cycle. Centre a servo before
setting its limits: the limits count from the centre, so centring moves them.

Everything that moves leaves torque on, so the joint holds its position. Ctrl+C
stops the sequence but the servo still holds; run `limp` to release it. Support
the body before running `limp` on a standing robot.

The status column decodes the servo's error bits: voltage, sensor, overheat,
overcurrent, angle, overload.

## Cameras

**Camera Module 3:** `test_camera.sh` finds the camera, takes an autofocused
photo (`~/camera_test.jpg`, copy it off with `scp raven@hexapod:camera_test.jpg .`)
and times 5 s of 720p video. It needs `rpicam-apps` (`sudo apt install rpicam-apps-lite`).

**Arducam ToF:** install Arducam's SDK and camera overlay first, following
[Arducam_tof_camera](https://github.com/ArduCAM/Arducam_tof_camera)
(`Install_dependencies.sh`, then `pip install ArducamDepthCamera`), and reboot.
Then:

```bash
test_tof.py --seconds 10 --save depth.pgm
```

It streams the median distance at the centre of the image (point it at a wall
a measured distance away), the share of pixels with a confident depth and the
frame rate. `--index` picks the camera if both are on the overlay, `--range
2000` selects near mode.

`test_tof.py` was written against the SDK's current Python API
(`ArducamCamera`, `Connection.CSI`, `FrameType.DEPTH`, `Control.RANGE`) and
hasn't been run on the real camera. Older SDK releases named these
differently.
