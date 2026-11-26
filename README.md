# BNO085 IMU on ESP32 → ROS 2 (Humble)

An ESP32 reads a BNO085 IMU and streams it over USB serial. A ROS 2 driver package turns that stream into standard IMU topics.

```
BNO085 --I2C--> ESP32 --USB serial (921600)--> PC: bno085_driver --> /imu/data, /imu/rpy, /imu/mag
```

> **Status:** tested with the **I2C** interface only. **SPI** support is planned for a later phase and is not implemented yet.

## Layout

| Path | Purpose |
|------|---------|
| `esp32_bno085_firmware/esp32_bno085_firmware.ino` | Arduino firmware for the ESP32 |
| `src/bno085_driver/` | ROS 2 (`ament_python`) driver package |

## Hardware (I2C)

| BNO085 | ESP32 |
|--------|-------|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

```
        ESP32 dev board                      BNO085 breakout
      +-----------------+                  +-----------------+
      |             3V3 |------------------| VIN             |
      |             GND |------------------| GND             |
      |  GPIO21 (SDA)   |------------------| SDA             |
      |  GPIO22 (SCL)   |------------------| SCL             |
      |                 |                  | INT   (not used)|
      |      USB        |                  | RST   (optional)|
      +--------+--------+                  | PS0/PS1 -> GND  |
               |                           +-----------------+
               | USB cable
               v
              PC  (/dev/ttyACM0)
```

Use only 3.3 V for the sensor. Keep the wires short, because the bus runs at 400 kHz. If your breakout has no I2C pull-ups, add 4.7 kΩ from SDA and from SCL to 3V3.

- Default I2C address is `0x4A` (`0x4B` if the address pin is pulled high).
- INT and RST are not needed. If you wire RST, set `RESET_PIN` in the firmware.
- On breakouts with PS0/PS1 pins, both must be low for I2C mode.

## 1. Flash the ESP32

1. Install the **Adafruit BNO08x** library (Arduino Library Manager).
2. Select the board **ESP32 Dev Module** and the serial port.
3. Open `esp32_bno085_firmware/esp32_bno085_firmware.ino` and upload.

Notes:
- The sensor starts at 100 kHz, then the bus switches to 400 kHz (`I2C_RUN_HZ`). At 100 kHz the board only reached about 38 Hz. At 400 kHz it measured 100 Hz. If you see garbled data, set `I2C_RUN_HZ` to `100000`.
- Close the Arduino serial monitor before running ROS, because it holds the port.
- If the sensor is not found on startup, the firmware prints the I2C devices it sees. Unplug the USB for about 10 s to power-cycle the sensor if it is stuck.

Serial protocol, ASCII, one line per sample at 100 Hz:

```
$IMU,<ms>,<qx>,<qy>,<qz>,<qw>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>,<mx>,<my>,<mz>*<XOR checksum, hex>
```

Quaternion is unitless, gyro is in rad/s, accel is in m/s² (includes gravity), magnetic field is in µT. The driver also accepts lines without the three magnetometer fields. Lines starting with `#` are status messages from the firmware.

## 2. Build and run the ROS 2 driver

```bash
source /opt/ros/humble/setup.bash
cd bno08x_ros2_driver   # repository root (a colcon workspace)
colcon build --packages-select bno085_driver
source install/setup.bash
ros2 launch bno085_driver bno085.launch.py
```

You may need serial permissions: `sudo usermod -aG dialout $USER`, then log out and back in.

## Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/imu/data` | `sensor_msgs/Imu` | orientation, angular velocity, linear acceleration, with covariances; 100 Hz |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | x = roll, y = pitch, z = yaw (ZYX), radians by default; 100 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | calibrated magnetic field in Tesla |

The launch file also publishes a static transform `base_link` -> `imu_link` (x = 1.0 m, z = 1.1 m, no rotation). Edit `launch/bno085.launch.py` to match your mounting.

Message timestamps are taken on the PC when each line arrives, not from the ESP32 clock.

## Parameters

Set in `src/bno085_driver/config/bno085.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyACM0` | Serial device |
| `baud` | `921600` | Must match the firmware |
| `frame_id` | `imu_link` | Frame in message headers |
| `topic` | `imu/data` | IMU topic |
| `rpy_topic` | `imu/rpy` | Roll/pitch/yaw topic |
| `rpy_degrees` | `false` | Publish RPY in degrees |
| `mag_topic` | `imu/mag` | Magnetic field topic |
| `mag_scale` | `1e-6` | Converts firmware µT to Tesla |
| `orientation_stddev` | `0.01` | rad, used for covariance |
| `angular_velocity_stddev` | `0.005` | rad/s |
| `linear_acceleration_stddev` | `0.05` | m/s² |

The node drops lines with a bad checksum and reconnects automatically if the serial port disappears.

## Verify

```bash
ros2 topic hz /imu/data
ros2 topic hz /imu/rpy
ros2 topic echo /imu/rpy
```

## License

Gahan AI Non-Commercial Software License, see [LICENSE.md](./LICENSE.md).

## Roadmap

- SPI interface support for the BNO085 (not yet implemented or tested).
