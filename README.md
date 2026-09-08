# Kuartis Embedded System Pipeline Homework

This repository contains an STM32-to-ROS2 sensor pipeline developed for the Kuartis Hardware and Embedded Systems Group Embedded System Pipeline Homework.

The system acquires calibrated magnetic-field data from a BNO085 over SPI, computes magnetic heading, filters the magnetic X/Y components with a scalar Kalman filter, formats the heading as an NMEA 0183 HDM sentence, and transmits it to a PC over UART. A ROS2 LifecycleNode receives and validates the NMEA stream, publishes yaw orientation as `sensor_msgs/msg/Imu`, broadcasts the `base_link -> imu_link` transform, and reports connection/frequency information on `/diagnostics`.

## System Architecture

```text
BNO085
  |
  | SPI2 + INT
  v
STM32F767ZI
  |
  |-- BNO085 SHTP processing
  |-- Calibrated magnetic field X/Y/Z
  |-- Scalar Kalman filters on X and Y
  |-- Heading calculation: atan2(Y, X)
  |-- Sensor state machine and recovery
  |
  v
FreeRTOS SensorTask
  |
  | Message Queue
  v
FreeRTOS CommunicationTask
  |
  | NMEA 0183 HDM formatting
  | UART TX ring buffer
  | Interrupt-driven UART transmission
  v
USART3 / USB Virtual COM Port
  |
  | $HCHDM,x.xx,M*hh\r\n
  v
ROS2 BNO085 Lifecycle Driver
  |
  |-- NMEA parsing and checksum validation
  |-- sensor_msgs/msg/Imu
  |-- base_link -> imu_link TF
  |-- /diagnostics
```

## Hardware

- Development board: NUCLEO-F767ZI (`STM32F767ZITx`)
- Sensor: BNO085 9-DoF IMU, using the calibrated magnetic-field report
- Sensor interface: SPI2, Mode 3
- PC communication: USART3, 115200 baud, 8-N-1
- RTOS: FreeRTOS with CMSIS-RTOS2 API

### BNO085 Connections

| BNO085 Signal | STM32F767ZI Pin | Function |
|---|---|---|
| SCK | PB13 | SPI2 SCK |
| MISO | PC2 | SPI2 MISO |
| MOSI | PB15 | SPI2 MOSI |
| CS | PB12 | Chip Select |
| INT | PC8 | Falling-edge EXTI |
| RST | PC9 | Sensor Reset |
| P0 / WAKE | PA8 | Wake / interface control |
| VIN | 3.3 V | Power |
| GND | GND | Ground |

## Repository Structure

```text
Kuartis_Homework/
├── embedded/
│   └── 000_Kuartis_HW_BNO085/
│       ├── Inc/
│       ├── Src/
│       ├── Drivers/
│       ├── Middlewares/
│       └── 000_Kuartis_HW_BNO085.ioc
├── ros2_ws/
│   └── src/
│       └── bno085_driver/
│           ├── include/bno085_driver/
│           ├── src/
│           ├── CMakeLists.txt
│           └── package.xml
├── docs/
└── README.md
```

## Embedded Firmware

### Sensor Acquisition

The BNO085 is connected through SPI and uses its interrupt output to notify the STM32 that data is available. The GPIO interrupt callback only signals the driver; SHTP packet processing is performed in the FreeRTOS sensor task instead of inside the ISR.

The firmware uses the BNO085 **Calibrated Magnetic Field** report. The reported X, Y, and Z values are converted to microtesla.

### Sensor State Machine

Sensor operation is controlled by a switch-based finite-state machine:

```text
STARTUP
   |
   v
WAIT_PRODUCT_ID
   |
   v
CALIBRATING
   |
   v
RUNNING
   |
   | communication failure / data timeout
   v
RECOVERY
   |
   +---------------------> STARTUP
```

The firmware waits for the BNO085 startup/initialization response and Product ID response before enabling normal measurement operation.

During calibration, calibrated magnetic-field data is requested at 50 Hz. After magnetometer accuracy reaches the highest reported level, the application switches the sensor report interval to the 10 Hz runtime rate.

### Heading Calculation

Magnetic heading is computed from the calibrated horizontal magnetic-field components:

```text
heading = atan2(Y, X)
```

The result is normalized to the `[0, 360)` degree range.

### Kalman Filtering

Two independent scalar Kalman filters are used:

```text
Magnetic X -> Kalman X ----\
                            +--> atan2(Y, X) --> filtered heading
Magnetic Y -> Kalman Y ----/
```

Current tuning:

```text
Process noise Q       = 0.25 uT^2/s
Measurement noise Rx = 0.433 uT^2
Measurement noise Ry = 0.393 uT^2
Initial covariance P = 1.0 uT^2
```

The measurement-noise variances were obtained from stationary sensor characterization. The final `Q` value was selected by comparing stationary noise reduction against dynamic heading response.

### RTOS Architecture

Two FreeRTOS tasks separate sensor processing from communication:

- `SensorTask`: BNO085 processing, calibration, Kalman filtering, heading generation, and sensor recovery.
- `CommunicationTask`: NMEA formatting and UART transmission.

A FreeRTOS message queue transfers valid heading values from `SensorTask` to `CommunicationTask`. This prevents the sensor-processing path from being coupled directly to serial transmission.

FreeRTOS stack-overflow and allocation-failure hooks are enabled and route unrecoverable RTOS failures to the system error handler.

### NMEA 0183 Output

The communication task generates an HDM sentence:

```text
$HCHDM,123.45,M*hh\r\n
```

The checksum is the XOR of all characters between `$` and `*`.

The implementation uses the **NMEA 0183 sentence format** for the application protocol. The physical PC link is the STM32 UART/USB Virtual COM Port rather than a marine NMEA electrical interface.

### UART Transmission

UART transmission is interrupt-driven. A software TX ring buffer decouples the communication task from the physical UART transfer.

The ring-buffer write operation is all-or-nothing for a sentence: if there is not enough free space, the frame is not partially enqueued.

### Embedded Error Handling

The firmware tracks sensor and communication faults explicitly.

Sensor-side errors include:

```text
SENSOR_ERROR_INIT
SENSOR_ERROR_PRODUCT_ID_TIMEOUT
SENSOR_ERROR_COMMUNICATION
SENSOR_ERROR_CONFIGURATION
SENSOR_ERROR_DATA_TIMEOUT
```

Communication-side errors include:

```text
COMM_ERROR_QUEUE_FULL
COMM_ERROR_NMEA_FORMAT
COMM_ERROR_UART_BUSY
COMM_ERROR_UART
```

Defined behavior:

- Sensor startup or communication failures transition the sensor state machine to recovery.
- Recovery periodically retries sensor initialization instead of permanently blocking the application.
- A runtime sensor-data timeout stops valid heading production and initiates recovery.
- Invalid heading values are rejected by the NMEA formatter.
- Queue, NMEA, UART-busy, and UART-error conditions are counted in the system status.
- Only successfully calculated and formatted heading data is transmitted.

## Building and Running the Embedded Firmware

### Requirements

- STM32CubeIDE
- ST-LINK
- NUCLEO-F767ZI
- BNO085 connected as shown above

### Build

Open/import the following STM32CubeIDE project:

```text
embedded/000_Kuartis_HW_BNO085
```

Select the Debug configuration and build the project.

The expected firmware image is generated under:

```text
embedded/000_Kuartis_HW_BNO085/Debug/
```

Flash the target through ST-LINK and run the application.

### Verify the UART Output

On Linux, identify the Virtual COM Port:

```bash
ls -l /dev/ttyACM*
```

The expected port is normally `/dev/ttyACM0`.

Monitor the NMEA stream with:

```bash
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

Expected output:

```text
$HCHDM,123.45,M*hh
$HCHDM,123.51,M*hh
...
```

The runtime output rate is approximately 10 Hz after sensor calibration is complete.

## ROS2 Driver

### Environment

The ROS2 implementation was developed and tested with:

```text
Ubuntu 22.04
ROS2 Humble
C++17
```

The package is:

```text
ros2_ws/src/bno085_driver
```

### Dependencies

Source ROS2 Humble and install package dependencies through `rosdep`:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

The driver uses ROS2 lifecycle support, `sensor_msgs`, TF2, and ROS diagnostics.

### Serial-Port Permission

Check the port:

```bash
ls -l /dev/ttyACM*
```

If the current user is not a member of `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in after changing the group membership.

### Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bno085_driver
source install/setup.bash
```

### Run

Start the node in one terminal:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run bno085_driver bno085_driver_node
```

The node starts in the `unconfigured` lifecycle state.

In a second terminal:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 lifecycle set /bno085_driver configure
ros2 lifecycle set /bno085_driver activate
```

Check the lifecycle state:

```bash
ros2 lifecycle get /bno085_driver
```

Expected state:

```text
active [3]
```

## ROS2 Lifecycle Behavior

| Transition | Behavior |
|---|---|
| Configure | Allocate driver resources and open `/dev/ttyACM0` at 115200 baud |
| Activate | Start serial processing and activate IMU publishing |
| Deactivate | Stop streaming/publishing while keeping the serial port open |
| Cleanup | Close the serial port and release allocated resources |
| Shutdown | Release resources during node shutdown |

When the node is reactivated, stale serial input is flushed before normal frame processing resumes.

## ROS2 Data Flow

The ROS2 node reads complete NMEA HDM sentences from the STM32, validates their checksum, extracts heading in degrees, and converts that heading to yaw orientation.

### IMU Topic

Topic:

```text
/imu/data
```

Type:

```text
sensor_msgs/msg/Imu
```

The message uses:

```text
frame_id: imu_link
```

The heading is represented as a yaw-only quaternion. The message timestamp is generated by the ROS2 node when a valid NMEA measurement is received/processed because the HDM sentence itself does not contain a timestamp.

Angular velocity and linear acceleration are not provided by this implementation and their covariance fields are marked accordingly.

### TF

The node broadcasts:

```text
base_link -> imu_link
```

The transform uses the same heading-derived yaw orientation as the IMU message.

### Diagnostics

Topic:

```text
/diagnostics
```

The diagnostics output reports:

```text
Connection
Frequency (Hz)
```

During normal operation the measured ROS2 publish rate is approximately 10 Hz and matches the embedded output rate.

If valid sensor data is no longer received, diagnostics reports the loss of recent sensor data rather than treating stale data as current.

## ROS2 Verification

After the node is configured and active:

### Check Published Interfaces

```bash
ros2 topic list
```

Expected application topics include:

```text
/imu/data
/diagnostics
/tf
```

### Check One IMU Message

```bash
ros2 topic echo /imu/data --once
```

Verify:

- `header.frame_id` is `imu_link`
- the timestamp is populated
- the orientation quaternion changes with heading

### Check Publish Frequency

```bash
ros2 topic hz /imu/data
```

Expected:

```text
average rate: ~10.0 Hz
```

### Check TF

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

The reported yaw should follow the heading received from the embedded device.

### Check Diagnostics

```bash
ros2 topic echo /diagnostics --once
```

During healthy operation verify:

```text
Connection: Connected
Frequency (Hz): approximately 10
```

### Check Lifecycle Deactivation

```bash
ros2 lifecycle set /bno085_driver deactivate
ros2 lifecycle get /bno085_driver
```

Expected:

```text
inactive [2]
```

While inactive, `/imu/data` publishing stops while the serial port remains allocated/open.

Reactivate with:

```bash
ros2 lifecycle set /bno085_driver activate
```

### Cleanup

Deactivate first if necessary, then:

```bash
ros2 lifecycle set /bno085_driver deactivate
ros2 lifecycle set /bno085_driver cleanup
```

Expected final state:

```text
unconfigured [1]
```

## Testing Error Behavior

### Embedded Sensor-Recovery Test

1. Run the system normally until the sensor state is `RUNNING`.
2. Disconnect only the BNO085 power while keeping the NUCLEO connected.
3. The embedded firmware detects loss of valid sensor communication/data and enters `RECOVERY`.
4. Reconnect the sensor.
5. The firmware retries initialization, calibration, and returns to `RUNNING` after successful recovery.

### Invalid-Heading Test

An out-of-range heading is rejected by the NMEA formatter. The communication error state/counter records the NMEA formatting error and no invalid frame is transmitted.

### ROS2 Connection-Loss Test

While the ROS2 node remains active, interrupt valid sensor data from the embedded side and inspect:

```bash
ros2 topic echo /diagnostics --once
```

The diagnostics status must indicate that recent sensor data is not being received.

## Performance Validation

The embedded output rate is fixed at approximately 10 Hz, and ROS2 publishing has been verified with:

```bash
ros2 topic hz /imu/data
```

to match the embedded rate.

The Kalman filter was also evaluated with stationary and dynamic heading tests using STM32CubeMonitor. The final process-noise setting is:

```text
Q = 0.25 uT^2/s
```

Detailed period, jitter, filtering, and ROS2 validation measurements are documented separately in the project performance report under `docs/`.

## Design Decisions

- Sensor packet processing is kept outside the GPIO ISR.
- Sensor processing and communication are separated into different FreeRTOS tasks.
- A queue is used for inter-task heading transfer.
- UART transmission is interrupt-driven and buffered.
- Sensor failures are recoverable through an explicit state machine.
- Measurement noise was characterized experimentally before Kalman tuning.
- The ROS2 bridge uses a LifecycleNode so resource lifetime and data streaming are explicitly controlled.
- C++ resources such as the serial-port object are managed with RAII/smart pointers.

## Limitations and Possible Improvements

- The published heading is magnetic heading; magnetic declination is not applied.
- The current heading calculation assumes that the sensor is approximately level.
- Tilt compensation is not implemented.
- The application uses the BNO085 calibrated magnetic-field report rather than register-level uncalibrated magnetometer values.
- The ROS2 `Imu` message contains heading-derived yaw orientation only; angular velocity and linear acceleration are not supplied by this pipeline.
- The current implementation uses a UART TX ring buffer on the embedded side; a separate RX ring buffer is not implemented.
- `/dev/ttyACM0` is the current default serial device and could be exposed as a ROS2 parameter in a future revision.
- A future version could use BNO085 rotation-vector output for full roll/pitch/yaw orientation and tilt-compensated heading.

## Performance Report and Presentation

A detailed performance report will be maintained under `docs/`, including:

- embedded average period
- embedded minimum/maximum period
- embedded jitter
- ROS2 publish-rate validation
- Kalman stationary-noise characterization
- Kalman dynamic-response characterization
- sensor-loss/recovery validation

The final presentation will summarize the architecture, key design decisions, validation results, challenges, and possible improvements.
