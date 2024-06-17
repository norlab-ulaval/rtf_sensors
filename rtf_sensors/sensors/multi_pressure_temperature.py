# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
import rclpy
from rclpy.node import Node
from rtf_sensors_msgs.msg import CustomPressureTemperature
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_tca9548a

try:
    import adafruit_dps310
except ImportError:
    pass


# Sensor type constants
SENSOR_DPS310 = "dps310"


# The two possible I2C addresses for DPS310 (SDO pin low/high)
DPS310_ADDR_LOW  = 0x76
DPS310_ADDR_HIGH = 0x77


def _read_dps310(sensor):
    """Read raw calibration coefficients and measurements from a DPS310 sensor.
    Returns a dict of field_name -> value ready to stamp onto the message."""
    raw_pressure = sensor._raw_pressure
    if raw_pressure & (1 << (24 - 1)):
        raw_pressure -= 1 << 24

    raw_temp = sensor._raw_temperature
    if raw_temp & (1 << (24 - 1)):
        raw_temp -= 1 << 24

    return dict(
        c0=float(sensor._c0),
        c1=float(sensor._c1),
        c00=float(sensor._c00),
        c01=float(sensor._c01),
        c10=float(sensor._c10),
        c11=float(sensor._c11),
        c20=float(sensor._c20),
        c21=float(sensor._c21),
        c30=float(sensor._c30),
        raw_pressure=float(raw_pressure),
        scale_pressure=float(sensor._pressure_scale),
        raw_temperature=float(raw_temp),
        scale_temperature=float(sensor._temp_scale),
        pressure=sensor.pressure * 100.0,   # hPa -> Pa
        temperature=sensor.temperature,      # °C
    )


class SensorEntry:
    """Holds everything needed to talk to one physical sensor."""

    def __init__(self, tca_channel, sensor_obj, topic: str, frame_id: str):
        self.tca_channel = tca_channel   # adafruit_tca9548a channel object
        self.sensor      = sensor_obj    # adafruit driver instance
        self.topic       = topic
        self.frame_id    = frame_id


class rtf_MultiPT(Node):
    """Single ROS 2 node that manages up to 16 DPS310 pressure/temperature
    sensors spread across one or two TCA9548A I2C multiplexers.

    Each sensor is identified by:
      - tca_index  : 0 = first TCA (addr 0x70), 1 = second TCA (addr 0x71)
      - tca_port   : 0-7  (channel on the chosen TCA)
      - i2c_address: 0x76 or 0x77 (SDO pin low/high on the DPS310)

    Parameter format
    ----------------
    sensor_count   : int   – total number of sensors (default 1)

    For sensor N (0-indexed):
      sensor_N_type        : str  – sensor type ("dps310", "bmp390", "lps22")
      sensor_N_tca_index   : int  – which TCA (0 or 1)
      sensor_N_tca_port    : int  – TCA channel 0-7
      sensor_N_i2c_address : int  – I2C address of the sensor
      sensor_N_frame_id    : str  – ROS frame_id for the message header
      sensor_N_topic       : str  – ROS topic to publish on
    """

    # TCA9548A I2C addresses for tca_index 0 and 1
    TCA_ADDRESSES = [0x70, 0x71]

    def __init__(self):
        super().__init__('rtf_multi_pt')

        i2c_bus = self.declare_parameter('i2c_bus', 1).value
        self._i2c = I2C(i2c_bus)

        # Instantiate both TCA objects unconditionally; they are cheap and
        # having both ready avoids any lazy-init race conditions.
        self._tca = [
            adafruit_tca9548a.TCA9548A(self._i2c, address=addr)
            for addr in self.TCA_ADDRESSES
        ]

        sensor_count = self.declare_parameter('sensor_count', 1).value
        self._sensors: list[SensorEntry]      = []
        self._publishers: list               = []
        self._msgs: list[CustomPressureTemperature] = []

        for n in range(sensor_count):
            p = f'sensor_{n}'
            s_type    = self.declare_parameter(f'{p}_type',        SENSOR_DPS310).value
            tca_idx   = self.declare_parameter(f'{p}_tca_index',   0).value
            tca_port  = self.declare_parameter(f'{p}_tca_port',    n % 8).value
            i2c_addr  = self.declare_parameter(f'{p}_i2c_address', DPS310_ADDR_HIGH).value
            frame_id  = self.declare_parameter(f'{p}_frame_id',    f'{s_type}_{n}').value
            topic     = self.declare_parameter(f'{p}_topic',       f'/sensors/pressure_{n}').value

            if tca_idx not in (0, 1):
                self.get_logger().error(
                    f'sensor_{n}: tca_index {tca_idx} is invalid (must be 0 or 1). Skipping.')
                continue

            tca_channel = self._tca[tca_idx][tca_port]

            try:
                sensor_obj = self._init_sensor(s_type, tca_channel, i2c_addr)
            except Exception as e:
                self.get_logger().error(
                    f'sensor_{n} ({s_type} @ TCA{tca_idx}[{tca_port}] addr={hex(i2c_addr)})'
                    f' failed to initialise: {e}')
                continue

            entry = SensorEntry(tca_channel, sensor_obj, topic, frame_id)
            self._sensors.append(entry)

            pub = self.create_publisher(CustomPressureTemperature, topic, 10)
            self._publishers.append(pub)

            msg = CustomPressureTemperature()
            msg.header.frame_id = frame_id
            self._msgs.append(msg)

            self.get_logger().info(
                f'sensor_{n}: {s_type} @ TCA{tca_idx}[{tca_port}] '
                f'addr={hex(i2c_addr)} → {topic}')

        if not self._sensors:
            self.get_logger().error('No sensors initialised successfully. Node idle.')
            return

        Hertz = 100
        self.create_timer(1.0 / Hertz, self._callback)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _init_sensor(self, s_type: str, tca_channel, i2c_address: int):
        """Instantiate the correct Adafruit driver for the given sensor type."""
        if s_type == SENSOR_DPS310:
            return adafruit_dps310.DPS310(tca_channel, i2c_address)

    def _callback(self):
        """Read every sensor in sequence, one at a time, then publish."""
        stamp = self.get_clock().now().to_msg()

        for entry, pub, msg in zip(self._sensors, self._publishers, self._msgs):
            # Re-select the TCA channel immediately before each transaction.
            # This is the critical step that prevents cross-talk between sensors
            # running in the same process — only one channel is ever active.
            _ = entry.tca_channel  # accessing the channel object re-asserts selection

            try:
                data = _read_dps310(entry.sensor)
            except OSError as e:
                self.get_logger().warn(
                    f'I2C error reading {entry.topic}: {e}', throttle_duration_sec=5.0)
                continue

            msg.header.stamp    = stamp
            msg.c0              = data['c0']
            msg.c1              = data['c1']
            msg.c00             = data['c00']
            msg.c01             = data['c01']
            msg.c10             = data['c10']
            msg.c11             = data['c11']
            msg.c20             = data['c20']
            msg.c21             = data['c21']
            msg.c30             = data['c30']
            msg.raw_pressure    = data['raw_pressure']
            msg.scale_pressure  = data['scale_pressure']
            msg.raw_temperature = data['raw_temperature']
            msg.scale_temperature = data['scale_temperature']
            msg.pressure        = data['pressure']
            msg.temperature     = data['temperature']

            pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

class rtf_multi_pt(rtf_MultiPT):
    def __init__(self):
        super().__init__()
        self.sensor = rtf_MultiPT()



# Keep the original single-sensor entry points so existing launch files keep
# working while you migrate.

class rtf_dps310(Node):
    """Legacy single-sensor wrapper — use rtf_MultiPT for new deployments."""
    def __init__(self):
        super().__init__('rtf_dps310')
        self.get_logger().warn(
            'rtf_dps310 is deprecated. Migrate to rtf_MultiPT (multi_pt_node).')

