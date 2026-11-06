# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
import time

import rclpy
from rclpy.node import Node
from rtf_sensors_msgs.msg import CustomPressureTemperature
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_tca9548a

try:
    import adafruit_dps310
    from adafruit_dps310.advanced import Mode as _DPS310Mode
    from adafruit_dps310.advanced import Rate as _DPS310Rate
    from adafruit_dps310.advanced import SampleCount as _DPS310Count
except ImportError:
    adafruit_dps310 = None


# Sensor type constants
SENSOR_DPS310 = "dps310"


# The two possible I2C addresses for DPS310 (SDO pin low/high)
DPS310_ADDR_LOW  = 0x76
DPS310_ADDR_HIGH = 0x77

# Default time (s) a sensor gets for each wait step of its initialisation
# (reset, calibration coefficients, first measurement) before it is skipped.
DEFAULT_INIT_TIMEOUT_S = 2.0

# DPS310 measurement config: (measurements per second, oversampling count).
# Defaults match what adafruit_dps310 has always applied (64 Hz, 64x for both).
# NOTE: 64x oversampling takes ~104 ms per measurement, so the chip cannot
# keep up with 64 Hz: with these defaults it delivers ~5 fresh samples/s.
DEFAULT_PRESSURE_RATE_HZ = 64
DEFAULT_PRESSURE_OVERSAMPLE = 64
DEFAULT_TEMPERATURE_RATE_HZ = 64
DEFAULT_TEMPERATURE_OVERSAMPLE = 64

# Datasheet measurement time (ms) per oversampling count.
_DPS310_MEAS_TIME_MS = {1: 3.6, 2: 5.2, 4: 8.4, 8: 14.8, 16: 27.6, 32: 53.2, 64: 104.4, 128: 206.8}


def dps310_busy_fraction(p_rate, p_os, t_rate, t_os) -> float:
    """Fraction of each second the DPS310 needs to be measuring for this config.
    Above 1.0 the chip cannot sustain the requested rates (datasheet limit)."""
    return (p_rate * _DPS310_MEAS_TIME_MS[p_os] + t_rate * _DPS310_MEAS_TIME_MS[t_os]) / 1000.0


def _wait_until(predicate, timeout_s: float, what: str):
    """Poll predicate() every 1 ms until it is true, or raise TimeoutError."""
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(f'timed out after {timeout_s:.1f} s waiting for {what}')
        time.sleep(0.001)


class _SafeTCAChannel(adafruit_tca9548a.TCA9548A_Channel):
    """TCA9548A channel that always releases the I2C lock, even if switching fails.

    Upstream, try_lock() takes the bus lock and then writes the channel-select
    byte. If that write raises (e.g. Remote I/O error because the multiplexer
    stopped answering), the lock is never released. Every later transaction then
    spins forever in `while not try_lock()` at 100 % CPU, which hangs the node.
    """

    def try_lock(self) -> bool:
        while not self.tca.i2c.try_lock():
            time.sleep(0)
        try:
            self.tca.i2c.writeto(self.tca.address, self.channel_switch)
        except Exception:
            self.tca.i2c.unlock()
            raise
        return True

    def unlock(self) -> None:
        try:
            self.tca.i2c.writeto(self.tca.address, b"\x00")
        finally:
            self.tca.i2c.unlock()


if adafruit_dps310 is not None:
    class _DPS310WithTimeout(adafruit_dps310.DPS310):
        """Adafruit DPS310 driver with a timeout on its busy-wait loops.

        Upstream, reset(), _read_calibration(), wait_temperature_ready() and
        wait_pressure_ready() poll a status bit forever. A sensor that answers on
        the bus but never reports ready (bad wiring, flaky power) would then hang
        the whole node during initialisation. Here each wait raises TimeoutError
        instead, so rtf_MultiPT logs that sensor as failed and carries on.
        """

        def __init__(self, i2c_bus, address: int, timeout_s: float,
                     pressure_rate_hz: int = DEFAULT_PRESSURE_RATE_HZ,
                     pressure_oversample: int = DEFAULT_PRESSURE_OVERSAMPLE,
                     temperature_rate_hz: int = DEFAULT_TEMPERATURE_RATE_HZ,
                     temperature_oversample: int = DEFAULT_TEMPERATURE_OVERSAMPLE):
            # These must exist before the base __init__ runs initialize().
            self._timeout_s = timeout_s
            self._cfg = (getattr(_DPS310Rate, f'RATE_{int(pressure_rate_hz)}_HZ'),
                         getattr(_DPS310Count, f'COUNT_{int(pressure_oversample)}'),
                         getattr(_DPS310Rate, f'RATE_{int(temperature_rate_hz)}_HZ'),
                         getattr(_DPS310Count, f'COUNT_{int(temperature_oversample)}'))
            super().__init__(i2c_bus, address)

        def initialize(self) -> None:
            # Same as DPS310_Advanced.initialize(), but with a configurable rate/oversampling.
            self.reset()
            (self.pressure_rate, self.pressure_oversample_count,
             self.temperature_rate, self.temperature_oversample_count) = self._cfg
            self.mode = _DPS310Mode.CONT_PRESTEMP
            self.wait_temperature_ready()
            self.wait_pressure_ready()

        def reset(self) -> None:
            # Same steps as adafruit_dps310.basic.DPS310.reset(), with a bounded wait.
            self._reset_register = 0x89
            time.sleep(0.010)
            _wait_until(lambda: self._sensor_ready, self._timeout_s, 'sensor ready after reset')
            self._correct_temp()
            self._read_calibration()
            self._temp_measurement_src_bit = self._calib_coeff_temp_src_bit

        def _read_calibration(self) -> None:
            _wait_until(lambda: self._coefficients_ready, self._timeout_s,
                        'calibration coefficients')
            super()._read_calibration()     # its own wait loop now returns immediately

        def wait_temperature_ready(self) -> None:
            if self._mode_bits in (_DPS310Mode.IDLE, _DPS310Mode.ONE_PRESSURE,
                                   _DPS310Mode.CONT_PRESSURE):
                raise RuntimeError('sensor mode does not measure temperature')
            _wait_until(lambda: self._temp_ready, self._timeout_s, 'first temperature measurement')

        def wait_pressure_ready(self) -> None:
            if self._mode_bits in (_DPS310Mode.IDLE, _DPS310Mode.ONE_TEMPERATURE,
                                   _DPS310Mode.CONT_TEMP):
                raise RuntimeError('sensor mode does not measure pressure')
            _wait_until(lambda: self._pressure_ready, self._timeout_s, 'first pressure measurement')


def _twos24(b2: int, b1: int, b0: int) -> int:
    v = (b2 << 16) | (b1 << 8) | b0
    return v - (1 << 24) if v & (1 << 23) else v


def _read_dps310(sensor):
    """Read raw calibration coefficients and measurements from a DPS310 sensor.
    Returns a dict of field_name -> value ready to stamp onto the message.

    Pressure and temperature come from ONE burst read of registers 0x00-0x05
    (PSR_B2..TMP_B0). The adafruit properties used before read the raw registers
    5 times per sample (pressure/temperature each re-read them), and every read
    also switched the TCA channel on and off: 15 I2C transactions instead of 3.
    The compensation math is the same as adafruit_dps310's pressure/temperature.
    """
    buf = bytearray(6)
    with sensor.i2c_device as i2c:      # selects the TCA channel; deselects on exit
        i2c.write_then_readinto(bytes((0x00,)), buf)
    raw_pressure = _twos24(buf[0], buf[1], buf[2])
    raw_temp = _twos24(buf[3], buf[4], buf[5])

    sp = raw_pressure / sensor._pressure_scale
    st = raw_temp / sensor._temp_scale
    pressure_pa = (sensor._c00
                   + sp * (sensor._c10 + sp * (sensor._c20 + sp * sensor._c30))
                   + st * (sensor._c01 + sp * (sensor._c11 + sp * sensor._c21)))
    temperature_c = st * sensor._c1 + sensor._c0 / 2.0

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
        pressure=pressure_pa,               # Pa
        temperature=temperature_c,          # °C
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
    init_timeout_s : float – max wait per initialisation step before a sensor
                             is skipped (default 2.0)
    pressure_rate_hz, pressure_oversample,
    temperature_rate_hz, temperature_oversample
                   : int   – DPS310 measurement rate / oversampling, each one of
                             1, 2, 4, ..., 128 (defaults 64 / 64 / 64 / 64). The
                             chip can only sustain configs whose total measuring
                             time fits in 1 s (logged at startup).

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
        self._init_timeout_s = float(
            self.declare_parameter('init_timeout_s', DEFAULT_INIT_TIMEOUT_S).value)

        # DPS310 measurement config, shared by all sensors of this node.
        self._dps310_cfg = dict(
            pressure_rate_hz=int(self.declare_parameter(
                'pressure_rate_hz', DEFAULT_PRESSURE_RATE_HZ).value),
            pressure_oversample=int(self.declare_parameter(
                'pressure_oversample', DEFAULT_PRESSURE_OVERSAMPLE).value),
            temperature_rate_hz=int(self.declare_parameter(
                'temperature_rate_hz', DEFAULT_TEMPERATURE_RATE_HZ).value),
            temperature_oversample=int(self.declare_parameter(
                'temperature_oversample', DEFAULT_TEMPERATURE_OVERSAMPLE).value),
        )
        c = self._dps310_cfg
        for key in ('pressure_rate_hz', 'temperature_rate_hz',
                    'pressure_oversample', 'temperature_oversample'):
            if c[key] not in (1, 2, 4, 8, 16, 32, 64, 128):
                raise ValueError(f'{key}={c[key]} must be one of 1, 2, 4, 8, 16, 32, 64, 128')
        busy = dps310_busy_fraction(c['pressure_rate_hz'], c['pressure_oversample'],
                                    c['temperature_rate_hz'], c['temperature_oversample'])
        msg = (f'DPS310 config: pressure {c["pressure_rate_hz"]} Hz x{c["pressure_oversample"]}, '
               f'temperature {c["temperature_rate_hz"]} Hz x{c["temperature_oversample"]} '
               f'(chip busy {busy * 100:.0f}% of the time)')
        if busy > 1.0:
            self.get_logger().warn(
                msg + ': over 100%, so the sensor cannot deliver the requested rate and '
                'most published messages will repeat the previous measurement')
        else:
            self.get_logger().info(msg)
        self._sensors: list[SensorEntry]      = []
        # Not self._publishers: rclpy.Node uses that name internally and
        # create_publisher() appends to it, which would list every publisher twice.
        self._sensor_publishers: list        = []
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

            tca_channel = _SafeTCAChannel(self._tca[tca_idx], tca_port)

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
            self._sensor_publishers.append(pub)

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
            if adafruit_dps310 is None:
                raise RuntimeError('adafruit_dps310 is not installed')
            return _DPS310WithTimeout(tca_channel, i2c_address, self._init_timeout_s,
                                      **self._dps310_cfg)

    def _callback(self):
        """Read every sensor in sequence, one at a time, then publish."""
        stamp = self.get_clock().now().to_msg()

        for entry, pub, msg in zip(self._sensors, self._sensor_publishers, self._msgs):
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
    """Entry-point class for multi_pt_node (see nodes/sensor_nodes.py).

    It used to also create a second rtf_MultiPT inside itself. That initialised
    every sensor twice, and the inner copy was never spun, so it published nothing.
    """
    pass


# Keep the original single-sensor entry points so existing launch files keep
# working while you migrate.

class rtf_dps310(Node):
    """Legacy single-sensor wrapper — use rtf_MultiPT for new deployments."""
    def __init__(self):
        super().__init__('rtf_dps310')
        self.get_logger().warn(
            'rtf_dps310 is deprecated. Migrate to rtf_MultiPT (multi_pt_node).')

