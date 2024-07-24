# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################
import rclpy
from rclpy.node import Node
from rtf_sensors_msgs.msg import CustomTemperature, CustomPressure
#from geometry_msgs.msg import PointStamped
import board
import busio

try:
    import adafruit_dps310
except ImportError:
    pass

try:
    import adafruit_bmp3xx
except ImportError:
    pass

try:
    import adafruit_lps2x
except ImportError:
    pass

class rtf_PT(Node):
    def __init__(self, name, i2c=None):
        super().__init__(name)

        if i2c is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c

        Hertz = 100
        rate = 1.0/Hertz
        self.sensor = None

        self.timer = self.create_timer(rate, self.callback)

        self.pub_temp = self.create_publisher(CustomTemperature, 'temperature', 10)
        self.pub_pressure = self.create_publisher(CustomPressure, 'pressure', 10)
        #self.pub_altitude = self.create_publisher(PointStamped, 'altitude', 10)

        self.frame_id = self.declare_parameter('frame_id', "dps310").value
        self.i2c_address = self.declare_parameter('i2c_address', "0x77").value

        self.temp_msg = CustomTemperature()
        self.temp_msg.header.frame_id = self.frame_id
        
        self.pressure_msg = CustomPressure()
        self.pressure_msg.header.frame_id = self.frame_id

    def callback(self):
        stamp = self.get_clock().now().to_msg()
        self.temp_msg.header.stamp = stamp
        self.temp_msg.c0 = float(self.sensor._c0)
        self.temp_msg.c1 = float(self.sensor._c1)
        raw_temp = self.sensor._raw_temperature
        # raw temperature value is a two's complement so we need to deal with that.
        # 24 is the number of bits for the raw measurement bit shift when necessary.
        if raw_temp & (1 << (24 - 1)):
            raw_temp -= 1 << 24
        self.temp_msg.raw_temperature = float(raw_temp)
        self.temp_msg.scale_temperature = float(self.sensor._temp_scale)
        t = self.sensor.temperature
        self.temp_msg.temperature = t # C
        self.pub_temp.publish(self.temp_msg)
        
        self.pressure_msg.header.stamp = stamp
        self.pressure_msg.c00 = float(self.sensor._c00)
        self.pressure_msg.c01 = float(self.sensor._c01)
        self.pressure_msg.c10 = float(self.sensor._c10)
        self.pressure_msg.c11 = float(self.sensor._c11)
        self.pressure_msg.c20 = float(self.sensor._c20)
        self.pressure_msg.c21 = float(self.sensor._c21)
        self.pressure_msg.c30 = float(self.sensor._c30)
        raw_pressure = self.sensor._raw_pressure
        # raw pressure value is a two's complement so we need to deal with that.
        # 24 is the number of bits for the raw measurement bit shift when necessary.
        if raw_pressure & (1 << (24 - 1)):
            raw_pressure -= 1 << 24
        self.pressure_msg.raw_pressure = float(raw_pressure)
        self.pressure_msg.scale_pressure = float(self.sensor._pressure_scale)
        self.pressure_msg.raw_temperature = float(raw_temp)
        self.pressure_msg.scale_temperature = float(self.sensor._temp_scale)        
        p = self.sensor.pressure
        self.pressure_msg.pressure = p*100 # Pa
        self.pub_pressure.publish(self.pressure_msg)

        #msg = PointStamped()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = self.frame_id
        #msg.point.x = 0.0
        #msg.point.y = 0.0
        #msg.point.z = 44330. * (1. - (p/p0)**(1./5.255) )
        #self.pub_altitude.publish(msg)


class rtf_dps310(rtf_PT):
    def __init__(self, i2c=None):
        super().__init__('rtf_dps310', i2c)

        #self.sensor = adafruit_dps310.DPS310(self.i2c)
        #self.sensor = adafruit_dps310.DPS310(self.i2c, 0x76)
        self.sensor = adafruit_dps310.DPS310(self.i2c, self.i2c_address)


class rtf_bmp390(rtf_PT):
    def __init__(self, i2c=None):
        super().__init__('rtf_bmp390', i2c)

        self.sensor = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)


class rtf_lps22(rtf_PT):
    def __init__(self, i2c=None):
        super().__init__('rtf_lps22', i2c)

        self.sensor = adafruit_lps2x.LPS22(self.i2c)
