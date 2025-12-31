import smbus
import math

class mpu6050:
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up
        self.bus.write_byte_data(self.address, 0x6B, 0x00)

    def read_i2c_word(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def get_accel_data(self, g=False):
        # Default range +/- 2g; LSB sensitivity = 16384 LSB/g
        x = self.read_i2c_word(0x3B)
        y = self.read_i2c_word(0x3D)
        z = self.read_i2c_word(0x3F)
        
        vals = {}
        vals["x"] = x / 16384.0
        vals["y"] = y / 16384.0
        vals["z"] = z / 16384.0
        
        if g:
            return vals
        else:
            vals["x"] = vals["x"] * self.GRAVITIY_MS2
            vals["y"] = vals["y"] * self.GRAVITIY_MS2
            vals["z"] = vals["z"] * self.GRAVITIY_MS2
            return vals

    def get_gyro_data(self):
        # Default range +/- 250 deg/s; LSB = 131 LSB/deg/s
        x = self.read_i2c_word(0x43)
        y = self.read_i2c_word(0x45)
        z = self.read_i2c_word(0x47)
        
        vals = {}
        vals["x"] = x / 131.0
        vals["y"] = y / 131.0
        vals["z"] = z / 131.0
        return vals
