import board
import adafruit_mpu6050

class mpu6050:
    GRAVITIY_MS2 = 9.80665
    mpu = None

    def __init__(self):
        i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(i2c)

    def get_accel_data(self, g=False):
        # Default range +/- 2g; LSB sensitivity = 16384 LSB/g
        x = self.mpu.acceleration[0]
        y = self.mpu.acceleration[1]
        z = self.mpu.acceleration[2]
        
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
        x = self.mpu.gyro[0]
        y = self.mpu.gyro[1]
        z = self.mpu.gyro[2]
        
        vals = {}
        vals["x"] = x / 131.0
        vals["y"] = y / 131.0
        vals["z"] = z / 131.0
        return vals
