import time
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_vl53l1x

i2c = I2C(4)
sensor = adafruit_vl53l1x.VL53L1X(i2c, address=0x29)

sensor.distance_mode = 1
sensor.timing_budget = 100
sensor.start_ranging()

print("VL53L1X start ranging...")

while True:
    if sensor.data_ready:
        distance_cm = sensor.distance
        print(distance_cm, "cm")
        sensor.clear_interrupt()
    time.sleep(0.1)
