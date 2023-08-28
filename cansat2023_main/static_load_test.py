import bmx055
import bme280
import gps
import time
import pigpio
import other
import motor
import send
import melt
import land
from collections import deque
import datetime

def static_load_test():
    logpath = other.filename('./log/static_load/static_load_log', 'txt')

    other.log(logpath, datetime.datetime.now(), "-----Static Load Test Start-----")

    while True:
        try:
            accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz  = bmx055.bmx055_read()
            print("-----bmx055 data-----")
            print(accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz)            
            temp, press, hum, alt = bme280.bme280_read()
            print("-----bme280 data-----")
            print(temp, press, hum, alt)
            print("-----gps data-----")            
            utc, lat, lon, sHeight, gHeight = gps.read_gps()
            if utc == -1.0:
                if lat == -1.0:
                    print("Reading gps Error")
                else:
                    print("Status V")
            else:
                print(utc, lat, lon, sHeight, gHeight)
            
            time.sleep(1)

            other.log(logpath, datetime.datetime.now(), accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, temp, press, hum, alt, utc, lat, lon, sHeight, gHeight)

        except KeyboardInterrupt:
            gps.close_gps()
            print("\r\n")
        except Exception as e:
            gps.close_gps()
            print(e.message())

if __name__ == '__main__':
    #セットアップ
    bmx055.bmx055_setup()
    bme280.bme280_setup()
    bme280.bme280_calib_param()
    gps.open_gps()
    motor.setup()

    #-----振動試験-----#
    static_load_test()