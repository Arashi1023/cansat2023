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

pi = pigpio.pi()


def land_shock():
    logpath = other.filename('./log/land_shock/land_shock_log', 'txt')

    pressdata = deque([0.0, 0.0], maxlen=2)
    land_count = 0

    other.log(logpath, datetime.datetime.now(), "-----Land Impact Test Start-----")

    t_start = time.time()  

    while time.time() - t_start < 10:
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

    other.log(logpath, datetime.datetime.now(), "-----Land Detect Start-----")
    time.sleep(1)

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

            pressdata.append(press)
            delta_press = pressdata[1] - pressdata[0]
            if delta_press < 0.1:
                land_count += 1
            else:
                land_count = 0

            other.log(logpath, datetime.datetime.now(), accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, temp, press, hum, alt, utc, lat, lon, sHeight, gHeight, delta_press, land_count)

            if land_count > 4:
                print("#-----Landed-----#")
                other.log(logpath, datetime.datetime.now(), "#-----Landed-----#")
                break

        except KeyboardInterrupt:
            gps.close_gps()
            print("\r\n")
        except Exception as e:
            gps.close_gps()
            print(e.message())
    
    print("#-----Melt Start-----#")
    other.log(logpath, datetime.datetime.now(), "#-----Melt Start-----#")
    meltPin = 4
    try:
        melt.down()
    except:
        pi.write(meltPin, 0)
    time.sleep(1)
    print("Melt Finish")
    other.log(logpath, datetime.datetime.now(), "-----Melt Finish-----")
    time.sleep(2)
    print("Motor Start")
    other.log(logpath, datetime.datetime.now(), "-----Motor Start-----")
    motor.move(30, 30, 0.2)
    print("Motor Stop")
    other.log(logpath, datetime.datetime.now(), "-----Motor Stop-----")

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
    land_shock()