import bmx055
import bme280
import gps
import melt
import land
import send
import time
import pigpio
from smbus import SMBus
import smbus
import logging
import motor

#着地衝撃試験用プログラム

#使うモジュールのインストール
bmx055.bmx055_setup()
bme280.bme280_setup()
bme280.bme280_calib_param()
motor.setup()

print("wait 20s")
time.sleep(10)

#着地判定
print("land detect start")
send.send_data("TXDU 0001,0000")

landcount = 0
pressdata = [0.0, 0.0, 0.0, 0.0]

while True:
    presslandjudge = 0
    landcount, presslandjudge = land.pressdetect_land(0.1)
    print(f'count:{landcount}\tjudge:{presslandjudge}')
    if presslandjudge == 1:
        print('Press')
        send.send_data("TXDU 0001,1000")
        print('##--landed--##')
        send.send_data("TXDU 0001,1111")
        break
    else:
        print('Press unfulfilled')
        send.send_data("TXDU 0001,0001")

#溶断回路
print("melt start")
meltPin = 4
try:
    melt.down()
except:
    pi.write(meltPin, 0)

print("melt finish")


time.sleep(2)

#-----少しだけ回転させる-----#
#-----分離機構の扉が下を向いてしまったときように-----#
motor.move(40, 40, 0.05)
print("rotate finish")

#-----GPSセンサのデータ取得-----#
# try:
#     gps.open_gps()
#     t_start = time.time()
#     while True:
#         utc, lat, lon, sHeight, gHeight = gps.read_gps()
#         if utc == -1.0:
#             if lat == -1.0:
#                 print("Reading gps Error")
#                 # pass
#             else:
#                 # pass
#                 print("Status V")
#         else:
#             # pass
#             print(utc, lat, lon, sHeight, gHeight)
#         time.sleep(1)
# except KeyboardInterrupt:
#     gps.close_gps()
#     print("\r\nKeyboard Intruppted, Serial Closed")
# except:
#     gps.close_gps()
#     print(traceback.format_exc())

#-----気圧センサのデータ取得-----#

# try:
#     while 1:
#         temp,pres,hum,alt = bme280.bme280_read()
#         print(str(pres) + "\t" + str(alt) + "\t" + str(temp) + "\t" + str(hum))
#         #with open("preslog.txt","w")as f:
#         #	f.write(str(pres)+ "\t" + str(alt) + "\t"+str(temp) + "\t" + str(hum) + "\n")
#         time.sleep(0.8)
# except KeyboardInterrupt:
#     print("\r\n")
# except Exception as e:
#     print(e.message())

#-----9軸センサのデータ読み取り-----#
# try:
#     bmx055.bmx055_setup()
#     time.sleep(0.2)
#     while 1:
#         bmxData = bmx055.bmx055_read()
#         print(bmxData)
#         time.sleep(1)

# except KeyboardInterrupt:
#     print()
# except Exception as e:
#     print(e)

#-----3つのデータの読み取り-----#
try:
    gps.open_gps()
    t_start = time.time()
    while True:
        bmxData = bmx055.bmx055_read()
        print("-----bmx055 data-----")
        print(bmxData)
        print("-----bme280 data-----")
        temp,pres,hum,alt = bme280.bme280_read()
        print(str(pres) + "\t" + str(alt) + "\t" + str(temp) + "\t" + str(hum))
        print("-----gps data-----")
        utc, lat, lon, sHeight, gHeight = gps.read_gps()
        if utc == -1.0:
            if lat == -1.0:
                print("Reading gps Error")
                # pass
            else:
                # pass
                print("Status V")
        else:
            # pass
            print(utc, lat, lon, sHeight, gHeight)

        print("----------")
        time.sleep(1)

except KeyboardInterrupt:
    gps.close_gps()
    print("\r\n")
except Exception as e:
    gps.close_gps()
    print(e.message())
