#import module

import time
import gps
import motor
import bmx055
import bme280
import send
import take

def bf_launch():
    start_time = time.time()
    try:
        print('Checking BME280')
        while time.time() - start_time < 5:
            temp,pres,hum,alt = bme280.bme280_read()
            print(temp,pres,hum,alt)
            time.sleep(0.8)

        print('Checking BMX055')
        while time.time() - start_time < 10:
            bmxData = bmx055.bmx055_read()
            print(bmxData)
            time.sleep(1)

        print('Checking GPS')
        while time.time() - start_time < 15:
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
            time.sleep(1)
        
        print('Checking Motor')
        motor.move(30, 30, 1.5)
        time.sleep(2)
        motor.move(-30, -30, 1.5)

        print('Checking IM920')
        text = 'radio_check'
        if text =="A":
            try:
                print("キルで")
                send.send_reset(t_reset = 10)
                print("finish")
            except:
                pi.write(sendPin, 0)

        
        send.send_data(text)
        print('送信しました')

        time.sleep(1)

        print('Checking Camera')
        take.picture('../imgs/test_imgs/take', 320, 240)
        print('finish')

    except KeyboardInterrupt:
        print("\r\n")

if __name__ == '__main__':
    bmx055.bmx055_setup()
    bme280.bme280_setup()
    bme280.bme280_calib_param()
    gps.open_gps()
    motor.setup()

    time.sleep(1)

    print('Before Launch Check Start')
    bf_launch()


