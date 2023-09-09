#import module

import time
import gps
import motor
import bmx055
import bme280
import send

def bf_launch(check):
    start_time = time.time()
    if check == 'all'
        try:
            print('Checking BME280')
            while time.time() - start_time < 5:
                temp,pres,hum,alt = bme280.bme280_read()
                time.sleep(0.8)

            print('Checking BMX055')
            while time.time() - start_time < 10:
                bmxData = bmx055.bmx055_read()
                time.sleep(1)

            print('Checking GPS')
            while time.time() - start_time < 15:
                utc, lat, lon, sHeight, gHeight = read_gps()
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
                    send_reset(t_reset = 10)
                    print("finish")
                except:
                    pi.write(sendPin, 0)

            
            send_data(text)
            print('送信しました')
            received_text = receive_data()
            receive_data
            print("受信",received_text)

        except KeyboardInterrupt:
            print("\r\n")
        except Exception as e:
            print(e.message())

if __name__ == '__main__':
bmx055.bmx055_setup()
bme280.bme280_setup()
bme280.bme280_calib_param()
gps.open_gps()
motor.setup()

time.sleep(1)

print('Before Launch Check Start')


