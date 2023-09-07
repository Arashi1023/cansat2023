import release
import land
import melt
import parachute_avoid
import paradetection
import time
import bme280
import pigpio
import send
import motor
import traceback


if __name__  == "__main__":

    ###-------release judge -------###
    print("START: Release judge")
    thd_press_release = 0.1
    pressreleasecount = 0
    pressreleasejudge = 0
    t_delta_release = 10
    bme280.bme280_setup()
    bme280.bme280_calib_param()
    press_d = 0

    while True:
        press_count_release, press_judge_release = release.pressdetect_release(thd_press_release, t_delta_release)
        print(f'count:{pressreleasecount}\tjudge{pressreleasejudge}')
        if press_count_release  > 3:
            print('Press')
            send.send_data("TXDU 0001.A001")
            break
        else:
            print('unfulfilled')

    print("release finish!!!")
    send.send_data("TXDU 0001.AAAA")

    ###-------land judge -------###
    print("START: Land judge")
    send.send_data("TXDU 0001,B000")

    #bme280.bme280_setup()
    #bme280.bme280_calib_param()

    landcount = 0
    pressdata = [0.0, 0.0, 0.0, 0.0]

    while True:
        presslandjudge = 0
        landcount, presslandjudge = land.pressdetect_land(0.1)
        print(f'count:{landcount}\tjudge:{presslandjudge}')
        if presslandjudge == 1:
            print('Press')
            send.send_data("TXDU 0001,B002")
            print('##--landed--##')
            break
        else:
            print('Press unfulfilled')
            send.send_data("TXDU 0001,B001")
    print("land finish!!!")
    send.send_data("TXDU 0001,BBBB")

    ###-------melt-------###

    print("START: Melt")
    pi = pigpio.pi()

    meltPin = 4
    try:
        melt.down()
        send.send_data("TXDU 0001,C001")
    except:
        pi.write(meltPin, 0)
    print("melt finish!!!")
    send.send_data("TXDU 0001,CCCC")
    ###------paraavo-------###
    try:
        motor.setup()

        print("START: Parachute avoidance")

        flug, area, gap, photoname = paradetection.para_detection("photostorage/photostorage_paradete/para", 320, 240,
                                                                  200, 10, 120, 1)
        print(f'flug:{flug}\tarea:{area}\tgap:{gap}\tphotoname:{photoname}')
        print("paradetection phase success")
        count_paraavo = 0
        while count_paraavo < 3:
            flug, area, gap, photoname = paradetection.para_detection("photostorage/photostorage_paradete/para", 320,
                                                                      240, 200, 10, 120, 1)
            print(f'flug:{flug}\tarea:{area}\tgap:{gap}\tphotoname:{photoname}')
            parachute_avoid.parachute_avoidance(flug, gap)
            print(flug)
            if flug == -1 or flug == 0:
                count_paraavo += 1
                print(count_paraavo)

        print("パラシュート回避完了")

    except KeyboardInterrupt:
        print("emergency!")

    except:
        print(traceback.format_exc())
        
    print("paraavo finish!!!")
    send.send_data("TXDU 0001,DDDD")



