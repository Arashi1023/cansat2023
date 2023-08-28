import release
import land
import melt
import parachute_avoid
import paradetection
import time
import bme280
import pigpio
import send
import traceback
import other
import datetime
import wgps_beta_photo_running as photo_running
import cv2
import save_photo as save_img
import take
import motor
import beta_para_avoid as para_avoid
import stuck2

#variable for log
log_phase=other.filename('/home/dendenmushi/cansat2023/sequence/log/phaselog','txt')
log_release=other.filename('/home/dendenmushi/cansat2023/sequence/log/releaselog','txt')
log_landing=other.filename('/home/dendenmushi/cansat2023/sequence/log/landinglog','txt')
log_melting=other.filename('/home/dendenmushi/cansat2023/sequence/log/meltinglog','txt')
log_para=other.filename('/home/dendenmushi/cansat2023/sequence/log/para_avoid_log','txt')
# log_gpsrunning1=other.filename('/home/dendenmushi/cansat2023/sequence/log/gpsrunning1log','txt')
# log_humandetect=other.filename('/home/dendenmushi/cansat2023/sequence/log/humandetectlog','txt')
# log_gpsrunning2=other.filename('/home/dendenmushi/cansat2023/sequence/log/gpsrunning2log','txt')

if __name__  == "__main__":
    ###----------set up -----------###
    t_start=time.time()
    ###-------release judge -------###
    print("START: Release judge")
    other.log(log_phase,'2',"release phase",datetime.datetime.now(),time.time()-t_start)
    #phase=other.phase(log_phase)
    thd_press_release = 0.15
    # pressreleasecount = 0
    # pressreleasejudge = 0
    t_delta_release = 0.6

    #タイムアウトを10分に設定
    timeout_release = time.time()+(5*60)
    
    bme280.bme280_setup()
    bme280.bme280_calib_param()
    # press_d = 0

    #while True:
    while time.time() < timeout_release:
        press_count_release, press_judge_release = release.pressdetect_release(thd_press_release, t_delta_release)
        print(f'count:{press_count_release}\tjudge:{press_judge_release}')
        other.log(log_release, datetime.datetime.now(), time.time() - t_start,
                          bme280.bme280_read(), press_count_release, press_judge_release)
        if press_count_release  >= 2:
            print('Release')
            send.send_data("TXDU 0001.A001")
            break
        else:
            print('unfulfilled')

    print("release finish!!!")
    send.send_data("TXDU 0001.AAAA")

    ###-------land judge -------###
    print("START: Land judge")
    other.log(log_phase,'3',"land phase",datetime.datetime.now(),time.time()-t_start)
    #phase=other.phase(log_phase)
    send.send_data("TXDU 0001,B000")

    #bme280.bme280_setup()
    #bme280.bme280_calib_param()

    landcount = 0
    pressdata = [0.0, 0.0, 0.0, 0.0]

    #タイムアウトを10分に設定
    timeout_land = time.time() + (3*60)

    #while True:
    while time.time() < timeout_land:
        presslandjudge = 0
        landcount, presslandjudge, delta_p, Prevpress, latestpress = land.pressdetect_land(0.1)
        print(f'count:{landcount}\tjudge:{presslandjudge}')
        other.log(log_landing, datetime.datetime.now(), time.time() - t_start,
                           bme280.bme280_read())
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
    other.log(log_phase,'4',"melt phase",datetime.datetime.now(),time.time()-t_start)
    #phase=other.phase(log_phase)
    pi = pigpio.pi()

    meltPin = 4
    other.log(log_melting, datetime.datetime.now(), time.time() - t_start,  "Melting Start")
    try:
        melt.down()
        send.send_data("TXDU 0001,C001")
    except:
        pi.write(meltPin, 0)
    print("melt finish!!!")
    other.log(log_melting, datetime.datetime.now(), time.time() - t_start,  "Melting Finished")
    send.send_data("TXDU 0001,CCCC")
    ###------paraavo-------###
    # try:
    #     motor.setup()

    #     print("START: Parachute avoidance")
    #     other.log(log_phase,'5',"Paraavo phase",datetime.datetime.now(),time.time()-t_start)

    #     flug, area, gap, photoname = paradetection.para_detection("photostorage/photostorage_paradete/para", 320, 240,
    #                                                               200, 10, 120, 1)
    #     print(f'flug:{flug}\tarea:{area}\tgap:{gap}\tphotoname:{photoname}')
    #     print("paradetection phase success")
    #     count_paraavo = 0
    #     while count_paraavo < 3:
    #         flug, area, gap, photoname = paradetection.para_detection("photostorage/photostorage_paradete/para", 320,
    #                                                                   240, 200, 10, 120, 1)
    #         print(f'flug:{flug}\tarea:{area}\tgap:{gap}\tphotoname:{photoname}')
    #         other.log(log_paraavoidance, datetime.datetime.now(), time.time() -
    #                   t_start, flug, area, gap, photoname)
    #         parachute_avoid.parachute_avoidance(flug, gap)
    #         print(flug)
    #         if flug == -1 or flug == 0:
    #             count_paraavo += 1
    #             print(count_paraavo)

    #     print("パラシュート回避完了")

    # except KeyboardInterrupt:
    #     print("emergency!")

    # except:
    #     print(traceback.format_exc())
        
    # print("paraavo finish!!!")
    # send.send_data("TXDU 0001,DDDD")

    #-----上ジャッジ-----#
    motor.setup()
    stuck2.ue_jug()

    #-----スタビの復元まち-----#
    time.sleep(15)

    #-----praschute avoid-----#
    other.log(log_para, datetime.datetime.now(), "Parachute avoidance Start")
    red_area, angle = para_avoid.detect_para()
    para_avoid.para_avoid(red_area, angle, check_count=5)
    other.log(log_para, datetime.datetime.now(), "Parachute avoidance Finish")


