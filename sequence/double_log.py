import release
import land
import melt
import parachute_avoid
import paradetection
import time
import bme280

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

    press_count_release = 0
    press_judge_release = 0

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