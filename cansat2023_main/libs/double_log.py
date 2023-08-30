import time
import datetime
import cv2




import release
import land
import melt
import parachute_avoid
import paradetection
import time
import libs.bme280 as bme280

import libs.send as send
import traceback
import libs.other as other
import wgps_beta_photo_running as photo_running
import libs.save_photo as save_img 
import libs.take as take
import libs.motor as motor
import para_avoid
import libs.stuck2 as stuck

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