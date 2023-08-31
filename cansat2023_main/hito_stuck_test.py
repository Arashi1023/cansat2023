import datetime
import time
import sys
import cv2
import pigpio
import traceback
from math import sqrt

import bme280
import bmx055
import motor
import save_photo as save_img
import send
import gps
import gps_navigate
import stuck2
import other
import send_photo
import take
from machine_learning import DetectPeople
import calibration
import PID
import log
import basics

from main_const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import para_avoid
import goal_detect
import beta_gps_running as gps_running


# log_humandetect=other.filename('/home/dendenmushi/cansat2023/sequence/log/humandetectlog/humandetectlog','txt')

def get_locations(lat_human, lon_human):
#最後の位置情報をもとに周囲の4つの点の座標を求める

    #北緯40度における10mあたりの緯度経度の差
    #緯度は0.3236246秒　経度は0.3242秒
    #lat_dif = 0.0000323
    #lon_dif = 0.0000324

    lat_dif = 0.0000090
    lon_dif = 0.0000110

    #北緯40度における10mあたりの緯度経度の差
    #lon_dif = 0.0000117
    
    #捜索範囲の四角形の一辺の長さ
    side_length = 40

    #赤点から青点までの距離 red to blue distance
    rtb_distance = (side_length/4)*sqrt(2) 

    #周囲の4つの位置を求める
    #north
    lat_n = lat_human + lat_dif*(rtb_distance)
    lon_n = lon_human
    #east
    lat_e = lat_human
    lon_e = lon_human - lon_dif*(rtb_distance)
    #south
    lat_s = lat_human - lat_dif*(rtb_distance)
    lon_s = lon_human
    #west
    lat_w = lat_human
    lon_w = lon_human + lon_dif*(rtb_distance)

    return {
        'lat_n':lat_n,
        'lon_n':lon_n,
        'lat_e':lat_e,
        'lon_e':lon_e,
        'lat_s':lat_s,
        'lon_s':lon_s,
        'lat_w':lat_w,
        'lon_w':lon_w
        }

def take_and_rotation(human_judge_count, break_outer_loop,judge_probability,start_time,logpath, model):

    for i in range(24):
        elapsed_time = time.time()-start_time
        if break_outer_loop == False:
            motor.move(25, -25, 0.15)
            human_judge_count = 0
            # 撮影
            img_path = take.picture('../imgs/human_detect/all/image-', 320, 240)
            # モデルの読み込み
            result = model.predict(image_path=img_path)
            # other.log(logpath, datetime.datetime.now(), time.time() -
            #           start_time,result,human_judge_count,break_outer_loop,elapsed_time)
            # hitoの確率50%かどうか
            if result >= judge_probability:
                human_judge_count += 1
                print(human_judge_count)
                # 追加の写真を撮影
                for j in range(2):
                    additional_img_path = take.picture('../imgs/human_detect/additional/additional_image-', 320, 240)
                    additional_result = model.predict(image_path=additional_img_path)
                    # other.log(logpath, datetime.datetime.now(), time.time() -
                    #   start_time,additional_result,human_judge_count,break_outer_loop,elapsed_time)
                    if additional_result >= judge_probability:
                        human_judge_count += 1
                        print(human_judge_count)
                        if human_judge_count >= 3:
                            break_outer_loop = True
                            print("遭難者発見")
                            break
                    else:
                        human_judge_count = 0
            else:
                if elapsed_time >= timeout_mission:  # 20分経ったか
                    break_outer_loop = True
                    break
                else:
                    print("捜索続けます")
        else:
            break
    if break_outer_loop == False:
        print("24回撮影しました")
        print("次のエリアに移動します")
    return human_judge_count , break_outer_loop

    
def move_to_bulearea(count, lat_human, lon_human):
 
    
    blue_loc = get_locations(lat_human, lon_human)
    lat_n = blue_loc['lat_n']
    lon_n = blue_loc['lon_n']
    lat_e = blue_loc['lat_e']
    lon_e = blue_loc['lon_e']
    lat_s = blue_loc['lat_s']
    lon_s = blue_loc['lon_s']
    lat_w = blue_loc['lat_w']
    lon_w = blue_loc['lon_w']


    # print(count)
    
    # if count == 1:
    #     PID.drive(lon_n, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第1エリアです")
    # elif count == 2:
    #     PID.drive(lon_e, lat_e, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start) 
    #     print("第2エリアです")  
    # elif count == 3:
    #     PID.drive(lon_s, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第3エリアです")
    # elif count == 4:
    #     PID.drive(lon_w, lat_w, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第4エリアです")
    # elif count == 5:
    #     PID.drive(lon_w, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第5エリアです")
    # elif count == 6:
    #     PID.drive(lon_e, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第6エリアです")
    # elif count == 7:
    #     PID.drive(lon_e, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第7エリアです")
    # elif count == 8:
    #     PID.drive(lon_w, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
    #     print("第8エリアです")
    # else:
    #     print("青点エリア捜索終了")
    # エリア情報の辞書を作成
    area_info = {
        1: (lon_n, lat_n),
        2: (lon_e, lat_e),
        3: (lon_s, lat_s),
        4: (lon_w, lat_w),
        5: (lon_w, lat_n),
        6: (lon_e, lat_n),
        7: (lon_e, lat_s),
        8: (lon_w, lat_s),
    }

    print(count)

    if count in area_info:
        lon, lat = area_info[count]
        # PID.drive(lon, lat, thd_distance=3, t_run=60, logpath=0, t_start=start_time)
        lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=lon, lat_dest=lat, thd_distance=8, t_cal=T_CAL, loop_num=LOOP_NUM)
        print('distance to dest: ', distance_to_dest)
        print(f"第{count}エリアです")
    else:
        print("青点エリア捜索終了")

def detect_main_area(human_judge_count, break_outer_loop,judge_probability,start_time,logpath, model):

    # 初期化時の向きを保存する変数
    initial_azimuth = None
    # 各回転ごとのrover_azimuthを保存する変数
    rover_azimuths = []
    # 各回転ごとの差分を保存する変数
    azimuth_diffs = []
    # 一定の値を超えたときにモーター出力を上げるための閾値
    threshold_diff_sum = 90.0
    #モーターの出力の初期化
    motor_l,motor_r = 25 , -25

    magx_off, magy_off = calibration.cal(30, -30, 40) #整地、オフセット値
    for k in range(24):
        #-----経過時間の更新-----#
        elapsed_time = time.time()-start_time
        #-----スタック判定-----#
        magdata= bmx055.mag_dataRead()
        mag_x, mag_y = magdata[0], magdata[1]

        rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)

        

        # 初期向きが取得された後、毎回rover_azimuthを算出し、差分を計算
        if initial_azimuth is not None:
            if len(rover_azimuths) > 0:
                prev_azimuth = rover_azimuths[-1]
                azimuth_diff = abs(rover_azimuth - prev_azimuth)
                azimuth_diffs.append(azimuth_diff)
            
            rover_azimuths.append(rover_azimuth)
        else:
            initial_azimuth = rover_azimuth
        
        # 6回分の差分が閾値を超えていない場合にモーター出力を上げて初期化
        if len(azimuth_diffs) >= 6 and sum(azimuth_diffs) < threshold_diff_sum:
            print("モーター出力を上げます")
            # モーターの出力を上げるコードを追加
            motor_l+=2
            motor_r-=2
            
            # azimuth_diffsを初期化
            azimuth_diffs = []

        elif len(azimuth_diffs) >= 6:
            # azimuth_diffsを初期化
            azimuth_diffs = []
        
        #-----回転＋人検知-----#
        if break_outer_loop == False:
            motor.move(motor_r, motor_l, 0.15)
            human_judge_count = 0
            #撮影
            img_path = take.picture('../imgs/human_detect/all/image-', 320, 240)
            
            #モデルの読み込み
            result = model.predict(image_path=img_path)
            # other.log(log_humandetect, datetime.datetime.now(), time.time() -
            #           start_time,result,human_judge_count,break_outer_loop,elapsed_time)
            #hitoの確率50%かどうか
            if result >= judge_probability:
                human_judge_count += 1
                print(human_judge_count)
                # 追加の写真を撮影
                for h in range(2):
                    additional_img_path = take.picture('../imgs/human_detect/additional/additional_image', 320, 240)
                    additional_result = model.predict(image_path=additional_img_path)
                    # other.log(logpath, datetime.datetime.now(), time.time() -
                    #   start_time,additional_result,human_judge_count,break_outer_loop,elapsed_time)
                    if additional_result >= judge_probability:
                        human_judge_count += 1
                        print(human_judge_count)
                        if human_judge_count >= 3:
                            break_outer_loop = True
                            print("遭難者発見")
                            break
                    else:
                        human_judge_count = 0
            else:
                if elapsed_time >= timeout_mission:  # 20分経ったか
                    break_outer_loop = True
                    break
                else:
                    print("捜索続けます")
        else:
            break
    if break_outer_loop == False:
        print("24回撮影しました")
        print("次のエリアに移動します")
    
    return human_judge_count , break_outer_loop
    
def human_detect_main():
    '''
    作成 by 田口 2023/8/30
    '''
    



    pass                     
    
if __name__ == "__main__":
    # motor.setup()
    gps.open_gps()
    bmx055.bmx055_setup()



    # t_start = time.time()
    count = 0
    human_judge_count = 0
    break_outer_loop = False
    start_time = time.time()
    timeout_mission = 20 * 60
    judge_probability=0.50

    #グランドの中央
    lat_human = 35.9243068
    lon_human = 139.9124594



    ML_people = DetectPeople(model_path="model_mobile.tflite" )
    log_humandetect=other.filename('../logs/ver2_logs/humandetectlog/humandetectlog','txt')

    hito_log = log.Logger(dir='../Logs/test_logs/hito_test', filename='hito_test', t_start=start_time, columns=[])



    #まずはメインエリアを捜索

    '''
    for k in range(24):
        if break_outer_loop == False:
            human_judge_count = 0
            #撮影
            img_path = take.picture('../imgs/human_detect/all/image-', 320, 240)
            
            #モデルの読み込み
            result = ML_people.predict(image_path=img_path)
            other.log(log_humandetect, datetime.datetime.now(), time.time() -
                      t_start,result,0,human_judge_count,break_outer_loop,elapsed_time)
            #hitoの確率50%かどうか
            if result >= judge_probability:
                human_judge_count += 1
                # 追加の写真を撮影
                for h in range(2):
                    additional_img_path = take.picture('../imgs/human_detect/additional/additional_image', 320, 240)
                    additional_result = ML_people.predict(image_path=additional_img_path)
                    other.log(log_humandetect, datetime.datetime.now(), time.time() -
                      t_start,result,additional_result,human_judge_count,break_outer_loop,elapsed_time)
                    if additional_result >= judge_probability:
                        human_judge_count += 1
                        if human_judge_count >= 3:
                            break_outer_loop = True
                            print("遭難者発見")
                            break
                    else:
                        human_judge_count = 0
            else:
                if elapsed_time >= timeout_mission:  # 20分経ったか
                    break_outer_loop = True
                    break
                else:
                    print("捜索続けます")
            #motor.move(35, -35, 0.2) # 芝生の上
            motor.move(25, -25, 0.15) #グランド
        else:
            break
    if break_outer_loop == False:
        print("24回撮影しました")
        print("次のエリアに移動します")
    '''
    human_judge_count, break_outer_loop = detect_main_area(human_judge_count,break_outer_loop,judge_probability,start_time,log_humandetect,ML_people)
    if human_judge_count==0:
        print ("青点エリア捜索に移行")
        for j in range(8):#8地点について行うよ
            elapsed_time = time.time()-start_time #経過時間の更新
            if break_outer_loop == True:
                break
            else:
                count += 1
                move_to_bulearea(count, lat_human, lon_human)
                human_judge_count, break_outer_loop = take_and_rotation(human_judge_count,break_outer_loop,judge_probability,start_time,log_humandetect,ML_people)
    print("human detection finish!!!")
    


    
