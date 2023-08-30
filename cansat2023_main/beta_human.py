import datetime
import time
import sys
import cv2
import pigpio
import traceback
from math import sqrt

import libs.bme280 as bme280
import libs.bmx055 as bmx055
import libs.motor as motor
import libs.save_photo as save_img
import libs.send as send
import libs.gps as gps
import libs.stuck2 as stuck2
import libs.other as other
import libs.send_photo as send_photo
import libs.take as take
from libs.machine_learning import DetectPeople
import libs.calibration as calibration
import libs.PID as PID
import libs.log as log

from main_const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import para_avoid
import wgps_beta_photo_running as imgguide

log_humandetect=other.filename('/home/dendenmushi/cansat2023/sequence/log/humandetectlog/humandetectlog','txt')


#グローバル変数として宣言
# global human_judge_count
# global break_outer_loop
# human_judge_count = 0
# break_outer_loop = False

motor.setup()
gps.open_gps()
bmx055.bmx055_setup()
'''
#chatGPTさんより青点の設定
#lon1,lat1は赤点の位置
def calculate_square_corners(lon1, lat1):
    # 1度あたりの緯度経度の差（おおよその値）
    lat_diff_per_meter = 0.000009
    lon_diff_per_meter = 0.000011

    # 正方形の一辺の長さ（メートル）
    square_side_length = 20

    # 正方形の角の座標を計算
    lon_n = lon1 + (square_side_length / 2) * lon_diff_per_meter
    lat_n = lat1 - (square_side_length / 2) * lat_diff_per_meter

    lon_e = lon1 + (square_side_length / 2) * lon_diff_per_meter
    lat_e = lat1 + (square_side_length / 2) * lat_diff_per_meter

    lon_s = lon1 - (square_side_length / 2) * lon_diff_per_meter
    lat_s = lat1 - (square_side_length / 2) * lat_diff_per_meter

    lon_w = lon1 - (square_side_length / 2) * lon_diff_per_meter
    lat_w = lat1 + (square_side_length / 2) * lat_diff_per_meter

    return lat_n, lon_n, lat_e, lon_e, lat_s, lon_s, lat_w, lon_w
'''


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


    #for i in range(6):
    for i in range(24):
        elapsed_time = time.time()-start_time
        if break_outer_loop == False:
            motor.move(25, -25, 0.15)
            human_judge_count = 0
            # 撮影
            img_path = take.picture('ML_imgs/image', 320, 240)

            # モデルの読み込み
            #result = ML_people.predict(image_path=img_path)
            result = model.predict(image_path=img_path)
            other.log(logpath, datetime.datetime.now(), time.time() -
                      t_start,result,0,human_judge_count,break_outer_loop,elapsed_time)
            # hitoの確率50%かどうか
            if result >= judge_probability:
                human_judge_count += 1
                print(human_judge_count)
                # 追加の写真を撮影
                for j in range(2):
                    additional_img_path = take.picture('ML_imgs/additional_image', 320, 240)
                    #additional_result = ML_people.predict(image_path=additional_img_path)
                    additional_result = model.predict(image_path=additional_img_path)
                    other.log(logpath, datetime.datetime.now(), time.time() -
                      t_start,result,additional_result,human_judge_count,break_outer_loop,elapsed_time)
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
                if elapsed_time >= threshold:  # 20分経ったか
                    break_outer_loop = True
                    break
                else:
                    print("捜索続けます")
            #motor.move(30, -30, 0.2)  # 芝生の上
            #motor.move(30, -30, 0.15)  #グランド
        else:
            break
    if break_outer_loop == False:
        print("24回撮影しました")
        print("次のエリアに移動します")
    return human_judge_count , break_outer_loop

    
def move_to_bulearea(count, lat_human, lon_human):
 
    # data_dist_bulearea1 =gps_navigate.vincenty_inverse(lat_now,lon_now,lat_n,lon_n)
    # data_dist_bulearea2 =gps_navigate.vincenty_inverse(lat_now,lon_now,lat_e,lon_e)
    # data_dist_bulearea3 =gps_navigate.vincenty_inverse(lat_now,lon_now,lat_s,lon_s)
    # data_dist_bulearea4 =gps_navigate.vincenty_inverse(lat_now,lon_now,lat_w,lon_w)

    
    blue_loc = get_locations(lat_human, lon_human)
    lat_n = blue_loc['lat_n']
    lon_n = blue_loc['lon_n']
    lat_e = blue_loc['lat_e']
    lon_e = blue_loc['lon_e']
    lat_s = blue_loc['lat_s']
    lon_s = blue_loc['lon_s']
    lat_w = blue_loc['lat_w']
    lon_w = blue_loc['lon_w']


    print(count)

    if count == 1:
        PID.drive(lon_n, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第1エリアです")
    elif count == 2:
        PID.drive(lon_e, lat_e, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start) 
        print("第2エリアです")  
    elif count == 3:
        PID.drive(lon_s, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第3エリアです")
    elif count == 4:
        PID.drive(lon_w, lat_w, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第4エリアです")
    elif count == 5:
        PID.drive(lon_w, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第5エリアです")
    elif count == 6:
        PID.drive(lon_e, lat_n, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第6エリアです")
    elif count == 7:
        PID.drive(lon_e, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第7エリアです")
    elif count == 8:
        PID.drive(lon_w, lat_s, thd_distance=3, t_run=60, logpath=log_humandetect,t_start=t_start)
        print("第8エリアです")
    else:
        print("青点エリア捜索終了")             
    
if __name__ == "__main__":
    t_start = time.time()
    count = 0
    human_judge_count = 0
    break_outer_loop = False
    judge_probability=0.50
    start_time = time.time()
    threshold = 20*60
    # elapsed_time = time.time()-start_time
    #lat1 = 35.12345 #赤点
    #lon1 = 139.67890 #赤点

    #12号館前
    #lat_human = 35.91896917
    #lon_human = 139.90859362

    #グランドのゴール前
    # lat_human = 35.923914
    # lon_human = 139.912223

    #グランドの中央
    lat_human = 35.9243068
    lon_human = 139.9124594

    #lat_human = 35.9243467
    #lon_human = 139.9113996

    #中庭の芝生
    #lat_human = 35.91817415
    #lon_human = 139.90825559

    #人検知に使用するモデルの読み込み
    # global ML_people
    ML_people = DetectPeople(model_path="model_mobile.tflite" )

    #まずはメインエリアを捜索

    # for k in range(6):
    for k in range(24):
        elapsed_time = time.time()-start_time
        if break_outer_loop == False:
            motor.move(25, -25, 0.15) #グランド
            human_judge_count = 0
            #撮影
            img_path = take.picture('ML_imgs/image', 320, 240)
            
            #モデルの読み込み
            result = ML_people.predict(image_path=img_path)
            other.log(log_humandetect, datetime.datetime.now(), time.time() -
                      t_start,result,0,human_judge_count,break_outer_loop,elapsed_time)
            #hitoの確率50%かどうか
            if result >= judge_probability:
                human_judge_count += 1
                # 追加の写真を撮影
                for h in range(2):
                    additional_img_path = take.picture('ML_imgs/additional_image', 320, 240)
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
                if elapsed_time >= threshold:  # 20分経ったか
                    break_outer_loop = True
                    break
                else:
                    print("捜索続けます")
            #motor.move(35, -35, 0.2) # 芝生の上
            # motor.move(25, -25, 0.15) #グランド
        else:
            break
    if break_outer_loop == False:
        print("24回撮影しました")
        print("次のエリアに移動します")


    if human_judge_count==0:
        print ("青点エリア捜索に移行")
        for j in range(8):
            elapsed_time = time.time()-start_time #経過時間の更新
            if break_outer_loop == True:
                break
            else:
                #lat_now, lon_now = gps.location()
                count += 1
                # move_to_bulearea(count, lat_human, lon_human)
                human_judge_count, break_outer_loop = take_and_rotation(human_judge_count=human_judge_count, break_outer_loop=break_outer_loop,judge_probability=judge_probability,start_time=start_time,logpath=log_humandetect,model=ML_people)
    print("human detection finish!!!")
    


    
