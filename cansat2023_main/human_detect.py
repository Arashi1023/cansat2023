'''
人検知用のプログラム
田口作成 by 2023/8/31

'''
from math import sqrt
import gps
import gps_navigate
import PID
from machine_learning import DetectPeople
import take
import motor
import bmx055
import calibration
from collections import deque
from main_const import *


def get_locations(lat_human, lon_human):
    '''
    人の位置情報をもとに周囲の4つの点の座標を求める
    Parameters
    ----------
    lat_human : float
        遭難者の緯度
    lon_human : float
        遭難者の経度
    '''

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

    area_info = {
        0: (lon_human, lat_human),    
        1: (lon_n, lat_n),
        2: (lon_e, lat_e),
        3: (lon_s, lat_s),
        4: (lon_w, lat_w),
        5: (lon_w, lat_n),
        6: (lon_e, lat_n),
        7: (lon_e, lat_s),
        8: (lon_w, lat_s),
    }

    return area_info

def main(lat_human, lon_human, model, judge_count, area_count, rotate_count, add_pwr):
    '''
    人の位置情報をもとに周囲を捜索するプログラム
    Parameters
    ----------
    lat_human : float
        遭難者の緯度
    lon_human : float
        遭難者の経度
    model : 人検知用のモデル
        学習済みのモデル
    '''
    #-初期設定-#
    isHuman = 0

    ###---捜索する場所の決定---###
    area_info = get_locations(lat_human, lon_human)
    lat_search, lon_serch = area_info[area_count]

    ###---撮影した画像に人がいる確率を求める---###
    img_path = take.picture('../imgs/human_detect/image', 320, 240)
    result = model.predict(image_path=img_path)

    if result > JUDGE_PROBABILITY:
        if judge_count < ADDITIONAL_JUDGE_COUNT:
            judge_count += 1
            print('Take Another Picture')
        else:
            print('Found a Missing Person')
            isHuman = 1
    else: #人がいないとき回転する
        judge_count = 0
        print(('Rotate'))
        rotate_count += 1
        motor.move(strength_l=HD_ROT_PWR+add_pwr, strength_r=-(HD_ROT_PWR+add_pwr), t_moving=HD_ROT_TIME)

    if rotate_count > ROTATE_COUNT  and judge_count ==0: #24回に1回次の場所に向かう
        rotate_count = 0 #回転回数の初期化
        area_count += 1
        if area_count <= 8:
            print('Move to next area')
            lat_search, lon_serch = area_info[area_count]
            PID.drive2(lat_search, lon_serch, thd_distance=5, t_cal=60, loop_num=20)
            magx_off, magy_off = calibration.cal(30, -30, 30)

    return result, judge_count, area_count, rotate_count, isHuman, magx_off, magy_off

if __name__ == '__main__':

    gps.open_gps()

    ###---人検知用のモデルの読み込み---###
    ML_people = DetectPeople('model_mobile.tflite')

    ###---変数定義---###
    result = 0
    area_count = 0
    rotate_count = 0
    isHuman = 0
    judge_count = 0
    stuck_check_array = deque([0]*6, maxlen=6)
    add_pwr = 0
    add_count = 0

    # magx_off, magy_off = calibration.cal(30, -30, 30)

    while True:
        ###---回転場所の整地---###
        if rotate_count == 0: #ある地点で1枚目の写真を撮影するとき
            magx_off_stuck, magy_off_stuck = calibration.cal(30, -30, 30)
            stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化
            add_pwr = 0 #捜索地点を変えたら追加のパワーをリセット

        ###---現在のローバーの方位角を求める---###
        magdata = bmx055.mag_dataRead()
        magx, magy = magdata[0], magdata[1]
        rover_aziimuth = calibration.angle(magx=magx, magy=magy, magxoff=magx_off_stuck, magyoff=magy_off_stuck)
        stuck_check_array.append(rover_aziimuth)

        if add_pwr != 0 and stuck_check_array[3] != 0: #追加のパワーがあるとき
            for i in range(3):
                expect_azimuth_add = stuck_check_array[i] + 30
                if expect_azimuth_add >= 360:
                    expect_azimuth_add = expect_azimuth_add % 360
                if stuck_check_array[i+1] - expect_azimuth_add > 30: #add_pwrを追加していて回りすぎているとき
                    add_count += 1
                else:
                    add_count = 0
            if add_count == 3:
                add_pwr = 0
                add_count = 0

        if stuck_check_array[5] != 0: #スタックチェックを判定できるデータがそろったとき
            expect_azimuth = stuck_check_array[0] + 90

            if expect_azimuth >= 360:
                expect_azimuth = expect_azimuth % 360

            if stuck_check_array[5] - expect_azimuth < 0: #本来回っているはずの角度を下回っているとき
                print('Rotation Stuck Detected')
                add_pwr = 5
                stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化

        result, judge_count, area_count, rotate_count, isHuman = main(lat_human=LAT_HUMAN, lon_human=LON_HUMAN, model=ML_people, judge_count=judge_count, area_count=area_count, rotate_count=rotate_count, add_pwr=add_pwr)
        print('result:', result)
        if isHuman == 1:
            print('Found a Missing Person')
            break
        if area_count == 9:
            print('Could Not Find a Missin Person')
            print('Mission Failed')
            break



    
    

