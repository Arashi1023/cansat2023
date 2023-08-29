#パラシュート回避
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
import libs.test_PID as test_PID
import libs.log as log
import libs.basics as basics

from main_const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import beta_para_avoid as para_avoid
import wgps_beta_photo_running as photo_running

def detect_para():
    #画像の撮影

    path_all_para = '../imgs/parachute_avoid/all/para_image-'
    path_para_detect = '../imgs/parachute_avoid/detected'
    photoname = take.picture(path_all_para)
    para_img = cv2.imread(photoname)
    angle = 0

    #画像を圧縮
    small_img = photo_running.mosaic(para_img, ratio=0.8)

    #赤色であると認識させる範囲の設定
    mask, masked_img = photo_running.detect_red(small_img)

    #圧縮した画像から重心と輪郭を求めて、画像に反映
    para_img, max_contour, cx, cy = photo_running.get_center(mask, small_img)

    #赤色が占める割合を求める
    red_area = photo_running.get_para_area(max_contour, para_img)

    #重心の位置から現在位置とパラシュートと相対角度を大まかに計算
    angle = photo_running.get_angle(cx, cy, para_img)

    if red_area == 0:
        angle = 0

    #パラシュートが検出された場合に画像を保存
    if red_area != 0:
        red_area = int(red_area)
        save_img(path_para_detect, 'para_detected_', str(red_area), para_img)
    
    return red_area, angle

def para_avoid(check_count, thd_para_avoid=0, thd_para_count=4):

    #-----パラメータの設定-----#
    #-----周囲を確認する-----#
    pwr_check = 25
    t_check = 0.15
    i = 0
    
    #赤色発見=1 赤色未発見=0
    #found parachute
    f_para = 0

    #直進する
    pwr_f = 30
    t_forward = 1

    #読み込み
    red_area, angle = detect_para()

    #パラシュートが覆いかぶさっていたとき用の閾値
    thd_para_covered = 69120

    #-----パラシュートが覆いかぶさっていたとき用の処理-----#
    while red_area > thd_para_covered:
        print("parachute on top")
        time.sleep(5)
        red_area, angle = detect_para()

    #-----初めて取った写真にパラシュートが映っていなかった場合-----#
    if red_area == 0:
        print("パラシュートは前方にありません。")
        print("念のため周囲を確認します。")

        #-----周囲を確認する-----#
        pwr_check = 25
        t_check = 0.15
        i = 0
        
        #赤色発見=1 赤色未発見=0
        #found parachute
        f_para = 0

        #-----右側を確認する-----#
        for r in range(check_count):
            print("右回転" + str(i+1) + "回目")
            motor.move(pwr_check, -pwr_check, t_check)
            red_area, angle = detect_para()
            i += 1

            #-----パラシュートを確認した場合-----#
            if red_area != 0:
                print("パラシュートを発見。")
                f_para = 1
                #-----初期位置に戻す-----#
                print("初期位置に戻します。")
                #右回転した分だけ左回転する。
                # for n in range(i+1):
                #     motor.move(-pwr_check, pwr_check, t_check)
                i += 1
                while i != 0:
                    motor.move(-pwr_check, pwr_check, t_check)
                    i -= 1
                break
            #-----パラシュートを確認できた場合ここでループから抜け出す-----#
        
        #-----パラシュートを確認できなかった場合-----#
        if f_para == 0:
            #直進する前に角度を調整する。2回だけ左回転する・
            print("直進する前に角度を調整します。")
            for a in range(2):
                print("角度調整" + str(a+1) + "回目（左回転）")
                motor.move(-pwr_check, pwr_check, t_check)
    
    else:
        print("パラシュートを前方に発見しました。")
        #-----初めて取った写真にパラシュートが映っていた場合-----#
        while red_area > thd_para_avoid:
            if angle == 1:
                print("右回転します。")
                t_rotate = 0.2
            elif angle == 2:
                print("強く右回転します。")
                t_rotate = 0.3
            elif angle == 3:
                print("左回転します。")
                t_rotate = -0.2

            if t_rotate > 0:
                motor.move(pwr_check, -pwr_check, t_rotate)
            else:
                motor.move(-pwr_check, pwr_check, abs(t_rotate))

            #-----回転後にパラシュートがあるかを確認-----#
            red_area, angle = detect_para()

    #-----パラシュート回避完了-----#

    #パラシュートが前方にないことが確認できたので、直進する。
    print("前方にパラシュートがないことを確認しました。直進します。")
    motor.move(pwr_f, pwr_f, t_forward)
    print("パラシュートは回避できました。")

# def beta_para_avoid(para_thd_covered : int, para_thd_avoid :int, check_count :int):
#     '''
#     パラシュートを回避する関数 0821作成
#     Parameters
#     ----------
#     para_thd_avoid : int
#         赤色の面積がこれ以上大きいとパラシュートがあると判断する
#     check_count : int
#         何回確認するか
#     '''

#     #-----パラメータの設定-----#
#     motor_pwr = 25
#     motor_time = 0.15

#     para_red_area, para_angle = detect_para()

#     #-----パラシュートが覆いかぶさっていた時の処理-----#
#     while para_red_area > para_thd_covered:
#         print("Parachute On Top")
#         time.sleep(20)
#         para_red_area, para_angle = detect_para()

#     #-----初めて撮った写真にパラシュートが映っていなかった場合-----#
#     if para_red_area == 0:
#         print("Parachute Not Found\nCheck Around")

#         for i in range(check_count):
#             print("Check Right " + str(i+1) + "times")
#             motor.move(motor_pwr, -motor_pwr, motor_time)
#             para_red_area, para_angle = detect_para()
#             i += 1
            
#             #-----パラシュートが映っていた場合-----#
#             if para_red_area != 0:
#                 print("Parachute Found\nReturn To Initial Position")
#                 while i != 0:
#                     motor.move(-motor_pwr, motor_pwr, motor_time)
#                     i -= 1
#                 break
    
#     #-----パラシュートが映っていた場合-----#

def wgps_para_avoid(small_thd_dist :int, large_thd_dist :int, check_count :int, thd_para_avoid=0, thd_para_count=4):
    '''
    Parameters
    ----------

    '''
    #-----setup-----#
    count = 0

    stuck2.ue_jug()

    #-----着地地点のGPS座標の取得-----#
    lat_land, lon_land = gps.location()

    para_info = calibration.calculate_direction(lat_land, lon_land)
    para_dist = para_info['distance']

    #-----パラシュートがすぐ近くにあるとき-----#
    while para_dist <= small_thd_dist:
        print("Warning: Parachute is very close\nStarting Parachute Avoid Sequence")
        try:
            para_avoid(check_count, thd_para_avoid, thd_para_count)
        except:
            print("Parachute Avoid Sequence Failed")
            print("Trying Again")

        #-----パラシュートまでの距離を計算-----#
        para_info = calibration.calculate_direction(lat_land, lon_land)
        para_dist = para_info['distance']

        count += 1
    
    #-----パラシュートからsmall_thd_dist以上離れたとき-----#
    #-----キャリブレーション-----#
    print("Starting Calibration")
    magx_off, magy_off = calibration.cal(30, -30, 30)
    #-----パラシュート位置の取得-----#
    direction = calibration.calculate_direction(lon_land, lat_land)
    target_azimuth = direction["azimuth1"]
    
    #-----パラシュートが近くにあるとき-----#
    while small_thd_dist < para_dist <= large_thd_dist:
        print("Parachute is near\nGetting away from parachute")

        #-----PID制御による角度調整（パラシュートがある方向に向かせる）-----#
        theta_array = []
        test_PID.make_theta_array(theta_array, 5)
        test_PID.PID_adjust_direction(target_azimuth, magx_off, magy_off, theta_array)

        rotate_count = 2

        while True:
            #-----写真を撮影してパラシュートの位置を確認する-----#
            red_area, angle = detect_para()
            if red_area != 0 and angle == 2:
                magdata = bmx055.mag_dataRead()
                para_mag_x, para_mag_y = magdata[0], magdata[1]
                para_angle = calibration.angle(para_mag_x, para_mag_y, magx_off, magy_off)
                break
            else:
                if angle == 1:
                    print('Rotating Left')
                    azimuth_nxt = target_azimuth - 15
                elif angle == 3:
                    print('Rotating Right')
                    azimuth_nxt = target_azimuth + 15
                else:
                    print('Parachute Not Found\nChecking Around')
                    azimuth_nxt = target_azimuth + 15*rotate_count
                    rotate_count += 1

            #-----機体の回転-----#
            basics.standarize_angle(azimuth_nxt)
            theta_array = []
            test_PID.make_theta_array(theta_array, 5)
            test_PID.PID_adjust_direction(azimuth_nxt, magx_off, magy_off, theta_array)
        
    
        #-----パラシュートから離れる-----#
        print("Getting away from Parachute")
        target_azimuth = para_angle + 120
        target_azimuth = basics.standarize_angle(target_azimuth)

        T_FORWARD = 5

        t_start_run = time.time()
        theta_array = []
        test_PID.make_theta_array(theta_array, 5)

        while time.time() - t_start_run <= T_FORWARD:
            test_PID.PID_run(target_azimuth, magx_off, magy_off, theta_array, loop_num=25)

if __name__ == '__main__':
    # パラメータ
    PARA_THD_COVERED = 69120
    PARA_CHECK_COUNT = 5
    PARA_THD_AVOID = 0

    #セットアップ
    motor.setup()

    # red_area, angle = detect_para()
    # para_avoid(red_area, angle, check_count=5)
    wgps_para_avoid(para_thd_covered=PARA_THD_COVERED, para_thd_avoid=PARA_THD_AVOID, check_count=PARA_CHECK_COUNT)
