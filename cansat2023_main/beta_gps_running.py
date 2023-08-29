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

def angle_goal(magx_off, magy_off, lon2, lat2):
    """
    ゴールとの相対角度を算出する関数

    -180~180度
    """
    magdata = bmx055.mag_dataRead()
    mag_x = magdata[0]
    mag_y = magdata[1]
    theta = calibration.angle(mag_x, mag_y, magx_off, magy_off)
    direction = calibration.calculate_direction(lon2, lat2)
    azimuth = direction["azimuth1"]
    angle_relative = azimuth - theta
    if angle_relative >= 0:
        angle_relative = angle_relative if angle_relative <= 180 else angle_relative - 360
    else:
        angle_relative = angle_relative if angle_relative >= -180 else angle_relative + 360
    return angle_relative


def adjust_direction(theta, magx_off, magy_off, lon2, lat2):
    """
    方向調整
    """

    theta = angle_goal(magx_off, magy_off, lon2, lat2)

    print('ゴールとの角度theta = ' + str(theta) + '---回転調整開始！')
    stuck2.ue_jug()
    an = 25
    t_short = 0.1
    t_middle = 0.2
    t_long = 0.4

    while 45 < theta <= 180 or -180 < theta < -45:
        if 90 < theta <= 180 :
            motor.move(an, -an, t_middle)
        elif -180 < theta < -90:
            motor.move(-an, an, t_middle)
        elif 45 <= theta <= 90:
            motor.move(an, -an, t_short)
        elif -90 <= theta <= -45:
            motor.move(-an, an, t_short)
        elif 15 <= theta <= 45:
            motor.move(an, -an, t_short)
        elif -45 <= theta <= -15:
            motor.move(-an, an, t_short)
        
        theta = angle_goal(magx_off, magy_off, lon2, lat2)

        #print('Calculated angle_relative: {theta}')
        print('Calculated angle_relative')
        print(f'theta = {theta}')
        time.sleep(0.03)

    print("-----adjust_direction finished!!!------")
    send.send_data("TXDU 0001,C2")

def drive(lon2, lat2, thd_distance, t_adj_gps, logpath='/home/dendenmushi/cansat2023/sequence/log/gpsrunningLog.txt', t_start=0):
    """
    GPS走行の関数
    統合する場合はprintをXbee.str_transに変更 other.saveLogのコメントアウトを外す
    """
    direction = calibration.calculate_direction(lon2, lat2)
    goal_distance = direction['distance']
    goal_theta = direction['azimuth1']

    # ------------- 上向き判定 -------------#
    while goal_distance >= thd_distance:
        t_stuck_count = 1
        stuck2.ue_jug()

        # ------------- calibration -------------#
        # xbee.str_trans('calibration Start')
        other.print_im920sl('##--calibration Start--##\n')
        print("------calibration Start------")
        magx_off, magy_off = calibration.cal(30, -30, 30)
        print(f'magx_off: {magx_off}\tmagy_off: {magy_off}\n')
        print("------calibration finished------")

        #-----PID制御による角度調整開始-----#
        #-----初期設定-----#
        rotate_theta_array = []

        #-----積分区間の設定-----#
        rotate_theta_array = test_PID.make_theta_array(rotate_theta_array, 5)

        test_PID.adjust_direction_PID(goal_theta, magx_off, magy_off, rotate_theta_array)
        #-----PID制御による角度調整終了-----#

        t_cal = time.time()
        lat_old, lon_old = gps.location()
        #print("-------gps走行開始-------")
        while time.time() - t_cal <= t_adj_gps:
            print("-------gps走行-------")
            lat1, lon1 = gps.location()
            lat_str = "{:.8f}".format(lat1)  # 緯度を小数点以下8桁に整形
            lon_str = "{:.8f}".format(lon1)  # 経度を小数点以下8桁に整形
            send.send_data("TXDU 0001,F0" + lat_str)
            send.send_data("TXDU 0001,F1" + lon_str)
            # send.send_data("TXDU 0001,F0" + str(lat1) + "0")
            # send.send_data("TXDU 0001,F1" + str(lon1))
            print(lat1, lon1)
            lat_new, lon_new = lat1, lon1
            direction = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
            azimuth, goal_distance = direction["azimuth1"], direction["distance"]
            other.print_im920sl(
                f'lat: {lat1}\tlon: {lon1}\tdistance: {goal_distance}\tazimuth: {azimuth}\n')

            #-----スタックしているかチェック-----#
            if t_stuck_count % 25 == 0:
                ##↑何秒おきにスタックジャッジするかを決める##
                if stuck2.stuck_jug(lat_old, lon_old, lat_new, lon_new, 1):
                    pass
                else:
                    stuck2.stuck_avoid()
                    pass
                lat_old, lon_old = gps.location()

            if goal_distance <= thd_distance:
                break
            else:
                print("ゴールまでの距離は" + str(goal_distance) + "です")
                #-----PID制御による走行開始-----#
                #-----初期設定-----#
                drive_theta_array = []

                #-----積分区間の設定-----#
                drive_theta_array = test_PID.make_theta_array(drive_theta_array, 5)

                for _ in range(25):
                    print("25回ループの部分")
                    #-----現在位置の取得-----#
                    lat1, lon1 = gps.location()
                    direction = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
                    target_theta, goal_distance = direction["azimuth1"], direction["distance"]

                    if goal_distance <= thd_distance:
                        #-----ゴールに到着したら停止-----#
                        break
                    else:
                        test_PID.PID_drive(target_theta, magx_off, magy_off, drive_theta_array, 25)


            t_stuck_count += 1
            other.log(logpath, datetime.datetime.now(), time.time() -
                      t_start, lat1, lon1,  direction['distance'])
            lat_new, lon_new = gps.location()
            print("whileの最下行")

        direction = calibration.calculate_direction(lon2, lat2)
        goal_distance = direction['distance']
        other.print_im920sl(f'-----distance: {goal_distance}-----')

if __name__ == '__main__':
    send.send_data("TXDU 0001,C0")
    # lat2 = 35.918548
    # lon2 = 139.908896
    # lat2 = 35.9234892
    # lon2 = 139.9118744
    #lat2 = 35.9240057
    #lon2 = 139.9114077
    #lat2 = 35.9184282 シダックス
    #lon2 = 139.9111039シダックス
    #lat2 = 35.9240087
    #lon2 = 139.9113212
    
    #生協入口
    #lat2 = 35.91818718
    #lon2 = 139.90814829

    #12号館前
    #lat2 = 35.91896917
    #lon2 = 139.90859362

    #グランドのゴール前
    lat2 = 35.9239389
    lon2 = 139.9122408

    #狭いグランドのほう
    #lat2 = 35.9243874
    #lon2 = 139.9114187

    #中庭の芝生
    #lat2 = 35.91817415
    #lon2 = 139.90825559

    #実験棟の前
    #lat2 = 35.9189778
    #lon2 = 139.9071493 

    gps.open_gps()
    bmx055.bmx055_setup()
    motor.setup()

    drive(lon2, lat2, thd_distance=5, t_adj_gps=40)