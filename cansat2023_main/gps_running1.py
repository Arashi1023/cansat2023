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
import libs.test_PID as PID
import libs.log as log

from const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import beta_para_avoid as para_avoid
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
    '''
    stuck_count = 1
    t_small = 0.1
    t_big = 0.2
    force = 40
    while 30 < theta <= 180 or -180 < theta < -30:
        if stuck_count >= 16:
            ##方向調整が不可能な場合はスタックしたとみなして、もう一度キャリブレーションからスタート##
            other.print_im920sl(
                "!!!!can't ajdust direction.   start stuck avoid!!!!!")
            stuck2.stuck_avoid()
            magx_off, magy_off = calibration.cal(40, 40, 30)
            stuck_count = -1
        if stuck_count % 7 == 0:
            other.print_im920sl('Increase output')
            force += 10
        if 30 <= theta <= 60:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver1 (stuck:{stuck_count})')
            motor.move(force, -force, t_small)

        elif 60 < theta <= 180:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver2 (stuck:{stuck_count})')
            motor.move(force, -force, t_big)

        elif -60 <= theta <= -30:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver3 (stuck:{stuck_count})')
            motor.move(-force, force, t_small)
        elif -180 < theta < -60:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver4 (stuck:{stuck_count})')
            motor.move(-force, force, t_big)
        else:
            print(f'theta = {theta}')

        stuck_count += 1
        stuck2.ue_jug()
        
    
        print('Calculated angle_relative: {theta}')
        time.sleep(1)
    other.print_im920sl(f'theta = {theta} \t rotation finished!!!')
    '''

def drive(lon2, lat2, thd_distance, t_adj_gps, logpath='/home/dendenmushi/cansat2023/sequence/log/gpsrunningLog.txt', t_start=0):
    """
    GPS走行の関数
    統合する場合はprintをXbee.str_transに変更，other.saveLogのコメントアウトを外す
    """
    direction = calibration.calculate_direction(lon2, lat2)
    goal_distance = direction['distance']

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
        
        theta = angle_goal(magx_off, magy_off, lon2, lat2)
        adjust_direction(theta, magx_off, magy_off, lon2, lat2)
        
        #-----角度のログ出力-----#
        magdata = bmx055.mag_dataRead()
        mag_x = magdata[0]
        mag_y = magdata[1]

        # rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)
        #other.log(logpath, datetime.datetime.now(), time.time() -
                      #t_start, lat1, lon1, rover_azimuth, direction['distance'])
        


        t_cal = time.time()
        lat_old, lon_old = gps.location()

        # other.log(logpath, datetime.datetime.now(), time.time() -
        #               t_start, lat_old, lon_old, rover_azimuth, direction['distance'])

        lat_str = "{:.6f}".format(lat_old)  # 緯度を小数点以下8桁に整形
        lon_str = "{:.6f}".format(lon_old)  # 経度を小数点以下8桁に整形
        send.send_data(lat_str)
        time.sleep(9)
        send.send_data(lon_str)
        time.sleep(9)
        # send.send_data("TXDU 0001,F0" + str(lat1) + "0")
        # send.send_data("TXDU 0001,F1" + str(lon1))
        #print("-------gps走行開始-------")
        while time.time() - t_cal <= t_adj_gps:
            print("-------gps走行-------")
            lat1, lon1 = gps.location()
            # lat_str = "{:.8f}".format(lat1)  # 緯度を小数点以下8桁に整形
            # lon_str = "{:.8f}".format(lon1)  # 経度を小数点以下8桁に整形
            # send.send_data("TXDU 0001,F0" + lat_str)
            # time.sleep(3)
            # send.send_data("TXDU 0001,F1" + lon_str)
            # time.sleep(10)
            # send.send_data("TXDU 0001,F0" + str(lat1) + "0")
            # send.send_data("TXDU 0001,F1" + str(lon1))
            print(lat1, lon1)
            lat_new, lon_new = lat1, lon1
            direction = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
            azimuth, goal_distance = direction["azimuth1"], direction["distance"]
            other.print_im920sl(
                f'lat: {lat1}\tlon: {lon1}\tdistance: {goal_distance}\tazimuth: {azimuth}\n')

            #-----スタックしているかチェックする-----#
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
                for _ in range(25):
                    print("25回ループの部分")
                    magdata = bmx055.mag_dataRead()
                    mag_x = magdata[0]
                    mag_y = magdata[1]

                    print("----------mag_x, mag_yの読み取り値----------")
                    print(mag_x, mag_y)

                    magx, magy, magz = calibration.get_data()
                    print('********calibartion.get_data***********')
                    print(magx,magy,magz)                 
                    rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)
                    angle_relative = azimuth - rover_azimuth

                    if angle_relative >= 0:
                        angle_relative = angle_relative if angle_relative <= 180 else angle_relative - 360
                    else:
                        angle_relative = angle_relative if angle_relative >= -180 else angle_relative + 360
                    theta = angle_relative

                    if theta >= 0:
                        if theta <= 15:
                            pwr_r = 45
                            pwr_l = 40
                        elif theta <= 90:
                            pwr_r = 40
                            pwr_l = 40
                        else:
                            pwr_r = 35
                            pwr_l = 40
                    else:
                        if theta >= -15:
                            pwr_r = 45
                            pwr_l = 40
                        elif theta >= -90:
                            pwr_r = 45
                            pwr_l = 35
                        else:
                            pwr_r = 45
                            pwr_l = 30
                    print(f'angle ----- {theta}')
                    strength_l, strength_r = pwr_l ,pwr_r
                    motor.motor_continue(strength_l, strength_r)
                    time.sleep(0.04)

            t_stuck_count += 1
            #motor.deceleration(strength_l, strength_r)
            #time.sleep(2)
            lat_new, lon_new = gps.location()
            other.log(logpath, datetime.datetime.now(), time.time() - t_start, lat_new, lon_new, rover_azimuth, direction['distance'])
            print("whileの最下行")

        direction = calibration.calculate_direction(lon2, lat2)
        goal_distance = direction['distance']
        motor.motor_stop(1)
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
    # lat2 = 35.9239389
    # lon2 = 139.9122408

    #狭いグランドのほう
    #lat2 = 35.9243874
    #lon2 = 139.9114187

    #中庭の芝生
    lat2 = 35.91817415
    lon2 = 139.90825559

    #実験棟の前
    #lat2 = 35.9189778
    #lon2 = 139.9071493 

    gps.open_gps()
    bmx055.bmx055_setup()
    motor.setup()

    drive(lon2, lat2, thd_distance=5, t_adj_gps=40)

