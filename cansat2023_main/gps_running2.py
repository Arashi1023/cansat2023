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

    # print('ゴールとの角度theta = ' + str(theta) + '---回転調整開始！')
    # stuck.ue_jug()
    # an = 30
    # if 45 < theta <= 180 :
    #     motor.motor_continue(an, -an)
    # elif -180 < theta < -45:
    #     motor.motor_continue(-an, an)
    # elif 0 <= theta <= 45:
    #     motor.deceleration(an, -an)
    # elif -45 <= theta <= 0:
    #     motor.motor_continue(-an, an)

    # theta = angle_goal(magx_off, magy_off, lon2, lat2)
    # print('Calculated angle_relative: {theta}')
    # time.sleep(0.03)

    stuck_count = 1
    t_small = 0.1
    t_big = 0.2
    force = 25
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
            motor.move(force, force, t_small)

        elif 60 < theta <= 180:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver2 (stuck:{stuck_count})')
            motor.move(force, force, t_big)

        elif -60 <= theta <= -30:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver3 (stuck:{stuck_count})')
            motor.move(-force, -force, t_small)
        elif -180 < theta < -60:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver4 (stuck:{stuck_count})')
            motor.move(-force, -force, t_big)
        else:
            print(f'theta = {theta}')

        stuck_count += 1
        stuck2.ue_jug()
        theta = angle_goal(magx_off, magy_off, lon2, lat2)
        print('Calculated angle_relative: {theta}')
        time.sleep(1)
    other.print_im920sl(f'theta = {theta} \t rotation finished!!!')


def drive(lon2, lat2, thd_distance, t_adj_gps, logpath='/home/cansat2022/CANSAT2022/log/gpsrunningLog', t_start=0):
    """
    GPS走行の関数
    統合する場合はprintをXbee.str_transに変更，other.saveLogのコメントアウトを外す
    """
    direction = calibration.calculate_direction(lon2, lat2)
    goal_distance = direction['distance']
    while goal_distance >= thd_distance:
        t_stuck_count = 1
        stuck2.ue_jug()

        # ------------- calibration -------------#
        # xbee.str_trans('calibration Start')
        other.print_im920sl('##--calibration Start--##\n')
        magx_off, magy_off = calibration.cal(40, 40, 30)
        print(f'magx_off: {magx_off}\tmagy_off: {magy_off}\n')

        theta = angle_goal(magx_off, magy_off, lon2, lat2)
        adjust_direction(theta, magx_off, magy_off, lon2, lat2)

        t_cal = time.time()
        lat_old, lon_old = gps.location()
        while time.time() - t_cal <= t_adj_gps:
            lat1, lon1 = gps.location()
            lat_new, lon_new = lat1, lon1
            direction = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
            azimuth, goal_distance = direction["azimuth1"], direction["distance"]
            other.print_im920sl(
                f'lat: {lat1}\tlon: {lon1}\tdistance: {goal_distance}\tazimuth: {azimuth}\n')

            if t_stuck_count % 8 == 0:
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
                for _ in range(25):
                    magdata = bmx055.mag_dataRead()
                    mag_x = magdata[0]
                    mag_y = magdata[1]

                    theta = calibration.angle(mag_x, mag_y, magx_off, magy_off)
                    angle_relative = azimuth - theta
                    if angle_relative >= 0:
                        angle_relative = angle_relative if angle_relative <= 180 else angle_relative - 360
                    else:
                        angle_relative = angle_relative if angle_relative >= -180 else angle_relative + 360
                    theta = angle_relative
                    adj_r = 0
                    # if theta >= 0:
                    #     if theta <= 8:
                    #         adj = 0
                    #     elif theta <= 15:
                    #         adj = 5
                    #     elif theta <= 90:
                    #         adj = 20
                    #         adj_r = 5
                    #     else:
                    #         adj = 30
                    #         adj_r = 5
                    # else:
                    #     if theta >= - 8:
                    #         adj = 0
                    #     elif theta >= -15:
                    #         adj = -10
                    #     elif theta >= -90:
                    #         adj = -20
                    #     else:
                    #         adj = -30
                    if theta >= 0:
                        if theta <= 15:
                            adj = 0
                        elif theta <= 90:
                            adj = 20
                            adj_r = 5
                        else:
                            adj = 30
                            adj_r = 5
                    else:
                        if theta >= -15:
                            adj = 0
                        elif theta >= -90:
                            adj = -20
                        else:
                            adj = -30
                    print(f'angle ----- {theta}')
                    strength_l, strength_r = 60 + adj, -70 - adj
                    motor.motor_continue(strength_l, strength_r)
                    time.sleep(0.04)
            t_stuck_count += 1
            other.log(logpath, datetime.datetime.now(), time.time() -
                      t_start, lat1, lon1, direction['distance'], angle_relative)
            motor.deceleration(strength_l, strength_r)
            time.sleep(2)
            lat_new, lon_new = gps.location()

        direction = calibration.calculate_direction(lon2, lat2)
        goal_distance = direction['distance']
        other.print_im920sl(f'-----distance: {goal_distance}-----')


if __name__ == '__main__':
    # lat2 = 35.918548
    # lon2 = 139.908896
    # lat2 = 35.9234892
    # lon2 = 139.9118744
    #lat2 = 35.9240057
    #lon2 = 139.9114077
    #lat2 = 35.9184282 シダックス
    #lon2 = 139.9111039シダックス
    lat2 = 35.9240087
    lon2 = 139.9113212
    gps.open_gps()
    bmx055.bmx055_setup()
    motor.setup()

    drive(lon2, lat2, thd_distance=10, t_adj_gps=10)
