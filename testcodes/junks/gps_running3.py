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
#---------------方向調整---------------#

    theta = angle_goal(magx_off, magy_off, lon2, lat2)

    print('ゴールとの角度theta = ' + str(theta) + '---回転調整開始！')
    stuck2.ue_jug()
    
    base_pwr = 20
    pwr_1 = base_pwr + 20
    pwr_2 = base_pwr + 40

    an = base_pwr

    t_short = 0.1
    t_middle = 0.2
    t_long = 0.4

    count_adjust_direction = 1

    while 45 < theta <= 180 or -180 < theta < -45:
        print("adjust_direction" + str(count_adjust_direction))
        if 90 < theta <= 180 :
            motor.move(an, -an, t_long)
        elif -180 < theta < -90:
            motor.move(-an, an, t_long)
        elif 45 <= theta <= 90:
            motor.move(an, -an, t_middle)
        elif -90 <= theta <= -45:
            motor.move(-an, an, t_middle)
        elif 15 <= theta <= 45:
            motor.move(an, -an, t_short)
        elif -45 <= theta <= -15:
            motor.move(-an, an, t_short)
        
        theta = angle_goal(magx_off, magy_off, lon2, lat2)

        count_adjust_direction += 1

        if count_adjust_direction >= 5 and count_adjust_direction < 10:
            an = pwr_1
        elif count_adjust_direction >= 10:
            an = pwr_2

        #print('Calculated angle_relative: {theta}')
        print('-----調整後の相対角度-----')
        print(f'theta = {theta}')
        time.sleep(0.03)

    print("-----角度調整終了。お疲れ様でした。------")


if __name__ == '__main__':
    
    #生協入口
    #lat2 = 35.91818718
    #lon2 = 139.90814829

    #12号館前
    #lat2 = 35.91896917
    #lon2 = 139.90859362

    #グランドのゴール前
    #lat2 = 35.923914
    #lon2 = 139.912223

    #狭いグランドのほう
    #lat2 = 35.9243874
    #lon2 = 139.9114187

    #中庭の芝生
    #lat2 = 35.91817415
    #lon2 = 139.90825559

    lat2 = 35.9189778
    lon2 = 139.9071493 

    gps.open_gps()
    bmx055.bmx055_setup()
    motor.setup()