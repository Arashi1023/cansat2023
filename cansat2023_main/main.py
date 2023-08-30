'''
田口作成 2023/8/29
'''


import datetime
import time
import sys
import cv2
import pigpio
import traceback
from math import sqrt
import numpy as np

import libs.bme280 as bme280
import libs.bmx055 as bmx055
import libs.motor as motor
import libs.save_photo as save_photo
import libs.send as send
import libs.gps as gps
import libs.gps_navigate as gps_navigate
import libs.stuck2 as stuck2
import libs.other as other
import libs.send_photo as send_photo
import libs.take as take
from libs.machine_learning import DetectPeople
import libs.calibration as calibration
import libs.PID as PID
import libs.log as log
import libs.basics as basics

from main_const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import para_avoid
import wgps_beta_photo_running as imgguide

#####=====clock setup=====#####
t_start = time.time()

#####=====machine set up=====#####
print('Start machine set up')
gps.open_gps()
bmx055.bmx055_setup()
bme280.bme280_setup()
bme280.bme280_calib_param() #これなに？？？

#####=====log setup=====#####
phase_log = log.Logger(dir='../logs/0_phase_log', filename='phase', t_start=t_start)
report_log = log.Logger(dir='../logs/0_report_log', filename='report', t_start=t_start)
release_log = log.Logger(dir='../logs/1_release_log', filename='release', t_start=t_start)
land_log = log.Logger(dir='../logs/2_land_log', filename='land', t_start=t_start)
melt_log = log.Logger(dir='../logs/3_melt_log', filename='melt', t_start=t_start)
para_avoid_log = log.Logger(dir='../logs/4_para_avoid_log', filename='para_avoid', t_start=t_start)
gps_running_human_log = log.Logger(dir='../logs/5_gps_running_human_log', filename='gps_running_human', t_start=t_start)
human_detection_log = log.Logger(dir='../logs/6_human_detection_log', filename='human_detection', t_start=t_start)
gps_running_goal_log = log.Logger(dir='../logs/7_gps_running_goal_log', filename='gps_running_goal', t_start=t_start)
image_guide_log = log.Logger(dir='../logs/8_image_guide_log', filename='image_guide', t_start=t_start)

#####=====Mission Sequence=====#####


#####===== 1 Release Detect Sequence=====#####
print('#####-----Release Detect Sequence: Start-----#####')
release_log.save_log('Release Detect Start')


#-Release Detect-#
press_release_count = 0
press_array = [0]*2

while True:
    try:
        latest_press, delta_press, press_release_count, isRelease = release.release_main(press_release_count=press_release_count, press_array=press_array)
        #-Log-#
        release_log.save_log(latest_press, delta_press, press_release_count, isRelease)
        if isRelease == 1:
            break
    except:
        print('Error\nTrying again...')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('1', 'Release Detect Sequence: Start', lat_log, lon_log)
release_log.save_log('Release Detected')

#-send-#
print('Sending Data...')
send.send_data('Release finished')
time.sleep(10)

print('#####-----Release Detect Sequence: End-----#####')






#####===== 2 Land Detect Sequence=====#####
print('#####-----Land Detect Sequence: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location() #GPS情報取得できるのか？？
phase_log.save_log('2', 'Land Detect Sequence: Start', lat_log, lon_log) #GPS情報取得できるのか？？

#-Land Detect-#
press_land_count = 0
press_array = [0]*2

while True:
    try:
        latest_press, delta_press, press_land_count, isLand = land.land_main(press_land_count=press_land_count, press_array=press_array)
        #-Log-#
        land_log.save_log(latest_press, delta_press, press_land_count, isLand)
        if isLand == 1:
            break
    except:
        print('Error\nTrying again...')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('2', 'Land Detect Sequence: End', lat_log, lon_log) #GPS情報取得できるのか？？

#-send-#
print('Sending Data...')
send.send_data('Land finished')
time.sleep(10)

print('#####-----Land Detect Sequence: End-----#####')





#####===== 3 Melt Sequence=====#####
print('#####-----Melt Sequence: Start-----#####')

#-Log-#
lat_log, lon_log = gps.location()
phase_log.save_log('3', 'Melt Sequence: Start', lat_log, lon_log) #GPS情報取得できるのか？？
melt_log.save_log('Melt Start')

#-Melt-#
melt.melt_down(meltPin=MELT_PIN, t_melt=MELT_TIME)

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('3', 'Melt Sequence: End', lat_log, lon_log) #GPS情報取得できるのか？？
melt_log.save_log('Melt Finished')

#-send-#
print('Sending Data...')
send.send_data('Melt finished')
time.sleep(10)

print('#####-----Melt Sequence: End-----#####')



#====================================================================================================#
#####-----スタビライザーの復元-----#####
print('Waiting for Stabilizer to be restored...')
time.sleep(15)

#####-----GPSの取得チェック-----#####
while True:
    lat_test, lon_test = gps.location()
    print('Waiting for GPS...')
    if lat_test != 0 and lon_test != 0: #0だった場合はGPSが取得できていないので再取得
        print('GPS received')
        break
#====================================================================================================#





#####===== 4 Parachute Avoid Sequence=====#####
print('#####-----Parachute Avoid Sequence: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('4', 'Parachute Avoid Sequence: Start', lat_log, lon_log)

#-Parachute Avoid-#
lat_land, lon_land = lat_log, lon_log #着地地点のGPS座標を取得

stuck2.ue_jug()

check_count = 0 #パラ回避用のカウンター
while True:
    try:
        lat_now, lon_now, para_dist, red_area, angle, isDistant_parachute, check_count = para_avoid.para_avoid_main(lat_land, lon_land, lat_dest=LAT_HUMAN, lon_dest=LON_HUMAN, check_count=check_count)
        
        #-Log-#
        para_avoid_log.save_log(lat_now, lon_now, para_dist, red_area, angle, isDistant_parachute)
        if isDistant_parachute == 1: #パラシュート回避用のフラグ
            break
    except:
        print('Error\nTrying again...')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('4', 'Parachute Avoid Sequence: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
send.send_data('Parachute Avoid finished')
time.sleep(10)

print('#####-----Parachute Avoid Sequence: End-----#####')





#####===== 5 GPS Running Sequence to Human =====#####
print('#####-----GPS Running Sequence to Human: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('5', 'GPS Running Sequence to Human: Start', lat_log, lon_log)

#-GPS Running1-#
while True: #1ループおおよそT_CAL秒
    try:
        direction = calibration.calculate_direction(lon_dest=LON_HUMAN, lat_dest=LAT_HUMAN)
        distance_to_goal = direction["distance"]

        #-T_CALごとに以下の情報を取得-#
        lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_HUMAN, lat_dest=LAT_HUMAN, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)

        #-Log-#
        gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)    
        
        if isReach_dest == 1: #ゴール判定
            break
    except:
        print('Error\nTrying again...')

print(f'{distance_to_dest}m to Goal')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('5', 'GPS Running Sequence to Human: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
send.send_data('Run1 finished')
time.sleep(10)

print('#####-----GPS Running Sequence to Human: End-----#####')





#####===== 6 Human Detection Sequence=====#####
print('#####-----Human Detection Sequence: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('6', 'Human Detection Sequence: Start', lat_log, lon_log)

#-Human Detection-#




































#####===== 7 GPS Running Sequence to Goal=====#####
print('#####-----GPS Running Sequence to Goal: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('7', 'GPS Running Sequence to Goal: Start', lat_log, lon_log)

#-GPS Running2-#
while True: #1ループおおよそT_CAL秒
    try:
        direction = calibration.calculate_direction(lon_dest=LON_GOAL, lat_dest=LAT_GOAL)
        distance_to_goal = direction["distance"]

        #-T_CALごとに以下の情報を取得-#
        lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_GOAL, lat_dest=LAT_GOAL, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)

        #-Log-#
        gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)    
        
        if isReach_dest == 1: #ゴール判定
            break
    except:
        print('Error\nTrying again...')

print(f'{distance_to_dest}m to Goal')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('7', 'GPS Running Sequence to Goal: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
send.send_data('Run2 finished')
time.sleep(10)

print('#####-----GPS Running Sequence to Goal: End-----#####')





#####===== 8 Image Guide Sequence=====#####
print('#####-----Image Guide Sequence: Start-----#####')

#-Log-#
lat_log, lon_log = gps.location()
phase_log.save_log('8', 'Image Guide Sequence', lat_log, lon_log)
image_guide_log.save_log('Image Guide Sequence: Start')

#-Image Guide Drive-#
while True:
    try:
        lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal = imgguide.img_guide_drive(lat_dest=LAT_GOAL, lon_dest=LON_GOAL, thd_distance_goal=THD_DISTANCE_GOAL, thd_red_area=THD_RED_RATIO)
        image_guide_log.save_log(lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal)
        if isReach_goal == 1: #ゴール判定
            break
    except:
        print('Error\nTrying again...')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('8', 'Image Guide Sequence: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
send.send_data('Image Guide finished')
time.sleep(10)

print('#####-----Image Guide Sequence: End-----#####')





#####=====Mission End=====#####
print('Mission Accomlished')
send.send_data('Mission Accompished')
time.sleep(10)

#-Log-#
print('Saving Log...')
last_lat, last_lon = gps.location()
phase_log.save_log('10', 'All Phase Comleted', last_lat, last_lon)
release_log.save_log('(PDT)', 'N', last_lat, 'W', last_lon)

print("Log saved\nEnding Program")