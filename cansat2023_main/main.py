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

import bme280
import bmx055
import motor
import save_photo
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

#####=====clock setup=====#####
t_start = time.time()

#####=====machine set up=====#####
print('Start machine set up')
gps.open_gps()
bmx055.bmx055_setup()
bme280.bme280_setup()
bme280.bme280_calib_param() #これなに？？？

#####=====log setup=====#####
phase_log = log.Logger(dir='../logs/0_phase_log', filename='phase', t_start=t_start, columns=['phase', 'condition', 'lat', 'lon'])
report_log = log.Logger(dir='../logs/0_report_log', filename='report', t_start=t_start, columns=[])
release_log = log.Logger(dir='../logs/1_release_log', filename='release', t_start=t_start, columns=['latest_press', 'delta_press', 'press_release_count', 'isRelease'])
land_log = log.Logger(dir='../logs/2_land_log', filename='land', t_start=t_start, columns=['latest_press', 'delta_press', 'press_land_count', 'isLand'])
melt_log = log.Logger(dir='../logs/3_melt_log', filename='melt', t_start=t_start, columns=['condition'])
para_avoid_log = log.Logger(dir='../logs/4_para_avoid_log', filename='para_avoid', t_start=t_start, columns=['lat', 'lon', 'distance_to_parachute', 'red_area', 'angle', 'isDistant_parachute', 'check_count'])
gps_running_human_log = log.Logger(dir='../logs/5_gps_running_human_log', filename='gps_running_human', t_start=t_start, columns=['lat', 'lon', 'distance_to_human', 'rover_azimuth', 'isReach_human'])
human_detection_log = log.Logger(dir='../logs/6_human_detection_log', filename='human_detection', t_start=t_start, columns=[])
gps_running_goal_log = log.Logger(dir='../logs/7_gps_running_goal_log', filename='gps_running_goal', t_start=t_start, columns=['lat', 'lon', 'distance_to_goal', 'rover_azimuth', 'isReach_goal'])
image_guide_log = log.Logger(dir='../logs/8_image_guide_log', filename='image_guide', t_start=t_start, columns=['lat', 'lon', 'distance_to_goal', 'area_ratio', 'angle', 'isReach_goal'])

#####=====Mission Sequence=====#####


#####===== 1 Release Detect Sequence=====#####
print('#####-----Release Detect Sequence: Start-----#####')
phase_log.save_log('1', 'Release Detect Sequence: Start', 0, 0) #GPS情報取得できるのか？？？
release_log.save_log('Release Detect Start')


#-Release Detect-#
press_release_count = 0
press_array = [0]*2

while True:
    if time.time() - t_start > RELEASE_TIMEOUT: #タイムアウトの設定
        print('Release Sequence Timeout')
        release_log.save_log('Release Sequence Timeout')
        break

    latest_press, delta_press, press_release_count, isRelease = release.release_main(press_release_count=press_release_count, press_array=press_array)
    #-Log-#
    release_log.save_log(latest_press, delta_press, press_release_count, isRelease)
    print('isRelease: ' + str(isRelease))
    if isRelease == 1:
        print('Release Detected')
        break

#-Log-#
print('Saving Log...')
phase_log.save_log('1', 'Release Detect Sequence: Start', 0, 0)
release_log.save_log('Release Detected')

#-send-#
# print('Sending Data...')
# send.send_data('Release finished')
# time.sleep(10)

print('#####-----Release Detect Sequence: End-----#####')






#####===== 2 Land Detect Sequence=====#####
print('#####-----Land Detect Sequence: Start-----#####')

#-Log-#
print('Saving Log...')
phase_log.save_log('2', 'Land Detect Sequence: Start', 0, 0) #GPS情報取得できるのか？？

#-Land Detect-#
press_land_count = 0
press_array = [0]*2

while True:
    if time.time() - t_start > LAND_TIMEOUT: #タイムアウトの設定
        print('Land Sequence Timeout')
        land_log.save_log('Land Sequence Timeout')
        break

    latest_press, delta_press, press_land_count, isLand = land.land_main(press_land_count=press_land_count, press_array=press_array)
    print('isLand: ' + str(isLand))
    #-Log-#
    land_log.save_log(latest_press, delta_press, press_land_count, isLand)
    if isLand == 1:
        break

#-Log-#
print('Saving Log...')
phase_log.save_log('2', 'Land Detect Sequence: End', 0, 0)

#-send-#
# print('Sending Data...')
# send.send_data('Land finished')
# time.sleep(10)

print('#####-----Land Detect Sequence: End-----#####')





#####===== 3 Melt Sequence=====#####
print('#####-----Melt Sequence: Start-----#####')

#-Log-#
phase_log.save_log('3', 'Melt Sequence: Start', 0, 0) #GPS情報取得できるのか？？
melt_log.save_log('Melt Start')

#-Melt-#
melt.melt_down(meltPin=MELT_PIN, t_melt=MELT_TIME)

#-Log-#
print('Saving Log...')
phase_log.save_log('3', 'Melt Sequence: End', 0, 0) #GPS情報取得できるのか？？
melt_log.save_log('Melt Finished')

#-send-#
# print('Sending Data...')
# # send.send_data('Melt finished')
# # time.sleep(10)

print('#####-----Melt Sequence: End-----#####')



#====================================================================================================#
#####-----スタビライザーの復元-----#####
print('Waiting for Stabilizer to be restored...')
time.sleep(15)

#####-----GPSの取得チェック-----##### いらないかも...
# while True:
#     lat_test, lon_test = gps.location()
#     if lat_test == 0 and lon_test == 0:
#         print('Waiting for GPS...')
#     elif lat_test != 0 and lon_test != 0: #0だった場合はGPSが取得できていないので再取得
#         print('GPS received')
#         break

lat_test, lon_test = gps.location()
print('GPS received')

#-send-#
print('Sending Data...')
# lat_str = "{:.6f}".format(lat_test)  # 緯度を小数点以下8桁に整形
# lon_str = "{:.6f}".format(lon_test)  # 経度を小数点以下8桁に整形
# send.send_data(lat_str)
# time.sleep(9)
# send.send_data(lon_str)
# time.sleep(9)
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
    lat_now, lon_now, para_dist, red_area, angle, isDistant_parachute, check_count = para_avoid.main(lat_land, lon_land, lat_dest=LAT_HUMAN, lon_dest=LON_HUMAN, check_count=check_count)
    
    #-Log-#
    para_avoid_log.save_log(lat_now, lon_now, para_dist, red_area, angle, isDistant_parachute, check_count=check_count)
    if isDistant_parachute == 1: #パラシュート回避用のフラグ
        break

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('4', 'Parachute Avoid Sequence: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
# send.send_data('Parachute Avoid finished')
# time.sleep(10)

print('#####-----Parachute Avoid Sequence: End-----#####')





#####===== 5 GPS Running Sequence to Human =====#####
print('#####-----GPS Running Sequence to Human: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('5', 'GPS Running Sequence to Human: Start', lat_log, lon_log)

#-GPS Running1-#
direction = calibration.calculate_direction(lon2=LON_HUMAN, lat2=LAT_HUMAN)
distance_to_goal = direction["distance"]
print(f'{distance_to_goal}m to Human')

while True: #1ループおおよそT_CAL秒

    #-T_CALごとに以下の情報を取得-#
    lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_HUMAN, lat_dest=LAT_HUMAN, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)
    print('disntance to dest=' + str(distance_to_dest) + 'm')
    #-Log-#
    gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)    
    
    #-send-#
    lat_str = "{:.6f}".format(lat_now)  # 緯度を小数点以下8桁に整形
    lon_str = "{:.6f}".format(lon_now)  # 経度を小数点以下8桁に整形
    send.send_data(lat_str)
    time.sleep(9)
    send.send_data(lon_str)
    time.sleep(9)

    if isReach_dest == 1: #ゴール判定
        break

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



#-Load Model-#
print('Loading Machine Learning Model...')
ML_people = DetectPeople(model_path="model_mobile.tflite" )

# while True:



































#####===== 7 GPS Running Sequence to Goal=====#####
print('#####-----GPS Running Sequence to Goal: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('7', 'GPS Running Sequence to Goal: Start', lat_log, lon_log)

#-GPS Running2-#

direction = calibration.calculate_direction(lon2=LON_GOAL, lat2=LAT_GOAL)
distance_to_goal = direction["distance"]
print('distance to goal=' + str(distance_to_goal) + 'm')

while True: #1ループおおよそT_CAL秒

    #-T_CALごとに以下の情報を取得-#
    lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_GOAL, lat_dest=LAT_GOAL, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)

    print('disntance to dest=' + str(distance_to_dest) + 'm')
    #-Log-#
    gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)

    #-send-#
    lat_str = "{:.6f}".format(lat_now)  # 緯度を小数点以下8桁に整形
    lon_str = "{:.6f}".format(lon_now)  # 経度を小数点以下8桁に整形
    send.send_data(lat_str)
    time.sleep(9)
    send.send_data(lon_str)
    time.sleep(9)
    
    if isReach_dest == 1: #ゴール判定
        print('Finishing GPS Running')
        break

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
magx_off, magy_off = calibration.cal(30, -30, 30)

while True:
    lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal = goal_detect.main(lat_dest=LAT_GOAL, lon_dest=LON_GOAL, thd_distance_goal=THD_DISTANCE_GOAL, thd_red_area=THD_RED_RATIO, magx_off=magx_off, magy_off=magy_off)
    image_guide_log.save_log(lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal)
    print(f'{distance_to_goal}m to Goal')
    if isReach_goal == 1: #ゴール判定
        print('Goal')
        break

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