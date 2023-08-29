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

from main_const import *
import release
import land
import melt
import beta_gps_running as gps_running
import human_detection
import beta_para_avoid as para_avoid
import wgps_beta_photo_running as img_guide

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


#####=====Release Detect Sequence=====#####
print('Release Detect Sequence: Start')
release_log.save_log('start')


#####=====Land Detect Sequence=====#####
print('Land Detect Sequence: Start')

#####=====Melt Sequence=====#####
print('Melt Sequence: Start')
lat_melt, lon_melt = gps.location()



melt.main(meltPin=MELT_PIN, t_melt=MELT_TIME)
print('Melt Sequence: End')

#####=====Parachute Avoid Sequence=====#####
print('Parachute Avoid Sequence: Start')


stuck2.ue_jug()
time.sleep(15) #スタビライザーの復元待ち





print('Parachute Avoid Sequence: End')

#####=====GPS Running Sequence to Human=====#####




#####=====Human Detection Sequence=====#####




#####=====GPS Running Sequence to Goal=====#####
print('GPS Running Sequence to Goal: Start')
lat_log, lon_log = gps.location()
phase_log.save_log('8', 'GPS Running Sequence to Goal', lat_log, lon_log)
gps_running_goal_log.save_log('GPS Running Sequence to Goal: Start')


#-GPS Running2-#

while True:
    direction = calibration.calculate_direction(lon_dest=LON_GOAL, lat_dest=LAT_GOAL)
    distance_to_goal = direction["distance"]

    lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = drive2()

    #-Log-#
    gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)    
    if isReach_dest == 1: #ゴール判定
        break

    

#-Log-#

send.send_data('Run2 finished')
time.sleep(10)

print('GPS Running Sequence to Goal: End')



#####=====Image Guide Sequence=====#####
print('Image Guide Sequence: Start')

#-Log-#
img_guide_start_lat, img_guide_start_lon = gps.location()
phase_log.save_log('9', 'Image Guide Sequence', img_guide_start_lat, img_guide_start_lon)
image_guide_log.save_log('Image Guide Sequence: Start')

#-Image Guide Drive-#
while True:
    lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal = img_guide.img_guide_drive(lat_dest=LAT_GOAL, lon_dest=LON_GOAL, thd_distance_goal=THD_DISTANCE_GOAL, thd_red_area=THD_RED_AREA)
    image_guide_log.save_log(lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal)
    if isReach_goal == 1: #ゴール判定
        break

#-Log-#
image_guide_log.save_log('Image Guide Sequence: End')

print('Image Guide Sequence: End')



#####=====Mission End=====#####
print('Mission Accomlished')
send.send_data('Mission Accompished')
time.sleep(10)

#-Log-#
last_lat, last_lon = gps.location()
phase_log.save_log('10', 'All Phase Comleted', last_lat, last_lon)
release_log.save_log('(PDT)', 'N', last_lat, 'W', last_lon)

print("Saved Log\nEnding Program")