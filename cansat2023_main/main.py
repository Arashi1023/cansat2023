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
from collections import deque

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
import human_detect
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
motor.setup()
#####=====log setup=====#####
phase_log = log.Logger(dir='../logs/0_phase_log', filename='phase', t_start=t_start, columns=['phase', 'condition', 'lat', 'lon'])
report_log = log.Logger(dir='../logs/0_report_log', filename='report', t_start=t_start, columns=['N', 'W'])
release_log = log.Logger(dir='../logs/1_release_log', filename='release', t_start=t_start, columns=['latest_press', 'delta_press', 'press_release_count', 'isRelease'])
land_log = log.Logger(dir='../logs/2_land_log', filename='land', t_start=t_start, columns=['latest_press', 'delta_press', 'press_land_count', 'isLand'])
melt_log = log.Logger(dir='../logs/3_melt_log', filename='melt', t_start=t_start, columns=['condition'])
para_avoid_log = log.Logger(dir='../logs/4_para_avoid_log', filename='para_avoid', t_start=t_start, columns=['lat', 'lon', 'distance_to_parachute', 'red_area', 'angle', 'isDistant_parachute', 'check_count'])
gps_running_human_log = log.Logger(dir='../logs/5_gps_running_human_log', filename='gps_running_human', t_start=t_start, columns=['lat', 'lon', 'distance_to_human', 'rover_azimuth', 'isReach_human'])
human_detection_log = log.Logger(dir='../logs/6_human_detection_log', filename='human_detection', t_start=t_start, columns=['lat', 'lon', 'result', 'judge_count', 'area_count', 'rotate_count', 'add_pwr', 'isHuman'])
gps_running_goal_log = log.Logger(dir='../logs/7_gps_running_goal_log', filename='gps_running_goal', t_start=t_start, columns=['lat', 'lon', 'distance_to_goal', 'rover_azimuth', 'isReach_goal'])
image_guide_log = log.Logger(dir='../logs/8_image_guide_log', filename='image_guide', t_start=t_start, columns=['lat', 'lon', 'distance_to_goal', 'area_ratio', 'angle', 'add_pwr', 'isReach_goal'])

#####=====Mission Sequence=====#####


#####===== 1 Release Detect Sequence=====#####
print('#####-----Release Detect Sequence: Start-----#####')
phase_log.save_log('1', 'Release Detect Sequence: Start', 0, 0) #GPS情報は取得しちゃだめ
release_log.save_log('Release Detect Start')


#-Release Detect-#
press_release_count = 0
press_array = [0]*2

while True:
    if time.time() - t_start > 3: #RELEASE_TIMEOUT: タイムアウトの設定
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
phase_log.save_log('2', 'Land Detect Sequence: Start', 0, 0) #GPS情報取得は取得しちゃだめ

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
phase_log.save_log('3', 'Melt Sequence: Start', 0, 0) #GPS情報取得は取得しちゃだめ?
melt_log.save_log('Melt Start')

#-Melt-#
melt.melt_down(meltPin=MELT_PIN, t_melt=MELT_TIME)

#-Log-#
print('Saving Log...')
phase_log.save_log('3', 'Melt Sequence: End', 0, 0)
melt_log.save_log('Melt Finished')

#-send-#
# print('Sending Data...')
# # send.send_data('Melt finished')
# # time.sleep(10)

print('#####-----Melt Sequence: End-----#####')



#====================================================================================================#
#####-----スタビライザーの復元-----#####
print('Waiting for Stabilizer to be restored...')
time.sleep(2)

lat_test, lon_test = gps.location()
report_log.save_log(lat_test, lon_test) #着地地点のGPS座標の取得とログの保存 実質スタート地点の保存
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

# #-Log-#
# print('Saving Log...')
# lat_log, lon_log = gps.location()
# phase_log.save_log('4', 'Parachute Avoid Sequence: Start', lat_log, lon_log)

# #-Parachute Avoid-#
t_start = time.time()
stuck_check_array = deque([0]*6, maxlen=6)
add_pwr = 0
add_count = 0
magx_off = -830
magy_off = -980

#-Log Set up-#

print('Para Avoid Start')
check_count = 0 #パラ回避用のカウンター
lat_land, lon_land = gps.location()
while True:
    if time.time() - t_start >= 600: #10分たっても
        red_area = para_avoid.detect_para()
        if red_area == 0:
            motor.move(60, -60, 2)
            break
        else:
            print('Parachute is near')
            print('Wait 10s')
            time.sleep(10)

    ###---現在のローバーの方位角を求める---###
    magdata = bmx055.mag_dataRead()
    magx, magy = magdata[0], magdata[1]
    rover_aziimuth = calibration.angle(magx=magx, magy=magy, magx_off=magx_off, magy_off=magy_off)
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

    lat_now, lon_now, para_dist, red_area, angle, isDistant_para, check_count = para_avoid.main(lat_land, lon_land, lat_dest=LAT_HUMAN, lon_dest=LON_HUMAN, check_count=check_count, add_pwr=add_pwr)
    print(lat_now, lon_now, para_dist, red_area, angle, isDistant_para, check_count)
    para_avoid_log.save_log(lat_now, lon_now, para_dist, red_area, angle, isDistant_para, check_count)
    if isDistant_para == 1:
        break
print("Para Avoid End")

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
#編集ログ -> sendはコメントアウトした それ以外はOK
print('#####-----GPS Running Sequence to Human: Start-----#####')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('5', 'GPS Running Sequence to Human: Start', lat_log, lon_log)
report_log.save_log(lat_log, lon_log)

#-GPS Running1-#
direction = calibration.calculate_direction(lon2=LON_HUMAN, lat2=LAT_HUMAN)
distance_to_goal = direction["distance"]
print(f'{distance_to_goal}m to Human')

while True: #1ループおおよそT_CAL秒

    #-T_CALごとに以下の情報を取得-#
    lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_HUMAN, lat_dest=LAT_HUMAN, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)
    print('disntance to dest=' + str(distance_to_dest) + 'm')
    print('isReach_dest=' + str(isReach_dest))

    #-Log-#
    lat_now, lon_now = gps.location() #ログ用のGPS情報の取得
    report_log.save_log(lat_now, lon_now)
    gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)    
    
    #-send-#
    # lat_str = "{:.6f}".format(lat_now)  # 緯度を小数点以下8桁に整形
    # lon_str = "{:.6f}".format(lon_now)  # 経度を小数点以下8桁に整形
    # send.send_data(lat_str)
    # time.sleep(9)
    # send.send_data(lon_str)
    # time.sleep(9)

    if isReach_dest == 1: #ゴール判定
        break

print(f'{distance_to_dest}m to Goal')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('5', 'GPS Running Sequence to Human: End', lat_log, lon_log)

#-send-#
# print('Sending Data...')
# send.send_data('Run1 finished')
# time.sleep(10)

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

#-変数定義-#
result = 0
area_count = 0
rotate_count = 0
isHuman = 0
judge_count = 0
stuck_check_array = deque([0]*6, maxlen=6)
add_pwr = 0
add_count = 0
t_start_detect = time.time()

magx_off, magy_off = calibration.cal(30, -30, 30)

while True:
    ###---回転場所の整地---###
    if rotate_count == 0: #ある地点で1枚目の写真を撮影するとき
        magx_off_stuck, magy_off_stuck = calibration.cal(30, -30, 30)
        stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化
        add_pwr = 0 #捜索地点を変えたら追加のパワーをリセット
        lat_now, lon_now = gps.location()

    ###---現在のローバーの方位角を求める---###
    magdata = bmx055.mag_dataRead()
    magx, magy = magdata[0], magdata[1]
    rover_aziimuth = calibration.angle(magx=magx, magy=magy, magx_off=magx_off, magy_off=magy_off)
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
            add_pwr = min(add_pwr, 25) #最大で25
            stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化

    result, judge_count, area_count, rotate_count, isHuman = human_detect.main(lat_human=LAT_HUMAN, lon_human=LON_HUMAN, model=ML_people, judge_count=judge_count, area_count=area_count, rotate_count=rotate_count, add_pwr=add_pwr)
    
    #-Log-#
    lat_now, lon_now = gps.location()
    if rotate_count == 0: #場所を移動したときに最初の1回のみログに記録する
        report_log.save_log(lat_now, lon_now)
    human_detection_log.save_log(lat_now, lon_now, result, judge_count, area_count, rotate_count, add_pwr, isHuman)
    print('result:', result)
    if isHuman == 1:
        print('Found a Missing Person')
        break
    if area_count == 9:
        print('Could Not Find a Missin Person')
        print('Mission Failed')
        break

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('6', 'Human Detection Sequence: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
send.send_data('Human Detection finished')
time.sleep(10)

print('#####-----Human Detection Sequence: End-----#####')

#-Sending Photos-#
chunk_size = 4   # 送る文字数。この数字の2倍の文字数が送られる。1ピクセルの情報は16進数で6文字で表せられるため、6の倍数の文字を送りたい。
delay = 3   # 伝送間隔（秒）
num_samples = 10 #GPSを読み取る回数
photo_quality = 30 #伝送する画像の圧縮率
count = 0
count_v = 0
count_error = 0
id_counter = 1

while True:
    try:
        utc, lat, lon, sHeight, gHeight = gps.read_gps()
        if utc == -1.0:
            if lat == -1.0:
                print("Reading gps Error")
                count_error = count_error +1
                if count_error > num_samples:
                    send.send_data("human_GPS_start")
                    print("human_GPS_start")
                    time.sleep(delay)
                    send.send_data("Reading gps Error")
                    print("Reading gps Error")
                    time.sleep(delay)
                    send.send_data("human_GPS_fin")
                    print("human_GPS_fin")
                    time.sleep(delay)
                    break
                # pass
            else:
                # pass
                print("Status V")
                count_v = count_v + 1
                if count_v > num_samples:
                    time.sleep(delay)
                    send.send_data("human_GPS_start")
                    print("human_GPS_start")
                    time.sleep(delay)
                    send.send_data("Status V")
                    print("Status V")
                    time.sleep(delay)
                    send.send_data("human_GPS_fin")
                    print("human_GPS_fin")
                    time.sleep(delay)
                    break
        else:
            # pass
            print(utc, lat, lon, sHeight, gHeight)
            lat, lon = gps.location()
            print(lat,lon)
            count = count +1
            if count % num_samples == 0:
                send_lat = "{:.6f}".format(lat)
                send_lon = "{:.6f}".format(lon)
                print(send_lat,send_lon)
            # 無線で送信
                time.sleep(delay)
                send.send_data("human_GPS_start")
                print("human_GPS_start")
                time.sleep(delay)
                send.send_data(send_lat)
                send.send_data(send_lon)
                print(lat,lon)
                time.sleep(delay)
                send.send_data("human_GPS_fin")
                print("human_GPS_fin")
                time.sleep (delay)
                break
        time.sleep(1)
    except KeyboardInterrupt:
        gps.close_gps()
        print("\r\nKeyboard Intruppted, Serial Closed")
    except:
        gps.close_gps()
        print(traceback.format_exc())





#---------------------画像伝送----------------------------#

time.sleep(15)
lat_log,lon_log=gps.location()
phase_log.save_log('6', 'Image Sending Sequence: Start', lat_log, lon_log)
#file_path = latest_picture_path
file_name = "../imgs/human_detect/all/jpg"  # 保存するファイル名を指定
photo_take = take.picture(file_name, 320, 240)
print("撮影した写真のファイルパス：", photo_take)

# 入力ファイルパスと出力ファイルパスを指定してリサイズ
input_file = photo_take     # 入力ファイルのパスを適切に指定してください
photo_name = "../imgs/human_detect/all/send_photo_resize.jpg"  # 出力ファイルのパスを適切に指定してください
new_width = 60            # リサイズ後の幅を指定します
new_height = 80           # リサイズ後の高さを指定します

# リサイズを実行
send_photo.resize_image(input_file, photo_name, new_width, new_height)

print("写真撮影完了")

# 圧縮したい画像のパスと出力先のパスを指定します
input_image_path = photo_name
compressed_image_path = 'compressed_test.jpg'

# 圧縮率を指定します（0から100の範囲の整数）
compression_quality = photo_quality

# 画像を圧縮します
send_photo.compress_image(input_image_path, compressed_image_path, compression_quality)

# 圧縮後の画像をバイナリ形式に変換します
with open(compressed_image_path, 'rb') as f:
    compressed_image_binary = f.read()


data = compressed_image_binary  # バイナリデータを指定してください
output_filename = "output.txt"  # 保存先のファイル名

start_time = time.time()  # プログラム開始時刻を記録

send.send_data ("wireless_start")

print("写真伝送開始します")
time.sleep(1)


# バイナリデータを32バイトずつ表示し、ファイルに保存する
with open(output_filename, "w") as f:
    for i in range(0, len(data), chunk_size):
        if id_counter%30==0:
            time.sleep(10)
        chunk = data[i:i+chunk_size]
        chunk_str = "".join(format(byte, "02X") for byte in chunk)
        
        # 識別番号とデータを含む行の文字列を作成
        line_with_id = f"{id_counter}-{chunk_str}"

        #chunk_strにデータがある
        print(line_with_id)
        send.send_data(line_with_id)
        # 表示間隔を待つ
        time.sleep(delay)
        id_counter = id_counter +1

        # ファイルに書き込む
        f.write(line_with_id + "\n")

send.send_data ("wireless_fin")
send.send_data("num=" + str(id_counter))
time.sleep(10)

end_time = time.time()  # プログラム終了時刻を記録
execution_time = end_time - start_time  # 実行時間を計算

print("実行時間:", execution_time, "秒")
print("データを", output_filename, "に保存しました。")

lat_log,lon_log=gps.location()
phase_log.save_log('6', 'Image Sending Sequence: End', lat_log, lon_log)

########################################################################################################################################





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
    print('isReach_dest=' + str(isReach_dest))
    
    #-Log-#
    lat_now, lon_now = gps.location() #ログ用のGPS情報の取得
    report_log.save_log(lat_now, lon_now)
    gps_running_goal_log.save_log(lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest)

    #-send-#
    # lat_str = "{:.6f}".format(lat_now)  # 緯度を小数点以下8桁に整形
    # lon_str = "{:.6f}".format(lon_now)  # 経度を小数点以下8桁に整形
    # send.send_data(lat_str)
    # time.sleep(9)
    # send.send_data(lon_str)
    # time.sleep(9)
    
    if isReach_dest == 1: #ゴール判定
        print('Finishing GPS Running')
        break

print(f'{distance_to_dest}m to Goal')

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
report_log.save_log(lat_log, lon_log)
phase_log.save_log('7', 'GPS Running Sequence to Goal: End', lat_log, lon_log)

# #-send-#
# print('Sending Data...')
# send.send_data('Run2 finished')
# time.sleep(10)

print('#####-----GPS Running Sequence to Goal: End-----#####')





#####===== 8 Image Guide Sequence=====#####
print('#####-----Image Guide Sequence: Start-----#####')

#-Log-#
lat_log, lon_log = gps.location()
phase_log.save_log('8', 'Image Guide Sequence', lat_log, lon_log)
image_guide_log.save_log('Image Guide Sequence: Start')

#-Image Guide Drive-#
#-setup-#
t_start_goal = time.time()
stuck_check_array = deque([0]*6, maxlen=6)
add_pwr = 0
add_count = 0

magx_off, magy_off = calibration.cal(30, -30, 30)
print('Goal Detection Start')
while True:
    if time.time()-t_start_goal > 600: #ゴール検知をはじめて10分を超えたら
        print('Start Random Move')
        stuck2.stuck_avoid()
        t_start_goal = time.time() #タイマーのリセット
        print('Finish Random Move')
    
    ###---現在のローバーの方位角を求める---###
    magdata = bmx055.mag_dataRead()
    magx, magy = magdata[0], magdata[1]
    rover_aziimuth = calibration.angle(magx=magx, magy=magy, magx_off=magx_off, magy_off=magy_off)
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
            add_pwr = min(add_pwr, 25)
            stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化

    lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal = goal_detect.main(lat_dest=LAT_GOAL, lon_dest=LON_GOAL, thd_distance_goal=THD_DISTANCE_GOAL, thd_red_area=THD_RED_RATIO, magx_off=magx_off, magy_off=magy_off, add_pwr=add_pwr)
    
    lat_now, lon_now = gps.location() #ログ用のGPS情報の取得
    report_log.save_log(lat_now, lon_now)
    image_guide_log.save_log(lat_now, lon_now, distance_to_goal, area_ratio, angle, add_pwr, isReach_goal)
    print('area_ratio: ' + str(area_ratio))

    if isReach_goal == 1: #ゴール判定
        print('Goal')
        break

#-Log-#
print('Saving Log...')
lat_log, lon_log = gps.location()
phase_log.save_log('8', 'Image Guide Sequence: End', lat_log, lon_log)

#-send-#
print('Sending Data...')
# send.send_data('Image Guide finished')
# time.sleep(10)

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
