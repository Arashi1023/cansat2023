import datetime
import time
import sys
import cv2
import pigpio
# import traceback
from math import sqrt
import numpy as np
from collections import deque

# import libs.bme280 as bme280
import bmx055
import motor
import save_photo
# import libs.send as send
import gps
import gps_navigate
# import libs.stuck2 as stuck2
import other
# import libs.send_photo as send_photo
import take
# from libs.machine_learning import DetectPeople
import calibration
import PID
import log
import basics
import stuck2

from main_const import *
import gps_running1

#細かいノイズを除去するために画像を圧縮
def mosaic(original_img, ratio):
    small_img = cv2.resize(original_img, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)
    return cv2.resize(small_img, original_img.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

#赤色検出
def detect_red(small_img):
    # HSV色空間に変換
    hsv_img = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)
    
    # 赤色のHSVの値域1
    red_min = np.array([0,105,20])
    red_max = np.array([13,255,255])
    mask1 = cv2.inRange(hsv_img, red_min, red_max)
    
    # 赤色のHSVの値域2
    red_min = np.array([160,105,20])
    red_max = np.array([179,255,255])
    mask2 = cv2.inRange(hsv_img, red_min, red_max)
    
    mask = mask1 + mask2

    masked_img = cv2.bitwise_and(small_img, small_img, mask=mask)
    
    return mask, masked_img

#赤色の重心を求める
def get_center(mask, original_img):
    try:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #最大の輪郭を抽出
        max_contour = max(contours, key = cv2.contourArea)

        #最大輪郭の重心を求める
        # 重心の計算
        m = cv2.moments(max_contour)
        cx,cy= m['m10']/m['m00'] , m['m01']/m['m00']
        # print(f"Weight Center = ({cx}, {cy})")
        # 座標を四捨五入
        cx, cy = round(cx), round(cy)
        # 重心位置に x印を書く
        cv2.line(original_img, (cx-5,cy-5), (cx+5,cy+5), (0, 255, 0), 2)
        cv2.line(original_img, (cx+5,cy-5), (cx-5,cy+5), (0, 255, 0), 2)

        cv2.drawContours(original_img, [max_contour], -1, (0, 255, 0), thickness=2)

    except:
        max_contour = 0
        cx = 0
        cy = 0
    
    return original_img, max_contour, cx, cy

def get_area(max_contour, original_img):
    try:
        #輪郭の面積を計算
        area = cv2.contourArea(max_contour)
        img_area = original_img.shape[0] * original_img.shape[1] #画像の縦横の積
        area_ratio = area / img_area * 100 #面積の割合を計算
        if area_ratio < 0.1:
            area_ratio = 0.0
        # print(f"Area ratio = {area_ratio:.1f}%")
    except:
        area_ratio = 0

    return area_ratio


#パラシュート回避用の関数
def get_para_area(max_contour, original_img):
    try:
        #輪郭の面積を計算
        area = cv2.contourArea(max_contour)
        img_area = original_img.shape[0] * original_img.shape[1] #画像の縦横の積
        area_ratio = area / img_area * 100 #面積の割合を計算
        print(f"Area = {area}")
        

        # 割合表示すると、小さな赤に反応しないので、赤色面積をそのまま表示する。
        # if area_ratio < 1.0:
        #     area_ratio = 0.0
        # print(f"Area ratio = {area_ratio:.1f}%")

        # print(f"Area = {area}")

    except:
        # area_ratio = 0
        area = 0

    return area

def get_angle(cx, cy, original_img):
    angle = 0
    #重心から現在位置とゴールの相対角度を大まかに計算
    img_width = original_img.shape[1]
    quat_width = img_width / 5
    x0, x1, x2, x3, x4, x5 = 0, quat_width, quat_width*2, quat_width*3, quat_width*4, quat_width*5

    if x0 < cx <x1:
        angle = 1
    elif x1 < cx < x4:
        angle = 2
    elif x4 < cx < x5:
        angle = 3
    
    # print("angle = ", angle)

    return angle

def detect_goal(lat2, lon2, thd_dist_goal=10, run_t=2):
    #-----赤色検知モードの範囲内にいるかどうかを判定-----#
    lat1, lon1 = gps.location()
    distance_azimuth = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
    dist_flag = distance_azimuth['distance']
    print("ゴールまでの距離は", dist_flag, "です。")

    #-----赤色検知モードの範囲外にいた場合の処理-----#
    while dist_flag > thd_dist_goal:
            print("GPS誘導を行います。")
            gps_running1.drive(lon2, lat2, thd_dist_goal, run_t)
            lat1, lon1 = gps.location()
            distance_azimuth = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
            dist_flag = distance_azimuth['distance']

    #-----赤色検知モードの範囲内にいた場合の処理-----#
    if dist_flag <= thd_dist_goal:
        #画像の撮影から「角度」と「占める割合」を求めるまでの一連の流れ
        path_all_photo = '../imgs/goal_detect/all/ImageGuide-'
        path_detected_photo = '../imgs/goal_detect/detected'
        photoname = take.picture(path_all_photo)
        original_img = cv2.imread(photoname)

        #画像を圧縮
        small_img = mosaic(original_img, 0.8)
        
        mask, masked_img = detect_red(small_img)

        original_img, max_contour, cx, cy = get_center(mask, small_img)

        #赤が占める割合を求める
        area_ratio = get_area(max_contour, original_img)

        #重心から現在位置とゴールの相対角度を大まかに計算
        angle = get_angle(cx, cy, original_img)

        #ゴールを検出した場合に画像を保存
        if area_ratio != 0:
            area_ratio = int(area_ratio) #小数点以下を切り捨てる（画像ファイル名にピリオドを使えないため）
            save_photo.save_img(path_detected_photo, 'Detected-', str(area_ratio), original_img)
        
    return area_ratio, angle

def TEST_detect_goal():
    #画像の撮影から「角度」と「占める割合」を求めるまでの一連の流れ
    path_all_photo = '../imgs/goal_detect/all/ImageGuide-'
    path_detected_photo = '../imgs/goal_detect/detected'
    photoname = take.picture(path_all_photo)
    original_img = cv2.imread(photoname)

    #画像を圧縮
    small_img = mosaic(original_img, 0.8)
    
    mask, masked_img = detect_red(small_img)

    original_img, max_contour, cx, cy = get_center(mask, small_img)

    #赤が占める割合を求める
    area_ratio = get_area(max_contour, original_img)

    #重心から現在位置とゴールの相対角度を大まかに計算
    angle = get_angle(cx, cy, original_img)

    #ゴールを検出した場合に画像を保存
    if area_ratio != 0:
        area_ratio = int(area_ratio) #小数点以下を切り捨てる（画像ファイル名にピリオドを使えないため）
        save_photo.main(path_detected_photo, 'detected', str(area_ratio), original_img)
    
    return area_ratio, angle

def image_guided_driving(area_ratio, angle, lat2, lon2, thd_full_red, thd_dist_goal, log_path, t_start):
    #thd_full_red = 0mゴールと判断するときの赤色が画像を占める割合の閾値
    #thd_dist_goal = 赤色検知モードの範囲の円の半径。ゴールから5mのとき赤色検知モードに入る。

    #赤色検知モードの範囲内にいるかどうかを判定
    lat1, lon1 = gps.location()
    distance_azimuth = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
    dist_flag = distance_azimuth['distance']
    other.log(log_path, datetime.datetime.now(), time.time()-t_start, 0, lat1, lon1)
    print("ゴールまでの距離は", dist_flag, "です。")

    #赤色検知モードの範囲外にいた場合の処理に必要な情報
    #running_time(GPS誘導を行う時間を)を設定
    running_time = 2

    try:
        while 1:
            if area_ratio >= thd_full_red:
                print("ゴール判定4")
                break
            #----------赤色検知モードの動作条件を満たしているかどうかを判定----------#
            while dist_flag <= thd_dist_goal:
                print("赤色検知モードに入ります。")
                area_ratio, angle = detect_goal(lat2, lon2)
                if area_ratio >= thd_full_red:
                    print("ゴール判定3")
                    break

                while area_ratio == 0:
                    print("ゴールが見つかりません。回転します。")
                    pwr_undetect = 25
                    motor.move(pwr_undetect, -pwr_undetect, 0.15)
                    area_ratio, angle = detect_goal(lat2, lon2)
                    lat1, lon1 = gps.location()
                    other.log(log_path, datetime.datetime.now(), time.time()-t_start, area_ratio, lat1, lon1)

                else:
                    if area_ratio >= thd_full_red:
                        print("ゴール判定2")
                        break
                    print("ゴールを捉えました。ゴールへ向かいます。")
                    area_ratio, angle = detect_goal(lat2, lon2)

                    while 0 < area_ratio < thd_full_red:
                        #lost_goalの初期化
                        lost_goal = 0

                        #cansatの真正面にゴールがないとき
                        while angle == 1 or angle == 3:
                            pwr_adj = 25
                            if angle == 1:
                                motor.move(-pwr_adj, pwr_adj, 0.15)
                            elif angle == 3:
                                motor.move(pwr_adj, -pwr_adj, 0.15)
                            elif area_ratio == 0:
                                lost_goal = 1
                                break
                            
                            area_ratio, angle = detect_goal(lat2, lon2)
                            lat1, lon1 = gps.location()
                            other.log(log_path, datetime.datetime.now(), time.time()-t_start, area_ratio, lat1, lon1)

                        if lost_goal == 1:
                            break

                        print("正面にゴールがあります。直進します。")

                        #cansatの真正面にゴールがあるとき
                        #angle が2のとき
                        pwr_l = 25
                        pwr_r = 30
                        if area_ratio >= thd_full_red:
                            print("ゴール判定1")
                            break
                        elif 80 < area_ratio < thd_full_red:
                            t_running = 0.1
                            pwr_l = 25
                            pwr_r = 30
                        elif 60 < area_ratio <= 80:
                            t_running = 0.15
                        elif 40 < area_ratio <= 60:
                            t_running = 0.2
                        elif 0 < area_ratio <= 40:
                            t_running = 0.25
                        
                        motor.move(pwr_l, pwr_r, t_running)
                        area_ratio, angle = detect_goal(lat2, lon2)
                        lat1, lon1 = gps.location()
                        other.log(log_path, datetime.datetime.now(), time.time()-t_start, area_ratio, lat1, lon1)
                    else: 
                        #area_ratio が90以上のときゴールを発見したのでループを抜ける
                        if area_ratio != 0:
                            break
                        print("ゴールを見失いました。ゴールを捉えるまで回転します。")
            else:
                print("ゴールから遠すぎます。GPSによる誘導を開始します。")
                gps_running1.drive(lon2, lat2, thd_dist_goal, running_time)

                #GPS誘導後、再度ゴールまでの距離を得る
                lat1, lon1 = gps.location()
                other.log(log_path, datetime.datetime.now(), time.time()-t_start, 0, lat1, lon1)
                distance_azimuth = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
                dist_flag = distance_azimuth['distance']
                print("ゴールまでの距離は", dist_flag, "です。")

        print("目的地周辺に到着しました。案内を終了します。")
        print("お疲れさまでした。")
        lat1, lon1 = gps.location()
        other.log(log_path, datetime.datetime.now(), time.time()-t_start, area_ratio, lat1, lon1)

    except KeyboardInterrupt:
        print("stop")
    # except Exception as e:
    #     tb = sys.exc_info()[2]

def TEST_img_guide_drive(magx_off, magy_off, thd_distance_goal=10, thd_red_area=75):
    '''
    8月28日作成 by 田口
    室内テスト用の関数 GPSなし
    '''
    area_ratio = 0
    distance_to_goal = 5
    isReach_goal = 0

    ###-----画像誘導のセットアップ キャリブレーションを行う-----###
    # magx_off, magy_off = calibration.cal(30, -30, 30)

    # try:
        ###-----ゴールまでの距離を測定-----###
        # lat_now, lon_now = gps.location()
        # goal_info = gps_navigate.vincenty_inverse(lat_now, lon_now, lat2, lon2)
        # distance_to_goal = goal_info['distance']
        # print(f'{distance_to_goal}m')

        ###-----画像誘導モードの範囲内にいた場合の処理-----###
    if distance_to_goal <= thd_distance_goal:
        print('画像誘導モードの範囲内にいます\n画像誘導を行います')
        area_ratio, angle = TEST_detect_goal()
        print(f'area_ratio = {area_ratio}, angle = {angle}')
        mag_data = bmx055.mag_dataRead()
        mag_x, mag_y = mag_data[0], mag_data[1]
        rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)
        rover_azimuth = basics.standarize_angle(rover_azimuth)
        
        ###-----撮像した画像の中にゴールが映っていた場合の処理-----###
        if area_ratio >= thd_red_area:
            isReach_goal = 1
        elif 0 < area_ratio < thd_red_area:
            ###-----ゴールが真正面にあるときの処理-----###
            if angle == 2:
                # rover_azimuth はそのまま使用
                target_azimuth = rover_azimuth
            ###------ゴールが真正面にないときの処理------###
            ###-----目標角度を少しずらす-----###
            elif angle == 1:
                target_azimuth = rover_azimuth - 15
            elif angle == 3:
                target_azimuth = rover_azimuth + 15
                
            ###-----PID制御により前進-----###
            theta_array = [0]*5
            PID.PID_run(target_azimuth, magx_off, magy_off, theta_array=theta_array, loop_num=20)
            motor.deceleration(15, 15)
            motor.motor_stop(0.2)

        ###-----撮像した画像の中にゴールが映っていない場合の処理-----###
        elif area_ratio == 0:
            print('Goal Not Found')
            pwr_unfound = 25
            motor.motor_move(pwr_unfound, -pwr_unfound, 0.15)
            motor.motor_stop(0.2)
    
    ###-----画像誘導モードの範囲外にいた場合の処理-----###
    else:
        print('ゴールから遠すぎます\nGPS誘導を行います')

        lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_GOAL, lat_dest=LAT_GOAL, thd_distance=THD_DISTANCE_DEST, t_cal=T_CAL, loop_num=LOOP_NUM)

    time.sleep(0.04) #9軸センサ読み取り用

    ###-----ゴールした場合の処理-----###
    if isReach_goal == 1:
        print('ゴールしました。画像誘導を終了します。')
        
    # except:
    #     print('Error\nTry again')
    return area_ratio, angle, isReach_goal

def main(lat_dest: float, lon_dest: float, thd_distance_goal: float, thd_red_area: float, magx_off: float, magy_off: float, add_pwr: float):
    '''
    目的：画像誘導によりゴールに到達する
    8月28日作成 by 田口
    おそらくこっちにする
    
    Parameters
    ----------
    lat_dest : float
        目的地の緯度
    lon_dest : float
        目的地の経度
    thd_distance_goal : float
        画像誘導の範囲設定
    thd_red_area : float
        画面を占める赤色の割合の閾値 この割合を超えるとゴールしたと判定する
    '''

    area_ratio = 0
    angle = 0
    isReach_goal = 0

    ###-----ゴールまでの距離を測定-----###
    lat_now, lon_now = gps.location()
    goal_info = gps_navigate.vincenty_inverse(lat_now, lon_now, lat_dest, lon_dest)
    distance_to_goal = goal_info['distance']
    print(f'{distance_to_goal}m')

    ###-----画像誘導モードの範囲内にいた場合の処理-----###
    if distance_to_goal <= thd_distance_goal:
        print('画像誘導モードの範囲内にいます\n画像誘導を行います')
        area_ratio, angle = TEST_detect_goal()
        mag_data = bmx055.mag_dataRead()
        mag_x, mag_y = mag_data[0], mag_data[1]
        rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)
        rover_azimuth = basics.standarize_angle(rover_azimuth)
        
        ###-----撮像した画像の中にゴールが映っていた場合の処理-----###
        if area_ratio >= thd_red_area:
            isReach_goal = 1
        elif 0 < area_ratio < thd_red_area:
            ###-----ゴールが真正面にあるときの処理-----###
            if angle == 2:
                # rover_azimuth はそのまま使用
                target_azimuth = rover_azimuth
            ###------ゴールが真正面にないときの処理------###
            ###-----目標角度を少しずらす-----###
            elif angle == 1:
                target_azimuth = rover_azimuth - 15
            elif angle == 3:
                target_azimuth = rover_azimuth + 15
                
            ###-----PID制御により前進-----###
            theta_array = [0]*5
            PID.PID_run(target_azimuth, magx_off, magy_off, theta_array=theta_array, loop_num=20)
            motor.deceleration(15, 15)
            motor.motor_stop(0.2)

        ###-----撮像した画像の中にゴールが映っていない場合の処理-----###
        elif area_ratio == 0:
            print('Lost Goal')
            pwr_unfound = 25 + add_pwr
            motor.motor_move(pwr_unfound, -pwr_unfound, 0.15)
            motor.motor_stop(0.5)
    
    ###-----画像誘導モードの範囲外にいた場合の処理-----###
    else:
        print('ゴールから遠すぎます\nGPS誘導を行います')
        lat_now, lon_now, distance_to_dest, rover_azimuth, isReach_dest = PID.drive2(lon_dest=LON_GOAL, lat_dest=LAT_GOAL, thd_distance=5, t_cal=T_CAL, loop_num=LOOP_NUM)

    time.sleep(0.04) #9軸センサ読み取り用

    ###-----ゴールした場合の処理-----###
    if isReach_goal == 1:
        print('ゴールしました。画像誘導を終了します。')

    return lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal

if __name__ == "__main__":
    #実験用の座標
    #グランドのゴール前
    # lat2 = 35.9239389
    # lon2 = 139.9122408

    #狭いグランドのほう
    #lat2 = 35.9243874
    #lon2 = 139.9114187

    #中庭の芝生
    #lat2 = 35.9183424
    #lon2 = 139.9080371

    #実験棟の前
    #lat2 = 35.9189778
    #lon2 = 139.9071493

    #ゴール
    # lat2 = 35.9242411
    # lon2 = 139.9120618
    # log_photorunning =other.filename( '/home/dendenmushi/cansat2023/sequence/log/photorunninglog/photorunninglog','txt')
    # #セットアップ系
    # motor.setup()
    gps.open_gps()
    bmx055.bmx055_setup()

    # t_start = time.time()
    # image_guide_log = log.Logger(dir='../logs/test_logs/para_avoid_test', filename='para_avoid', t_start=t_start)
    # magx_off, magy_off = calibration.cal(30, -30, 30)

    # while True:
    #     area_ratio, angle, isReach_goal = TEST_img_guide_drive(magx_off=magx_off, magy_off=magy_off)
    #     image_guide_log.save_log(area_ratio, angle, isReach_goal)
    #     print(isReach_goal)
    #     if isReach_goal == 1:
    #         print('Goal')
    #         break


    stuck_check_array = deque([0]*6, maxlen=6)
    add_pwr = 0
    add_count = 0

    #-log-#
    t_start_goal = time.time()

    image_guide_log = log.Logger(dir='../logs/test_logs/image_guide_test', filename='Image_guide_test', t_start=t_start_goal, columns=['lat', 'lon', 'distance_to_goal', 'area_ratio', 'angle', 'isReach_goal'])

    #-Image Guide Drive-#
    magx_off, magy_off = calibration.cal(30, -30, 30) #キャリブレーション
    print('Start Image Guide Drive')
    while True:
        if t_start_goal - time.time() > 600: #ゴール検知をはじめて10分を超えたら
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
                stuck_check_array = deque([0]*6, maxlen=6) #スタックチェック用の配列の初期化

        lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal = main(lat_dest=LAT_GOAL, lon_dest=LON_GOAL, thd_distance_goal=THD_DISTANCE_GOAL, thd_red_area=THD_RED_RATIO, magx_off=magx_off, magy_off=magy_off, add_pwr=add_pwr)
        image_guide_log.save_log(lat_now, lon_now, distance_to_goal, area_ratio, angle, isReach_goal)
        print('distance_to_goal = ', distance_to_goal)
        print('area_ratio = ', area_ratio)
        print('angle = ', angle, '%')
        print('isReach_goal = ', isReach_goal)
                
        if isReach_goal == 1: #ゴール判定
            print('Goal')
            break
    
    print('Finish Image Guide Drive')
        
