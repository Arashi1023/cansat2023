import numpy as np
import cv2
import motor
import take
import sys
import gps_navigate
import datetime
import save_photo

#細かいノイズを除去するために画像を圧縮
def mosaic(original_img, ratio=0.1):
    small_img = cv2.resize(original_img, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)
    return cv2.resize(small_img, original_img.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

#赤色検出用のマスクを設定
def detect_red(small_img):
    # HSV色空間に変換
    hsv_img = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)
    
    # 赤色のHSVの値域1
    red_min = np.array([0,105,100])
    red_max = np.array([30,255,255])
    mask1 = cv2.inRange(hsv_img, red_min, red_max)
    
    # 赤色のHSVの値域2
    red_min = np.array([150,105,100])
    red_max = np.array([179,255,255])
    mask2 = cv2.inRange(hsv_img, red_min, red_max)
    
    mask = mask1 + mask2

    masked_img = cv2.bitwise_and(small_img, small_img, mask=mask)
    
    return mask, masked_img

#赤色の重心を求める
def get_center(mask, img):
    try:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #最大の輪郭を抽出
        max_contour = max(contours, key = cv2.contourArea)

        #最大輪郭の重心を求める
        # 重心の計算
        m = cv2.moments(max_contour)
        cx,cy= m['m10']/m['m00'] , m['m01']/m['m00']
        print(f"Weight Center = ({cx}, {cy})")
        # 座標を四捨五入
        cx, cy = round(cx), round(cy)
        # 重心位置に x印を書く
        cv2.line(img, (cx-5,cy-5), (cx+5,cy+5), (0, 255, 0), 2)
        cv2.line(img, (cx+5,cy-5), (cx-5,cy+5), (0, 255, 0), 2)

        cv2.drawContours(img, [max_contour], -1, (0, 255, 0), thickness=2)

    except:
        max_contour = 0
        cx = 0
        cy = 0
    
    return img, max_contour, cx, cy

def get_area(max_contour, original_img):
    try:
        #輪郭の面積を計算
        area = cv2.contourArea(max_contour)
        img_area = original_img.shape[0] * original_img.shape[1] #画像の縦横の積
        area_ratio = area / img_area * 100 #面積の割合を計算
        if area_ratio < 1.0:
            area_ratio = 0.0
        print(f"Area ratio = {area_ratio:.1f}%")
    except:
        area_ratio = 0

    return area_ratio

def get_angle(cx, cy, original_img):
    angle = 0
    #重心から現在位置とゴールの相対角度を大まかに計算
    img_width = original_img.shape[1]
    quat_width = img_width / 5
    x0, x1, x2, x3, x4, x5 = 0, quat_width, quat_width*2, quat_width*3, quat_width*4, quat_width*5

    if x0 < cx <x1:
        angle = 1
    elif x1 < cx < x2:
        angle = 2
    elif x2 < cx < x3:
        angle = 3
    elif x3 < cx < x4:
        angle = 4
    elif x4 < cx < x5:
        angle = 5
    
    print("angle = ", angle)

    return angle

def detect_goal():
    #画像の撮影から「角度」と「占める割合」を求めるまでの一連の流れ
    path_all_photo = '/home/dendenmushi/cansat2023/sequence/photo_imageguide/ImageGuide-'
    path_detected_photo = './photo_imageguide/detected'
    photoname = take.picture(path_all_photo)
    original_img = cv2.imread(photoname)

    #画像を圧縮
    small_img = mosaic(original_img, ratio=0.1)
    
    #赤色であると認識させる範囲を設定
    mask, masked_img = detect_red(small_img)

    #圧縮した画像から重心と輪郭を求めて、画像に反映
    original_img, max_contour, cx, cy = get_center(mask, small_img)

    #赤が占める割合を求める
    area_ratio = get_area(max_contour, original_img)

    #重心から現在位置とゴールの相対角度を大まかに計算
    angle = get_angle(cx, cy, original_img)

    #ゴールを検出した場合に画像を保存
    if area_ratio != 0:
        area_ratio = int(area_ratio)
        save_photo.save_img(path_detected_photo, 'detected_' + str(area_ratio) , original_img)

    return area_ratio, angle

def image_guided_driving(area_ratio, angle):
    t_running = 0
    area_ratio, angle = detect_goal()

    try:
        while 1:
            if area_ratio >= 90:
                print("ゴール判定3")
                break
            #----------赤色検知モードの動作条件を満たしているかどうかを判定----------#
            while area_ratio == 0:
                print("ゴールが見つかりません。回転します。")
                motor.move(30, -30, 0.1)
                area_ratio, angle = detect_goal()
            else:
                if area_ratio >= 90:
                    print("ゴール判定2")
                    break
                print("ゴールを捉えました。ゴールへ向かいます。")
                area_ratio, angle = detect_goal()

                while 0 < area_ratio < 90:
                    #lost_goalの初期化
                    lost_goal = 0

                    #cansatの真正面にゴールがないとき
                    while angle != 3:
                        if angle == 1:
                            motor.move(-25, 25, 0.2)
                        elif angle == 2:
                            motor.move(-25, 25, 0.1)
                        elif angle == 4:
                            motor.move(25, -25, 0.1)
                        elif angle == 5:
                            motor.move(25, -25, 0.2)
                        elif area_ratio == 0:
                            lost_goal = 1
                            break
                        
                        area_ratio, angle = detect_goal()

                    if lost_goal == 1:
                        break

                    print("正面にゴールがあります。直進します。")

                    #cansatの真正面にゴールがあるとき
                    pwr_l, pwr_r = 25, 25
                    if area_ratio >= 90:
                        print("ゴール判定1")
                        break
                    elif 80 < area_ratio < 90:
                        t_running = 0.1
                    elif 60 < area_ratio <= 80:
                        t_running = 0.15
                    elif 40 < area_ratio <= 60:
                        t_running = 0.2
                    elif 0 < area_ratio <= 40:
                        t_running = 0.25
                    
                    motor.move(pwr_l, pwr_r, t_running)
                    area_ratio, angle = detect_goal()

                else: 
                    #area_ratio が90以上のときゴールを発見したのでループを抜ける
                    if area_ratio != 0:
                        break
                    print("ゴールを見失いました。ゴールを捉えるまで回転します。")

    except KeyboardInterrupt:
        print("stop")
    # except Exception as e:
    #     tb = sys.exc_info()[2]

if __name__ == "__main__":
    motor.setup()
    angle = 0
    t_running = 0

    try:
        area_ratio, angle = detect_goal()
        image_guided_driving(area_ratio, angle)

    except KeyboardInterrupt:
        print("keyboard interrupt")
    # except Exception as e:
    #     tb = sys.exc_info()[2]