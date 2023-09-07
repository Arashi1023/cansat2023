#パラシュート回避
import wgps_beta_photo_running as photo_running
import cv2
from save_photo import save_img
import take
import motor
import time

def detect_para():
    #画像の撮影
    path_all_para = '/home/dendenmushi/cansat2023/sequence/photo_storage/para_all/para_detect-'
    path_para_detect = './photo_storage/para_detected'
    photoname = take.picture(path_all_para)
    para_img = cv2.imread(photoname)
    angle = 0

    #画像を圧縮
    small_img = photo_running.mosaic(para_img, ratio=0.8)

    #赤色であると認識させる範囲の設定
    mask, masked_img = photo_running.detect_red(small_img)

    #圧縮した画像から重心と輪郭を求めて、画像に反映
    para_img, max_contour, cx, cy = photo_running.get_center(mask, small_img)

    #赤色が占める割合を求める
    red_area = photo_running.get_para_area(max_contour, para_img)

    #重心の位置から現在位置とパラシュートと相対角度を大まかに計算
    angle = photo_running.get_angle(cx, cy, para_img)

    if red_area == 0:
        angle = 0

    #パラシュートが検出された場合に画像を保存
    if red_area != 0:
        red_area = int(red_area)
        save_img(path_para_detect, 'para_detected_', str(red_area), para_img)
    
    return red_area, angle

def para_avoid(red_area, angle, check_count, thd_para_avoid=0, thd_para_count=4):

    #-----パラメータの設定-----#
    #-----周囲を確認する-----#
    pwr_check = 25
    t_check = 0.15
    i = 0
    
    #赤色発見=1 赤色未発見=0
    #found parachute
    f_para = 0

    #直進する
    pwr_f = 30
    t_forward = 1

    #読み込み
    #red_area, angle = detect_para()

    #パラシュートが覆いかぶさっていたとき用の閾値
    thd_para_covered = 69120

    #-----パラシュートが覆いかぶさっていたとき用の処理-----#
    while red_area > thd_para_covered:
        print("parachute on top")
        time.sleep(5)
        red_area, angle = detect_para()





    #-----初めて取った写真にパラシュートが映っていなかった場合-----#
    if red_area == 0:
        print("パラシュートは前方にありません。")
        print("念のため周囲を確認します。")

        #-----周囲を確認する-----#
        pwr_check = 25
        t_check = 0.15
        i = 0
        
        #赤色発見=1 赤色未発見=0
        #found parachute
        f_para = 0

        #-----右側を確認する-----#
        for r in range(check_count):
            print("右回転" + str(i+1) + "回目")
            motor.move(pwr_check, -pwr_check, t_check)
            red_area, angle = detect_para()
            i += 1

            #-----パラシュートを確認した場合-----#
            if red_area != 0:
                print("パラシュートを発見。")
                f_para = 1
                #-----初期位置に戻す-----#
                print("初期位置に戻します。")
                #右回転した分だけ左回転する。
                # for n in range(i+1):
                #     motor.move(-pwr_check, pwr_check, t_check)
                i += 1
                while i != 0:
                    motor.move(-pwr_check, pwr_check, t_check)
                    i -= 1
                break
            #-----パラシュートを確認できた場合ここでループから抜け出す-----#
        
        #-----パラシュートを確認できなかった場合-----#
        if f_para == 0:
            #直進する前に角度を調整する。2回だけ左回転する・
            print("直進する前に角度を調整します。")
            for a in range(2):
                print("角度調整" + str(a+1) + "回目（左回転）")
                motor.move(-pwr_check, pwr_check, t_check)
    
    else:
        print("パラシュートを前方に発見しました。")
        #-----初めて取った写真にパラシュートが映っていた場合-----#
        while red_area > thd_para_avoid:
            if angle == 1:
                print("右回転します。")
                t_rotate = 0.2
            elif angle == 2:
                print("強く右回転します。")
                t_rotate = 0.3
            elif angle == 3:
                print("左回転します。")
                t_rotate = -0.2

            if t_rotate > 0:
                motor.move(pwr_check, -pwr_check, t_rotate)
            else:
                motor.move(-pwr_check, pwr_check, abs(t_rotate))

            #-----回転後にパラシュートがあるかを確認-----#
            red_area, angle = detect_para()

    #-----パラシュート回避完了-----#

    #パラシュートが前方にないことが確認できたので、直進する。
    print("前方にパラシュートがないことを確認しました。直進します。")
    motor.move(pwr_f, pwr_f, t_forward)
    print("パラシュートは回避できました。")



        # #thd_para_avoidはパラシュートがあると判定する割合の閾値
        # #thd_para_countはパラシュートがないとき何回確認するかの閾値
        # pwr = 30
        # check_count = 0
        # i = 0

        # while 1:
        #     red_area, angle = detect_para()
        #     while red_area > thd_para_avoid and check_count <= thd_para_count:
        #         # if check_count == thd_para_count:
        #         #     break

        #         if angle == 1:
        #             motor.move(pwr, -pwr, 0.2)
        #         elif angle == 2:
        #             motor.move(pwr, -pwr, 0.3)
        #         elif angle == 3:
        #             motor.move(pwr, -pwr, 0.4)
        #         elif angle == 4:
        #             motor.move(-pwr, pwr, 0.3)
        #         elif angle == 5:
        #             motor.move(-pwr, pwr, 0.2)
        #         #elif red_area == 0 or angle == 0:
        #         else:
        #             i = 1 + check_count
        #             print("パラシュートはありません。確認" + str(i) + "回目です。")

        #         check_count += 1

        #         red_area, angle = detect_para()
        #     else:
        #         print(str(i) + "回確認しました。パラシュートはありません。")
        #         print("直進します。")
        #         red_area, angle = detect_para()
        #         break

        # #パラシュートが前方にないことが確認できたので、直進する。
        # # if red_area == 0 and check_count == thd_para_count:
        # pwr_st = 30
        # motor.move(pwr_st, pwr_st, 2)
        # print("パラシュートは回避できました。")
        # # print(str(i) + "回パラシュートがないことを確認しました。")

if __name__ == '__main__':
    #セットアップ
    motor.setup()

    red_area, angle = detect_para()
    para_avoid(red_area, angle, check_count=5)