'''
制御に必要な基本的な処理まとめ
'''

import time
import datetime
import cv2

def standarize_angle(angle):
    '''
    角度を-180～180度に収める関数
    '''
    angle = angle % 360
    
    if angle >180:
        angle -= 360
    elif angle < -180:
        angle += 360

    return angle

def save_img(img, img_path, img_name):
    dt_now = datetime.datetime.now()
    dt_name = str(dt_now.strftime('%Y%m%d_%H%M%S'))
    # final_img_path = img_path + "/" + img_name_a + '_' + dt_name + '_' + img_name_b + ".jpg"
    final_img_path = img_path + "_" + dt_name + img_name + ".jpg"

    #画像の保存
    cv2.imwrite(final_img_path, img)

    print("photo_saved")

if __name__ == "__main__":
    angle = int(input())

    print(standarize_angle(angle))  