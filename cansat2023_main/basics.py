'''
制御に必要な基本的な処理まとめ
'''
import time
import datetime
import cv2
import send

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

def send_locations(lat: float, lon: float, text: str):
    send.send_data(text)
    time.sleep(10)
    lat_str = "{:.6f}".format(lat)  # 緯度を小数点以下8桁に整形
    lon_str = "{:.6f}".format(lon)  # 経度を小数点以下8桁に整形
    send.send_data(lat_str)
    time.sleep(9)
    send.send_data(lon_str)
    time.sleep(9)

if __name__ == "__main__":
    angle = int(input())

    print(standarize_angle(angle))  