#0623 作成開始　by田口
#人検知プログラム

#モデュールのインポート　（一時敵にコメントアウトしてる）
import time
import gps_navigate
from machine_learning import DetectPeople
import take

#サンプルデータ

#現在位置の情報
lat1 = 35.9192167
lon1 = 139.9081152
lat_human = 35.9192167
lon_human = 139.9081132

#人検知の情報
lat2 = 35.9192167
lon2 = 139.9081132

#最後の位置情報をもとに周囲の4つの点の座標を求める
def get_locations(lat_human, lon_human):
    #周囲の4つの位置を求める
    lat_n = 
    lon_n = 
    #east
    lat_e =
    lon_e =
    #south
    lat_s =
    lon_s =
    #west
    lat_w =
    lon_w =

    return lat_n, lon_n, lat_e, lon_e, lat_s, lon_s, lat_w, lon_w




if __name__ == "__main__":
    #現在位置と最終位置データの距離を得る
    data_distance_human = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
    print(data_distance_human)

    #距離が5m以内にいれば人検知モードに入る
    if data_distance_human['distance'] <= 5:
        print("人検知の範囲内にいます")
        print("人検知モードに入ります")
        ML_people = DetectPeople(model_path="model_mobile.tflite" )
        # image_path = 'imgs/hiroyuki.jpg'
        # image_path = 'imgs/saru.jpg'
        # ML_people.predict(image_path)
        while 1:
            img_path = take.picture('ML_imgs/image', 320, 240)
            ML_people.predict(image_path=img_path)


    else:
        print("人検知の範囲外にいます")
        print("人検知の範囲内に移動してください")
        #人検知の範囲外にいる場合は、移動するように指示をする。



