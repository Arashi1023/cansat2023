import gps_navigate
import time 
import rotation
import machine_learning #後で名前直して
from math import sqrt

def calculate_square_corners(lat1, lon1):#青点の設定
     #lat1,lon1はゴールの座標
    
    # 1度あたりの緯度経度の差（おおよその値）
    lat_diff_per_meter = 0.000009
    lon_diff_per_meter = 0.000011

    # 正方形の一辺の長さ（メートル）
    square_side_length = 20

    # 正方形の角の座標を計算
    lat2 = lat1 + (square_side_length / 2) * lat_diff_per_meter
    lon2 = lon1 - (square_side_length / 2) * lon_diff_per_meter

    lat3 = lat1 + (square_side_length / 2) * lat_diff_per_meter
    lon3 = lon1 + (square_side_length / 2) * lon_diff_per_meter

    lat4 = lat1 - (square_side_length / 2) * lat_diff_per_meter
    lon4 = lon1 - (square_side_length / 2) * lon_diff_per_meter

    lat5 = lat1 - (square_side_length / 2) * lat_diff_per_meter
    lon5 = lon1 + (square_side_length / 2) * lon_diff_per_meter

    return lat2, lon2, lat3, lon3, lat4, lon4, lat5, lon5


if __name__ =="__main__":

    start_time = time.time()
    threshold = 20 * 60
    elapsed_time = time.time()-start_time

    

    if elapsed_time >= threshold:
        print("A")#終了へ行くように変更して
    else:
        #print("B")#6回繰り返すところへ
        for i in range(6):
            rotation()
        
        print("C")#青点に移動するように変更して