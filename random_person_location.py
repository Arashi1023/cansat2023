from random import randint
import math
import random
from random import randint, random

# 最新のGPS座標
# lat = float(input("緯度を入力してください: "))
# lon = float(input("経度を入力してください: "))

lat = 35.9192167
lon = 139.9081122

# 極座標系に変換
lat_rad = math.radians(lat)
lon_rad = math.radians(lon)

#方位角を0~360度を決定する
# d= randint(270,90)  #北を向いている場合：270~90
d = randint(270, 360) if random() < 0.5 else randint(0, 90)
e = 20000
f = e/100


# 距離と方向を指定
distance_GPS = float(format(f, '.1f'))
direction_GPS = float(d)

# 方向をラジアンに変換
bearing = math.radians(direction_GPS)

# 緯度・経度に変換して移動
lat_new_GPS = math.degrees(math.asin(math.sin(lat_rad) * math.cos(distance_GPS / 6371000) + math.cos(lat_rad) * math.sin(distance_GPS / 6371000) * math.cos(bearing)))
lon_new_GPS = math.degrees(lon_rad + math.atan2(math.sin(bearing) * math.sin(distance_GPS / 6371000) * math.cos(lat_rad), math.cos(distance_GPS / 6371000) - math.sin(lat_rad) * math.sin(math.radians(lat_new_GPS))))

# 結果の表示
print("-----------------------------------------------")
print("通信が途絶える前に得られた遭難者の最新のGPS情報")
print(f"中心地点から、移動距離{distance_GPS}m、移動方向{direction_GPS}度の座標: ({lat_new_GPS:.6f}, {lon_new_GPS:.6f})")




lat = lat_new_GPS
lon = lon_new_GPS


# 極座標系に変換
lat_rad = math.radians(lat)
lon_rad = math.radians(lon)

#方位角を0~360度を決定する
a = randint(0,360)
b = randint(0,2000)
c = b/100


# 距離と方向を指定
distance = float(format(c, '.1f'))
direction = float(a)

# 方向をラジアンに変換
bearing = math.radians(direction)

# 緯度・経度に変換して移動
lat_new = math.degrees(math.asin(math.sin(lat_rad) * math.cos(distance / 6371000) + math.cos(lat_rad) * math.sin(distance / 6371000) * math.cos(bearing)))
lon_new = math.degrees(lon_rad + math.atan2(math.sin(bearing) * math.sin(distance / 6371000) * math.cos(lat_rad), math.cos(distance / 6371000) - math.sin(lat_rad) * math.sin(math.radians(lat_new))))

# 結果の表示
print("-----------------------------------------------")
print("遭難者のGPS情報")
print(f"A地点から、移動距離{distance}m、移動方向{direction}度の座標: ({lat_new:.6f}, {lon_new:.6f})")
print("-----------------------------------------------")
