import math

def calculate_square_coordinates(latitude, longitude, side_length):
    # 現在の座標を中心とした正方形の4つの角の座標を算出する

    # 緯度1度あたりのメートル数
    meters_per_latitude_degree = 111319.9
    # 経度1度あたりのメートル数（中央緯度付近では約111319.9メートル）
    meters_per_longitude_degree = meters_per_latitude_degree * math.cos(math.radians(latitude))

    # 正方形の一辺の長さの半分
    half_side_length = side_length / 2

    # 正方形の4つの角の座標を計算
    top_left_latitude = latitude + (half_side_length / meters_per_latitude_degree)
    top_left_longitude = longitude - (half_side_length / meters_per_longitude_degree)
    bottom_right_latitude = latitude - (half_side_length / meters_per_latitude_degree)
    bottom_right_longitude = longitude + (half_side_length / meters_per_longitude_degree)

    # 結果を返す
    return {
        'top_left': {'latitude': top_left_latitude, 'longitude': top_left_longitude},
        'top_right': {'latitude': top_left_latitude, 'longitude': bottom_right_longitude},
        'bottom_left': {'latitude': bottom_right_latitude, 'longitude': top_left_longitude},
        'bottom_right': {'latitude': bottom_right_latitude, 'longitude': bottom_right_longitude}
    }

# 中心座標を指定
center_latitude = 35.6895  # 例として東京の緯度を指定
center_longitude = 139.6917  # 例として東京の経度を指定

# 正方形の一辺の長さを指定（メートル）
side_length = 20

# 正方形の4つの角の座標を計算
square_coordinates = calculate_square_coordinates(center_latitude, center_longitude, side_length)

# 結果を表示
print('Top Left:', square_coordinates['top_left'])
print('Top Right:', square_coordinates['top_right'])
print('Bottom Left:', square_coordinates['bottom_left'])
print('Bottom Right:', square_coordinates['bottom_right'])
