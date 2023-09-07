import time
from PIL import Image
import base64
from jinja2 import Environment, FileSystemLoader
import math
import time
import pigpio
import numpy as np
import traceback
import send
import cv2
from take import picture

RX = 27
pi = pigpio.pi()
chunk_size = 8  # 1回に表示するバイト数
delay = 1  # 表示間隔（秒）
num_samples = 5 #GPSの平均取る回数
photo_quality = 20 #伝送する画像の圧縮率
count = 0

file_name = "/home/dendenmushi/cansat2023/sequence/ML_imgs/sendtest_photo.jpg"  # 保存するファイル名を指定

ELLIPSOID_GRS80 = 1  # GRS80
ELLIPSOID_WGS84 = 2  # WGS84

# Long Axis Radius and Flat Rate
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80] Long Axis Radius
        1 / 298.257222101,  # [GRS80] Flat Rate
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84] Long Axis Radius
        1 / 298.257223563,  # [WGS84] Flat Rate
    ],
}

# Limited times of Itereation
ITERATION_LIMIT = 1000


def open_gps():
    try:
        pi.set_mode(RX, pigpio.INPUT)
        pi.bb_serial_read_open(RX, 9600, 8)
    except pigpio.error as e:
        #print("Open gps Error")
        pi.set_mode(RX, pigpio.INPUT)
        pi.bb_serial_read_close(RX)
        pi.bb_serial_read_open(RX, 9600, 8)


def read_gps():
    utc = -1.0
    Lat = -1.0
    Lon = 0.0
    sHeight = 0.0
    gHeight = 0.0
    value = [0.0, 0.0, 0.0, 0.0, 0.0]

    (count, data) = pi.bb_serial_read(RX)
    if count:
        # print(data)
        # print(type(data))
        if isinstance(data, bytearray):
            gpsData = data.decode('utf-8', 'replace')

        # print(gpsData)
        # print()
        # print()、
        gga = gpsData.find('$GPGGA,')
        rmc = gpsData.find('$GPRMC,')
        gll = gpsData.find('$GPGLL,')

        # print(gpsData[rmc:rmc+20])
        # print(gpsData[gll:gll+40])
        if gpsData[gga:gga+20].find(",0,") != -1 or gpsData[rmc:rmc+20].find("V") != -1 or gpsData[gll:gll+60].find("V") != -1:
            utc = -1.0
            Lat = 0.0
            Lon = 0.0
        else:
            utc = -2.0
            if gpsData[gga:gga+60].find(",N,") != -1 or gpsData[gga:gga+60].find(",S,") != -1:
                gpgga = gpsData[gga:gga+72].split(",")
                # print(gpgga)
                if len(gpgga) >= 6:
                    utc = gpgga[1]
                    lat = gpgga[2]
                    lon = gpgga[4]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -2.0
                        Lat = 0.0
                        Lon = 0.0
                    if gpgga[3] == "S":
                        Lat = Lat * -1
                    if gpgga[5] == "W":
                        Lon = Lon * -1
                else:
                    utc = -2.0
                if len(gpgga) >= 12:
                    try:
                        sHeight = float(gpgga[9])
                        gHeight = float(gpgga[11])
                    except:
                        pass
                    #print(sHeight, gHeight)
            # print(gpsData[gll:gll+60].find("A"))
            if gpsData[gll:gll+40].find("N") != -1 and utc == -2.0:
                gpgll = gpsData[gll:gll+72].split(",")
                # print(gpgll)
                # print("a")
                if len(gpgll) >= 6:
                    utc = gpgll[5]
                    lat = gpgll[1]
                    lon = gpgll[3]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -2.0
                    if gpgll[2] == "S":
                        Lat = Lat * -1
                    if gpgll[4] == "W":
                        Lon = Lon * -1
                else:
                    utc = -2.0
            if gpsData[rmc:rmc+20].find("A") != -1 and utc == -2.0:
                gprmc = gpsData[rmc:rmc+72].split(",")
                # print(gprmc)
                # print("b")
                if len(gprmc) >= 7:
                    utc = gprmc[1]
                    lat = gprmc[3]
                    lon = gprmc[5]
                    try:
                        utc = float(utc)
                        Lat = round(float(lat[:2]) + float(lat[2:]) / 60.0, 6)
                        Lon = round(float(lon[:3]) + float(lon[3:]) / 60.0, 6)
                    except:
                        utc = -1.0
                        Lat = 0.0
                        Lon = 0.0
                    if(gprmc[4] == "S"):
                        Lat = Lat * -1
                    if(gprmc[6] == "W"):
                        Lon = Lon * -1
                else:
                    utc = -1.0
                    Lat = -1.0
                    Lon = 0.0
            if utc == -2.0:
                utc = -1.0
                Lat = -1.0
                Lon = 0.0

    value = [utc, Lat, Lon, sHeight, gHeight]
    for i in range(len(value)):
        if not (isinstance(value[i], int) or isinstance(value[i], float)):
            value[i] = 0
    return value


def close_gps():
    pi.bb_serial_read_close(RX)
    pi.stop()


def cal_rhoang(lat_a, lon_a, lat_b, lon_b):
    if(lat_a == lat_b and lon_a == lon_b):
        return 0.0, 0.0
    ra = 6378.140  # equatorial radius (km)
    rb = 6356.755  # polar radius (km)
    F = (ra-rb)/ra  # flattening of the earth
    rad_lat_a = np.radians(lat_a)
    rad_lon_a = np.radians(lon_a)
    rad_lat_b = np.radians(lat_b)
    rad_lon_b = np.radians(lon_b)
    pa = np.arctan(rb/ra*np.tan(rad_lat_a))
    pb = np.arctan(rb/ra*np.tan(rad_lat_b))
    xx = np.arccos(np.sin(pa)*np.sin(pb) + np.cos(pa) *
                   np.cos(pb)*np.cos(rad_lon_a-rad_lon_b))
    c1 = (np.sin(xx)-xx)*(np.sin(pa) + np.sin(pb))**2 / np.cos(xx/2)**2
    c2 = (np.sin(xx)+xx)*(np.sin(pa) - np.sin(pb))**2 / np.sin(xx/2)**2
    dr = F/8*(c1-c2)
    rho = ra*(xx + dr) * 1000  # Convert To [m]
    angle = math.atan2(lon_a-lon_b,  lat_b-lat_a) * 180 / math.pi  # [deg]
    return rho, angle


def vincenty_inverse(lat1, lon1, lat2, lon2, ellipsoid=None):
    if lat1 == lat2 and lon1 == lon2:
        return 0.0, 0.0

    # Calculate Short Axis Radius
    # if Ellipsoid is not specified, it uses GRS80
    a, f = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
    b = (1 - f) * a

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    lambda1 = math.radians(lon1)
    lambda2 = math.radians(lon2)

    # Corrected Latitude
    U1 = math.atan((1 - f) * math.tan(phi1))
    U2 = math.atan((1 - f) * math.tan(phi2))

    sinU1 = math.sin(U1)
    sinU2 = math.sin(U2)
    cosU1 = math.cos(U1)
    cosU2 = math.cos(U2)

    # Diffrence of Longtitude between 2 points
    L = lambda2 - lambda1

    # Reset lamb to L
    lamb = L

    # Calculate lambda untill it converges
    # if it doesn't converge, returns None
    for i in range(ITERATION_LIMIT):
        sinLambda = math.sin(lamb)
        cosLambda = math.cos(lamb)
        sinSigma = math.sqrt((cosU2 * sinLambda) ** 2 +
                             (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) ** 2)
        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda
        sigma = math.atan2(sinSigma, cosSigma)
        sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma
        cos2Alpha = 1 - sinAlpha ** 2
        cos2Sigmam = cosSigma - 2 * sinU1 * sinU2 / cos2Alpha
        C = f / 16 * cos2Alpha * (4 + f * (4 - 3 * cos2Alpha))
        lambdaʹ = lamb
        lamb = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma *
                                             (cos2Sigmam + C * cosSigma * (-1 + 2 * cos2Sigmam ** 2)))

        # Deviation is udner 1e-12, break
        if abs(lamb - lambdaʹ) <= 1e-12:
            break
    else:
        return None

    # if it converges, calculates distance and angle
    u2 = cos2Alpha * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    dSigma = B * sinSigma * (cos2Sigmam + B / 4 * (cosSigma * (-1 + 2 * cos2Sigmam ** 2) -
                                                   B / 6 * cos2Sigmam * (-3 + 4 * sinSigma ** 2) * (-3 + 4 * cos2Sigmam ** 2)))

    s = b * A * (sigma - dSigma)  # Distance between 2 points
    alpha = -1 * math.atan2(cosU2 * sinLambda, cosU1 * sinU2 -
                            sinU1 * cosU2 * cosLambda)  # Angle between 2 points

    # return s(distance), and alpha(angle)
    return s, math.degrees(alpha)


def gps_data_read():
    '''
    GPSを読み込むまでデータをとり続ける関数
    '''
    try:
        while True:
            utc, lat, lon, sHeight, gHeight = read_gps()
            print('gps reading')
            if utc != -1.0 and lat != -1.0:
                break
            time.sleep(1)
        return utc, lat, lon, sHeight, gHeight
    except KeyboardInterrupt:
        close_gps()
        print("\r\nKeyboard Intruppted, Serial Closed")


def location():
    try:
        while True:
            utc, lat, lon, sHeight, gHeight = read_gps()
            if utc != -1.0 and lat != -1.0:
                break
            time.sleep(1)
        return lat, lon
    except KeyboardInterrupt:
        close_gps()
        print("\r\nKeyboard Intruppted, Serial Closed")


def capture_and_save_photo(file_name):
    # カメラを起動
    camera = cv2.VideoCapture(0)
    
    # カメラが正常にオープンされたか確認
    if not camera.isOpened():
        print("カメラをオープンできませんでした。")
        return
    
    # 画像を撮影
    ret, frame = camera.read()
    
    if not ret:
        print("画像の取得に失敗しました。")
        camera.release()
        return
    
    # 画像をファイルに保存
    cv2.imwrite(file_name, frame)
    print("写真が保存されました。")
    
    # カメラを解放
    camera.release()


def compress_image(image_path, output_path, quality):
    image = Image.open(image_path)
    image.save(output_path, optimize=True, quality=quality)


def image_file_to_base64(file_path):
    with open(file_path, "rb") as image_file:
        data = base64.b64encode(image_file.read())

    return data.decode('utf-8')


#-----------------------------------------------------------------------------------------------------------#
if __name__ == '__main__':

# #---------------------GPS情報送信--------------------------#
# #gps情報取得

#     open_gps()
#     t_start = time.time()
#     count = 0
#     while True:
#         try:
#             utc, lat, lon, sHeight, gHeight = read_gps()
#             if utc == -1.0:
#                 if lat == -1.0:
#                     print("Reading gps Error")
#                     # pass
#                 else:
#                     # pass
#                     print("Status V")
#             else:
#                 # pass
#                 print(utc, lat, lon, sHeight, gHeight)
#                 lat, lon = location()
#                 lat_sum += lat
#                 lon_sum += lon
#                 print(lat,lon)
#                 count = count +1
#                 if count % num_samples == 0:
#                     #平均計算
#                     avg_lat = lat_sum / num_samples
#                     avg_lon = lon_sum / num_samples
#                     print(avg_lat,avg_lon)
#                     break
#             time.sleep(1)
#         except KeyboardInterrupt:
#             close_gps()
#             print("\r\nKeyboard Intruppted, Serial Closed")
#         except:
#             close_gps()
#             print(traceback.format_exc())
    
#     # 無線で送信
#     send.send_data("human_GPS_start")
#     print("human_GPS_start")
#     time.sleep(delay)
#     send.sed_data(avg_lat,avg_lon)
#     print(avg_lat,avg_lon)
#     time.sleep(delay)
#     send.send_data("human_GPS_fin")
#     print("human_GPS_fin")
#     time.sleep(delay)
    
    
    
    #---------------------画像伝送----------------------------#
    
    
    photo_name = picture(file_name, 320, 240)
    print("撮影した写真のファイルパス：", photo_name)

    
    print("写真撮影完了")
    
    # 圧縮したい画像のパスと出力先のパスを指定します
    input_image_path = photo_name
    compressed_image_path = 'compressed_test.jpg'
    
    # 圧縮率を指定します（0から100の範囲の整数）
    compression_quality = photo_quality
    
    # 画像を圧縮します
    compress_image(input_image_path, compressed_image_path, compression_quality)
    
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
            chunk = data[i:i+chunk_size]
            chunk_str = "".join(format(byte, "02X") for byte in chunk)
            #chunk_strにデータがある
            print(chunk_str)
            send.send_data(chunk_str)
            # 表示間隔を待つ
            time.sleep(delay)
    
            # ファイルに書き込む
            f.write(chunk_str + "\n")
    
    send.send_data ("wireless_fin")
    
    end_time = time.time()  # プログラム終了時刻を記録
    execution_time = end_time - start_time  # 実行時間を計算
    
    print("実行時間:", execution_time, "秒")
    print("データを", output_filename, "に保存しました。")
