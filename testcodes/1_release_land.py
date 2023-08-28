import sys
sys.path.append('/home/dendenmushi/cansat2023/sequence/bme280')  ####
import time
import signal

from gps import gps_data_read
import bme280
from bmx055 import bmx055_read

def pressdetect_release(thd_press_release, t_delta_release):
    global press_count_release
    global press_judge_release
    try:
        pressdata = bme280.bme280_read()
        prevpress = pressdata[1]
        time.sleep(t_delta_release) #５秒待って次の気圧の値を読むよ
        pressdata = bme280.bme280_read()
        latestpress = pressdata[1]
        deltP = latestpress - prevpress #初めにとった気圧-後にとった気圧
        if 0.0 in pressdata:
            print("bme280error!")
            press_judge_release = 2
            press_count_release = 0
        elif deltP > thd_press_release:
            press_count_release += 1
            if press_count_release > 0:
                press_judge_release = 1
                print("pressreleasejudge")
        else:
            press_count_release = 0
            press_judge_release = 0
    except:
        press_count_release = 0
        press_judge_release = 2
    return press_count_release, press_judge_release


def pressdetect_land(anypress):
    """
    気圧情報による着地判定用
    引数はどのくらい気圧が変化したら判定にするかの閾値
    """
    global press_count_land
    global press_judge_land
    try:
        pressdata = bme280.bme280_read()
        Prevpress = pressdata[1]
        time.sleep(1) #1秒待って次の気圧の値を読むよ
        pressdata = bme280.bme280_read()
        Latestpress = pressdata[1]
        deltP = abs(Latestpress - Prevpress) #初めにとった気圧-後にとった気圧
        if 0.0 in pressdata:
            print("bme280error!")
            press_count_land = 0
            press_judge_land = 2
        elif deltP < anypress:
            press_count_land += 1
            if press_count_land > 5:
                press_judge_land = 1
                print("presslandjudge")
        else:
            press_count_land = 0
            press_judge_land = 0
    except:
        press_count_land = 0
        press_judge_land = 2
    return press_count_land, press_judge_land

def gpsdetect_land(anyalt):
    """
    GPS高度情報による着地判定用
    引数はどのくらい高度が変化したら判定にするかの閾値
    """
    global gps_count_land
    global gps_judge_land
    try:
        gpsdata = get_gps_data()  # GPSデータを取得する関数を仮定
        Prevalt = gpsdata['altitude']
        time.sleep(1)  # 1秒待って次の高度の値を読むよ
        gpsdata = get_gps_data()  # GPSデータを再度取得
        Latestalt = gpsdata['altitude']
        deltA = abs(Latestalt - Prevalt)  # 初めにとった高度 - 後にとった高度
        if 'altitude' not in gpsdata:
            print("GPS error!")
            press_count_land = 0
            press_judge_land = 2
        elif deltA < anyalt:
            press_count_land += 1
            if press_count_land > 5:
                press_judge_land = 1
                print("presslandjudge")
        else:
            press_count_land = 0
            press_judge_land = 0
    except:
        gps_count_land = 0
        gps_judge_land = 2
    return gps_count_land, gps_judge_land
    
def acc_detect_land(anymax):
    """
    加速度センサーによる着地判定用
    引数はどのくらい加速度が変化したら判定にするかの閾値
    """
    global acc_count_land
    global acc_judge_land
    try:
        bmx_data = bmx055_read()
        acc_x, acc_y, acc_z = bmx_data[:3]
        acceleration = (acc_x ** 2 + acc_y ** 2 + acc_z ** 2) ** 0.5  # 加速度の大きさを計算
        delta_acceleration = abs(acceleration - prev_acceleration)  # 前回の加速度との変化量

        if delta_acceleration < anymax:
            acc_count_land += 1
            if acc_count_land > 5:
                acc_judge_land = 1
                print("acclandjudge")
        else:
            acc_count_land = 0
            acc_judge_land = 0

        prev_acceleration = acceleration  # 現在の加速度を保存

    except:
        acc_count_land = 0
        acc_judge_land = 2

    return acc_count_land, acc_judge_land

def handle_interrupt(signal, frame):
    #キーボードの割り込み処理
    print("Interrupted")
    sys.exit(0)
    

if __name__ == '__main__':
    bme280.bme280_setup()
    bme280.bme280_calib_param()
    startTime = time.time()
    #放出判定用
    press_count_release = 0
    press_judge_release = 0
    #着地判定用
    press_count_land = 0
    press_judge_land = 0

    #キーボードの割り込みのシグナルハンドラを設定
    signal.signal(signal.SIGINT, handle_interrupt)

    #####放出判定#######
    try:
        while 1:
            press_count_release, press_judge_release = pressdetect_release(0.3,0.5) 
            pressdate = bme280.bme280_read()
            print(f'count{press_count_release}\tjudge{press_judge_release}\ttemprature{pressdate[0]}\tpressure{pressdate[1]}')
            if press_judge_release == 1:
                print('release detected')
                break
    except KeyboardInterrupt:
       print("Program interrupted")
    sys.exit(0)

    #####着地判定######
    try:
         while 1:
            press_count_land, press_judge_land = pressdetect_land(0.1) #閾値0.1
            gps_count_land, gps_judge_land = gpsdetect_land(10) #閾値10
            acc_count_land, acc_judge_land = acc_detect_land(0.05) # 加速度による着地判定（閾値0.05）

            # GPSの高度情報を取得
            utc, lat, lon, sHeight, gHeight = gps_data_read()
            print(f'count{press_count_land}\tjudge{press_judge_land}\count{gps_count_land}\tjudge{gps_judge_land}')
            if (press_judge_land == 1 and gps_judge_land == 1) or (press_judge_land == 1 and acc_judge_land == 1) or (gps_judge_land == 1 and acc_judge_land == 1):
                print('land detected')
                break


             #if press_judge_land == 1:
#                 print('land detected')
#                 break
    except KeyboardInterrupt:
        print("Program interrupted")
        sys.exit(0)




    # try:
    #     while 1:
    #         presscount_release, pressjudge_release = pressdetect_release(0.3)
    #         print(f'count{presscount_release}\tjudge{pressjudge_release}')
    #         if pressjudge_release == 1:
    #             print('release detected')
    #             break
    #
    #     while 1:
    #         presscount_land, pressjudge_land = pressdetect_land(0.1)
    #         print(f'count{presscount_land}\tjudge{pressjudge_land}')
    #         if pressjudge_land == 1:
    #             print('land detected')
    #             break
    #
    #     print('finished')
    # except KeyboardInterrupt:
    #     print('interrupted')
    # except:
    #     print('finished')
