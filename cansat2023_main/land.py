import time
import bme280 as bme280
import send as send
from main_const import *
import log as log

def land_main(press_land_count: int, press_array: list):

    isLand = 0

    press_data = bme280.bme280_read()
    latest_press = press_data[1]
    press_array.append(latest_press) #press_arrayの更新
    press_array.pop(0) #press_arrayの更新
    if press_array[0] != 0 and press_array[1] != 0:
        delta_press = abs(press_array[1] - press_array[0])

        if delta_press < LAND_THD_PRESS:
            press_land_count += 1
            if press_land_count >= LAND_JUDGE_COUNT:
                isLand = 1
        else:
            press_land_count = 0 #カウンターの初期化
    
    elif press_array[0] == 0 or press_array[1] == 0:
        print('Reading Press Again')
        delta_press = 0
        press_land_count = 0
    
    time.sleep(LAND_GET_PRESS_TIME)
    
    return latest_press, delta_press, press_land_count, isLand

# def pressdetect_land(thd_press_land):
#     """
#     気圧情報による着地判定用
#     引数はどのくらい気圧が変化したら判定にするかの閾値
#     """
#     global press_count_land
#     global press_judge_land
#     try:
#         pressdata = bme280.bme280_read()
#         Prevpress = pressdata[1]
#         time.sleep(1)
#         pressdata = bme280.bme280_read()
#         latestpress = pressdata[1]
#         delta_p = abs(latestpress - Prevpress)
#         if 0.0 in pressdata:
#             print("bme280error!")
#             press_count_land = 0
#             press_judge_land = 2
#         elif delta_p < thd_press_land:
#             press_count_land += 1
#             if press_count_land > 4:
#                 press_judge_land = 1
#                 print("presslandjudge")
#         else:
#             press_count_land = 0
#             press_judge_land = 0
#     except KeyboardInterrupt:
#         print('pressdetect_land_Interrupt')
#         exit()
#     except:
#         press_count_land = 0
#         press_judge_land = 2
#     return press_count_land, press_judge_land, delta_p, Prevpress, latestpress

if __name__ == "__main__":
    # print("Start")
    # send.send_data("TXDU 0001,0000")

    bme280.bme280_setup()
    bme280.bme280_calib_param()

    # landcount = 0
    # pressdata = [0.0, 0.0, 0.0, 0.0]

    # while True:
    #     presslandjudge = 0
    #     landcount, presslandjudge = pressdetect_land(0.1)
    #     print(f'count:{landcount}\tjudge:{presslandjudge}')
    #     if presslandjudge == 1:
    #         print('Press')
    #         send.send_data("TXDU 0001,1000")
    #         print('##--landed--##')
    #         send.send_data("TXDU 0001,1111")
    #         break
    #     else:
    #         print('Press unfulfilled')
    #         send.send_data("TXDU 0001,0001")


    #####-----test-----#####
    t_start = time.time()

    land_log = log.Logger(dir='../logs/test_logs/land_test', filename='Land_test', t_start=t_start, columns=['latest_press', 'delta_press', 'press_land_count', 'isLand'])
    #-Land Detect-#
    press_land_count = 0
    press_array = [0]*2

    while True:
        try:
            latest_press, delta_press, press_land_count, isLand = land_main(press_land_count=press_land_count, press_array=press_array)
            #-Log-#
            land_log.save_log(latest_press, delta_press, press_land_count, isLand)
            print(isLand)
            if isLand == 1:
                print('Land Detected')
                break
        except:
            print('Error\nTrying again...')
