import time
import libs.bme280 as bme280
import libs.send as send
import libs.log as log
from main_const import *
#from other import print

def pressdetect_release(thd_press_release, t_delta_release):
    '''
    気圧による放出判定
    '''
    global press_count_release
    global press_judge_release
    try:
        pressdata = bme280.bme280_read()
        prevpress = pressdata[1]
        time.sleep(t_delta_release)
        pressdata = bme280.bme280_read()
        latestpress = pressdata[1]
        deltP = latestpress - prevpress
        if 0.0 in pressdata:
            print("##--bme280rror!--##")
            press_count_release = 0
            press_judge_release = 2
        elif deltP > thd_press_release:
            press_count_release += 1
            if press_count_release > 1:
                press_judge_release = 1
                print("##--pressreleasejudge--##")
        else:
            press_count_release = 0
            press_judge_release = 0
    except KeyboardInterrupt:
        print('pressdetect_release_Interrupt')
        exit()
    except:
        press_count_release = 0
        press_judge_release = 2
    return press_count_release, press_judge_release

def release_main(press_release_count: int, press_array: list):

    isRelease = 0

    press_data = bme280.bme280_read()
    latest_press = press_data[1]
    press_array.append(latest_press) #press_arrayの更新
    press_array.pop(0) #press_arrayの更新
    if press_array[0] != 0 and press_array[1] != 0:
        delta_press = abs(press_array[1] - press_array[0])

        if delta_press > RELEASE_THD_PRESS:
            press_release_count += 1
            if press_release_count >= RELEASE_JUDGE_COUNT:
                isRelease = 1
        else:
            press_release_count = 0
    
    elif press_array[0] == 0 or press_array[1] == 0:
        print('Reading Press Again')
        delta_press = 0
        press_release_count = 0
    
    time.sleep(1)

    return latest_press, delta_press, press_release_count, isRelease

if __name__ == "__main__":
#     thd_press_release = 0.1
#     pressreleasecount = 0
#     pressreleasejudge = 0
#     t_delta_release = 10
#     bme280.bme280_setup()
#     bme280.bme280_calib_param()
#     press_d = 0

#     while True:
#         press_count_release, press_judge_release = pressdetect_release(thd_press_release, t_delta_release)
#         print(f'count:{pressreleasecount}\tjudge{pressreleasejudge}')
#         if press_count_release  > 3:
#             print('Press')
#             send.send_data("TXDU 0001.0001")
#             break
#         else:
#             print('unfulfilled')
# send.send_data("TXDU 0001.0002")

    bme280.bme280_setup()
    bme280.bme280_calib_param()

    t_start = time.time()

    release_log = log.Logger(dir='../logs/test_logs/release_test', filename='Release_test', t_start=t_start, columns=['latest_press', 'delta_press', 'press_release_count', 'isRelease'])

    #-Release Detect-#
    press_release_count = 0
    press_array = [0]*2

    while True:
        try:
            latest_press, delta_press, press_release_count, isRelease = release_main(press_release_count=press_release_count, press_array=press_array)
            #-Log-#
            release_log.save_log(latest_press, delta_press, press_release_count, isRelease)
            print(isRelease)
            if isRelease == 1:
                print("##--released--##")
                break
        except KeyboardInterrupt:
            print('release_Interrupt')
            exit()