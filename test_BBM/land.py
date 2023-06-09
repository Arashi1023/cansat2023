import time
import sensor.environment.bme280 as bme280
from other import print_im920sl

def pressdetect_land(thd_press_land):
    """
    気圧情報による着地判定用
    引数はどのくらい気圧が変化したら判定にするかの閾値
    """
    global press_count_land
    global press_judge_land

    try:
        pressdata = bme280.bme280_read()
        Prevpress = pressdata[1]
        time.sleep(1)
        pressdata = bme280.bme280_read()
        latestpress = pressdata[1]
        delta_p = abs(latestpress - Prevpress)

        if 0.0 in pressdata:
            print_im920sl("bme280error!")
            press_count_land = 0
            press_judge_land = 2

        elif delta_p < thd_press_land:
            press_count_land += 1

            if press_count_land > 4:
                press_judge_land = 1
                print_im920sl("presslandjudge")

        else:
            press_count_land = 0
            press_judge_land = 0

    except:
        press_count_land = 0
        press_judge_land = 2

    return press_count_land, press_judge_land


if __name__ == "__main__":
    print_im920sl("Start")

    bme280.bme280_setup()
    bme280.bme280_calib_param()

    landcount = 0
    pressdata = [0.0, 0.0, 0.0, 0.0]

    while True:
        presslandjudge = 0
        landcount, presslandjudge = pressdetect_land(0.1)
        print_im920sl(f'count:{landcount}\tjudge:{presslandjudge}')
        
        if presslandjudge == 1:
            print_im920sl('Press')
            print_im920sl('##--landed--##')
            break

        else:
            print_im920sl('Press unfulfilled')
