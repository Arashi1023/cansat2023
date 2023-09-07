import time
import random

from other import print_im920sl
import im920sl2
import motor
import gps_navigate
import gps
import bmx055


def ue_jug():
    ue_count = 0
    
    #ローバーの状態を確認する関数
    #通常状態：True
    #逆さになってる：False
    #加速度センサZ軸の正負で判定するよ
    
    while 1:
        za = []
        for i in range(3):
            accdata = bmx055.acc_dataRead()
            za.append(accdata[2])
            time.sleep(0.2)
        z = max(za)
        
        if z > 3:
            im920sl2.str_trans('Upward')
            print('上だよ')
            break
        else:
            im920sl2.str_trans(f'Upside-down{ue_count}')
            print(f'下だよ{ue_count}')
            print(f'acc: {z}')
            if ue_count >= 2 and ue_count < 4:
                motor.move(40, 40, 0.08, False)
            elif ue_count >= 4 and ue_count < 6:
                motor.move(70, 70, 0.08, False)
            elif ue_count >= 6 and ue_count < 8:
                motor.move(100, 100, 0.5, False)
            else:
                motor.move(12, 12, 1, False)
            time.sleep(2)
            ue_count += 1

            

def ue_jugkai():
    
    ue_count = 0
    """
    ローバーの状態を確認する関数
    通常状態：True
    逆さになってる：False
    加速度センサZ軸の正負で判定する
    """
    while 1:
        xa = []
        za = []
        for i in range(3):
            accData = bmx055.acc_dataRead()
            xa.append(accData[0])
            za.append(accData[2])
            time.sleep(0.2)
        x = max(xa)
        z = max(za)

        if z >= 7.5 and x > 0:
            im920sl2.str_trans('Upward')
            print('上だよ')
            break
        else:
            im920sl2.str_trans(f'Upside-down{ue_count}')
            print(f'下だよ{ue_count}')
            print(f'acc: {z}')
            if ue_count > 2:
                #motor.move(13, -16, 0.1, False)
                motor.move(85, -85, 0.1, False)
            elif ue_count > 4:
                #motor.move(16, -19, 0.1, False)
                motor.move(90, -90, 0.1, False)
            elif ue_count > 6:
                #motor.move(19, -21, 0.1, False)
                motor.move(95, -95, 0.1, False)
            elif ue_count > 8:
                #motor.move(22, -24, 0.1, False)
                motor.move(99, -99, 0.1, False)
            else:
                #motor.move(16, 15, 0.1, False)
                motor.move(80, -80, 0.1, False)
            time.sleep(2)
            ue_count += 1


def stuck_jug(lat1, lon1, lat2, lon2, thd=1.0):
    data_stuck = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
    if data_stuck['distance'] <= thd:
        print(str(data_stuck['distance']) + '----!!!    stuck   !!!')
        print_im920sl(str(data_stuck['distance']) + '----!!!    stuck   !!!')
        return False
    else:
        print(str(data_stuck['distance']) + '-----not stucked')
        print_im920sl(str(data_stuck['distance']) + '-----not stucked')
        return True


def random(a, b, k):
    ns = []
    while len(ns) < k:
        n = random.randint(a, b)
        if not n in ns:
            ns.append(n)
    return ns


def stuck_avoid_move(x):
    if x == 0:
        print_im920sl('stuck_avoid_move():0')
        motor.move(-100, -100, 2)
        time.sleep(1)
        motor.move(-60, -60, 5)
        time.sleep(1)
    elif x == 1:
        print_im920sl('stuck_avoid_move():1')
        motor.move(40, -40, 1)
        time.sleep(1)
        motor.move(80, 80, 5)
        time.sleep(1)
    elif x == 2:
        print_im920sl('stuck_avoid_move():2')
        motor.move(-100, 100, 2)
        time.sleep(1)
        motor.move(80, 80, 5)
        time.sleep(1)
    elif x == 3:
        print_im920sl('stuck_avoid_move():3')
        motor.move(100, -100, 2)
        time.sleep(1)
        motor.move(80, 80, 5)
        time.sleep(1)
    elif x == 4:
        print_im920sl('stuck_avoid_move():4')
        motor.move(40, -40, 1)
        time.sleep(1)
        motor.move(-80, -80, 5)
        time.sleep(1)
    elif x == 5:
        print_im920sl('stuck_avoid_move():5')
        motor.move(40, -40, 1)
        time.sleep(1)
        motor.move(-80, -80, 5)
        time.sleep(1)
    elif x == 6:
        print_im920sl('stuck_avoid_move():6')
        motor.move(100, -100, 3)
        time.sleep(1)
        motor.move(80, 80, 3)
        time.sleep(1)

def stuck_avoid():
    print_im920sl('start stuck  avoid')
    print('start stuck  avoid')
    flag = False
    while 1:
        lat_old, lon_old = gps.location()
        # 0~6
        for i in range(7):
            stuck_avoid_move(i)
            lat_new, lon_new = gps.location()
            bool_stuck = stuck_jug(lat_old, lon_old, lat_new, lon_new, 0.5)
            if bool_stuck == True:
                flag = True
                break
        if flag:
            break
        # 3,2,1,0
        for i in range(7):
            stuck_avoid_move(7 - i)
            lat_new, lon_new = gps.location()
            bool_stuck = stuck_jug(lat_old, lon_old, lat_new, lon_new, 0.5)
            if bool_stuck == False:
                # if i == 1 or i == 4 or i == 5:
                #     print('スタックもう一度引っかからないように避ける')
                #     motor.move(-60, -60, 2)
                #     motor.move(-60, 60, 0.5)
                #     motor.move(80, 80, 3)
                flag = True
                break
        if flag:
            break
        random = random(0, 6, 7)
        for i in range(7):
            stuck_avoid_move(random[i])
            lat_new, lon_new = gps.location()
            bool_stuck = stuck_jug(lat_old, lon_old, lat_new, lon_new, 0.5)
            if bool_stuck == False:
                # if i == 1 or i == 4 or i == 5:
                #     print('スタックもう一度引っかからないように避ける')
                #     motor.move(-60, -60, 2)
                #     motor.move(-60, 60, 0.5)
                #     motor.move(80, 80, 3)
                flag = True
                break
        if flag:
            break
    time.sleep(10) #モータ休めるために停止
    print_im920sl('complete stuck avoid')
    print('complete stuck avoid')


if __name__ == '__main__':
    motor.setup()
    ue_jug()
    while 1:
        a = int(input('出力入力しろ'))
        b = float(input('時間入力しろ'))
        motor.move(a, a, b)
