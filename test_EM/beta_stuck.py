#走破性能試験用プログラム
import time
import cansat2023_main.libs.bmx055 as bmx055
import libs.motor as motor
import libs.gps_navigate as gps_navigate
import random
import libs.gps as gps
import libs.bmx055 as bmx055
from cansat2023_main.libs import bmx055

def upside_down(pwr=40, pwr_adj=20, t_forward=0.08):
    count = 0
    
    while True:
        za = []
        for i in range(3):
            accdata = bmx055.acc_dataRead()
            za.append(accdata[2])
            time.sleep(0.2)
        z = max(za)

        if z > 5:
            print("upward")
            break
        else:
            print(f"Upside-down{count}{z}")
            if 2 <= count < 4: 
                pass
            elif 4 <= count < 6:
                pwr = pwr + pwr_adj 
            elif 6 <= count < 8:
                pwr = pwr + pwr_adj
            else:
                pwr = 12

            #move forward
            motor.move(pwr, pwr, t_forward)
            
            time.sleep(2)
            count += 1

def stuck_judge(lat_bf, lon_bf, lat_now, lon_now, thd_dist=10):
    data_stuck = gps_navigate.vincenty_inverse(lat_bf, lon_bf, lat_now, lon_now)
    move_dist = data_stuck['distance']
    if move_dist < thd_dist:
        print("-----stuck-----")
        return False
    else:
        print("-----not stuck-----")
        return True
    
def random_recover(num=7):
    random_num_list = []
    for i in range(num):
        random_num_list.append(i)
    rn = random.shuffle(random_num_list)
    return rn

def stuck_recover_move(x):
    if x == 0:
        print('stuck_recover:0')
        motor.move(-100, -100, 5)
        motor.move(-60, -60, 3)
    elif x == 1:
        print('stuck_recover:1')
        motor.move(40, -40, 1)
        motor.move(100, 100, 5)
    elif x == 2:
        print('stuck_recover:2')
        motor.move(-100, 100, 2)
        motor.move(100, 100, 5)
    elif x == 3:
        print('stuck_recover:3')
        motor.move(100, -100, 2)
        motor.move(100, 100, 5)
    elif x == 4:
        print('stuck_recover:4')
        motor.move(40, -40, 1)
        motor.move(-80, -100, 5)
    elif x == 5:
        print('stuck_recover:5')
        motor.move(40, -40, 1)
        motor.move(-100, -80, 5)
    elif x == 6:
        print('stuck_recover:6')
        motor.move(100, -100, 3)
        motor.move(100, 100, 3)

def stuck_recover():
    print("-----Start Stuck Recover-----")
    escape_flag = False
    while True:
        lat_bf, lon_bf = gps.location()
        for i in range(7):
            stuck_recover_move(i)
            lat_now, lon_now = gps.location()
            escape = stuck_judge(lat_bf, lon_bf, lat_now, lon_now)
            if escape == True:
                print("-----End Stuck Recover-----")
                escape_flag = True
                break
        if escape_flag == True:
            break
            
        random_num_list = random_recover()

        for i in range(7):
            stuck_recover_move(random_num_list[i])
            lat_now, lon_now = gps.location()
            escape = stuck_judge(lat_bf, lon_bf, lat_now, lon_now)
            if escape == True:
                print("-----End Stuck Recover-----")
                escape_flag = True
                break
        if escape_flag == True:
            break
    
    print("Recovered")

def running_test(thd_dist=3):
    print("-----Start Running Test-----")
    while True:
        upside_down()
        lat_bf, lon_bf = gps.location()
        motor.move(30, 30, 15)
        lat_now, lon_now = gps.location()

        #モータを前に回して動かしたのにも関わらず、動いていない場合はstuckと判断
        escape = stuck_judge(lat_bf, lon_bf, lat_now, lon_now, thd_dist)

        if escape == False:
            stuck_recover()
        else:
            print("not stuck")

if __name__ == '__main__':
    motor.setup()
    bmx055.bmx055_setup()
    gps.open_gps()

    running_test(thd_dist=2)
