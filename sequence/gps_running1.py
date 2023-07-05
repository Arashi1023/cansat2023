import datetime
import time
import gps_navigate
import gps
import bmx055
import motor #motor.move(l,r,t)
import im920sl
import calibration
import stuck2
import other

def angle_goal(magx_off, magy_off, lon2, lat2):
    """
    ゴールとの相対角度を算出する関数

    -180~180度
    """
    magdata = bmx055.mag_dataRead()
    mag_x = magdata[0]
    mag_y = magdata[1]
    theta = calibration.angle(mag_x, mag_y, magx_off, magy_off)
    direction = calibration.calculate_direction(lon2, lat2)
    azimuth = direction["azimuth1"]
    angle_relative = azimuth - theta
    if angle_relative >= 0:
        angle_relative = angle_relative if angle_relative <= 180 else angle_relative - 360
    else:
        angle_relative = angle_relative if angle_relative >= -180 else angle_relative + 360
    return angle_relative


def adjust_direction(theta, magx_off, magy_off, lon2, lat2):
    """
    方向調整
    """

    theta = angle_goal(magx_off, magy_off, lon2, lat2)

    print('ゴールとの角度theta = ' + str(theta) + '---回転調整開始！')
    stuck2.ue_jug()
    an = 20
    t_short = 0.1
    t_middle = 0.2
    t_long = 0.4

    while 45 < theta <= 180 or -180 < theta < -45:
        if 90 < theta <= 180 :
            motor.move(an, -an, t_long)
        elif -180 < theta < -90:
            motor.move(-an, an, t_long)
        elif 45 <= theta <= 90:
            motor.move(an, -an, t_middle)
        elif -90 <= theta <= -45:
            motor.move(-an, an, t_middle)
        elif 15 <= theta <= 45:
            motor.move(an, -an, t_short)
        elif -45 <= theta <= -15:
            motor.move(-an, an, t_short)
        
        theta = angle_goal(magx_off, magy_off, lon2, lat2)

        #print('Calculated angle_relative: {theta}')
        print('Calculated angle_relative')
        print(f'theta = {theta}')
        time.sleep(0.03)

    print("-----adjust_direction finished!!!------")

    '''
    stuck_count = 1
    t_small = 0.1
    t_big = 0.2
    force = 40
    while 30 < theta <= 180 or -180 < theta < -30:
        if stuck_count >= 16:
            ##方向調整が不可能な場合はスタックしたとみなして、もう一度キャリブレーションからスタート##
            other.print_im920sl(
                "!!!!can't ajdust direction.   start stuck avoid!!!!!")
            stuck2.stuck_avoid()
            magx_off, magy_off = calibration.cal(40, 40, 30)
            stuck_count = -1
        if stuck_count % 7 == 0:
            other.print_im920sl('Increase output')
            force += 10
        if 30 <= theta <= 60:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver1 (stuck:{stuck_count})')
            motor.move(force, -force, t_small)

        elif 60 < theta <= 180:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver2 (stuck:{stuck_count})')
            motor.move(force, -force, t_big)

        elif -60 <= theta <= -30:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver3 (stuck:{stuck_count})')
            motor.move(-force, force, t_small)
        elif -180 < theta < -60:
            other.print_im920sl(
                f'theta = {theta}\t---rotation_ver4 (stuck:{stuck_count})')
            motor.move(-force, force, t_big)
        else:
            print(f'theta = {theta}')

        stuck_count += 1
        stuck2.ue_jug()
        
    
        print('Calculated angle_relative: {theta}')
        time.sleep(1)
    other.print_im920sl(f'theta = {theta} \t rotation finished!!!')
    '''

def drive(lon2, lat2, thd_distance, t_adj_gps, logpath='/home/dendenmushi/cansat2023/sequence/log/gpsrunningLog.txt', t_start=0):
    """
    GPS走行の関数
    統合する場合はprintをXbee.str_transに変更，other.saveLogのコメントアウトを外す
    """
    direction = calibration.calculate_direction(lon2, lat2)
    goal_distance = direction['distance']

    # ------------- 上向き判定 -------------#
    while goal_distance >= thd_distance:
        t_stuck_count = 1
        stuck2.ue_jug()

        # ------------- calibration -------------#
        # xbee.str_trans('calibration Start')
        other.print_im920sl('##--calibration Start--##\n')
        print("------calibration Start------")
        magx_off, magy_off = calibration.cal(40, -40, 30)
        print(f'magx_off: {magx_off}\tmagy_off: {magy_off}\n')
        print("------calibration finished------")

        
        theta = angle_goal(magx_off, magy_off, lon2, lat2)
        adjust_direction(theta, magx_off, magy_off, lon2, lat2)
        

        t_cal = time.time()
        lat_old, lon_old = gps.location()
        #print("-------gps走行開始-------")
        while time.time() - t_cal <= t_adj_gps:
            print("-------gps走行-------")
            lat1, lon1 = gps.location()
            lat_new, lon_new = lat1, lon1
            direction = gps_navigate.vincenty_inverse(lat1, lon1, lat2, lon2)
            azimuth, goal_distance = direction["azimuth1"], direction["distance"]
            other.print_im920sl(
                f'lat: {lat1}\tlon: {lon1}\tdistance: {goal_distance}\tazimuth: {azimuth}\n')

            if t_stuck_count % 25 == 0:
                ##↑何秒おきにスタックジャッジするかを決める##
                if stuck2.stuck_jug(lat_old, lon_old, lat_new, lon_new, 1):
                    pass
                else:
                    stuck2.stuck_avoid()
                    pass
                lat_old, lon_old = gps.location()

            if goal_distance <= thd_distance:
                break
            else:
                print("ゴーるまでの距離は" + str(goal_distance) + "です")
                for _ in range(25):
                    print("25回ループの部分")
                    
                    magdata = bmx055.mag_dataRead()
                    mag_x = magdata[0]
                    mag_y = magdata[1]

                    print("----------mag_x, mag_yの読み取り値----------")
                    print(mag_x, mag_y)

                    magx, magy, magz = calibration.get_data()
                    print('********calibartion.get_data***********')
                    print(magx,magy,magz)

                    #theta = angle_goal(magx_off, magy_off, lon2, lat2)
                    #adjust_direction(theta, magx_off, magy_off, lon2, lat2)
                    
                    theta = calibration.angle(mag_x, mag_y, magx_off, magy_off)
                    angle_relative = azimuth - theta

                    if angle_relative >= 0:
                        angle_relative = angle_relative if angle_relative <= 180 else angle_relative - 360
                    else:
                        angle_relative = angle_relative if angle_relative >= -180 else angle_relative + 360
                    theta = angle_relative
                    
                    adj_r = 0
                    # if theta >= 0:
                    #     if theta <= 8:
                    #         adj = 0
                    #     elif theta <= 15:
                    #         adj = 5
                    #     elif theta <= 90:
                    #         adj = 20
                    #         adj_r = 5
                    #     else:
                    #         adj = 30
                    #         adj_r = 5
                    # else:
                    #     if theta >= - 8:
                    #         adj = 0
                    #     elif theta >= -15:
                    #         adj = -10
                    #     elif theta >= -90:
                    #         adj = -20
                    #     else:
                    #         adj = -30
                    if theta >= 0:
                        if theta <= 15:
                            pwr_r = 33
                            pwr_l = 35
                        elif theta <= 90:
                            pwr_r = 35
                            pwr_l = 40
                        else:
                            pwr_r = 30
                            pwr_l = 40
                    else:
                        if theta >= -15:
                            pwr_r = 35
                            pwr_l = 33
                        elif theta >= -90:
                            pwr_r = 40
                            pwr_l = 35
                        else:
                            pwr_r = 40
                            pwr_l = 30
                    print(f'angle ----- {theta}')
                    strength_l, strength_r = pwr_l ,pwr_r
                    #motor.motor_continue(strength_l, strength_r)
                    motor.motor_continue(strength_l, strength_r)
                    time.sleep(0.04)
                    #time.sleep(0.4)
            t_stuck_count += 1
            other.log(logpath, datetime.datetime.now(), time.time() -
                      t_start, lat1, lon1, direction['distance'], angle_relative)
            motor.deceleration(strength_l, strength_r)
            time.sleep(2)
            lat_new, lon_new = gps.location()
            print("whileの最下行")

        direction = calibration.calculate_direction(lon2, lat2)
        goal_distance = direction['distance']
        other.print_im920sl(f'-----distance: {goal_distance}-----')


if __name__ == '__main__':
    # lat2 = 35.918548
    # lon2 = 139.908896
    # lat2 = 35.9234892
    # lon2 = 139.9118744
    #lat2 = 35.9240057
    #lon2 = 139.9114077
    #lat2 = 35.9184282 シダックス
    #lon2 = 139.9111039シダックス
    #lat2 = 35.9240087
    #lon2 = 139.9113212
    
    #生協入口
    #lat2 = 35.91818718
    #lon2 = 139.90814829

    #12号館前
    #lat2 = 35.91896917
    #lon2 = 139.90859362

    #グランドのゴール前
    #lat2 = 35.923914
    #lon2 = 139.912223

    #狭いグランドのほう
    #lat2 = 35.9243874
    #lon2 = 139.9114187

    #中庭の芝生
    #lat2 = 35.91817415
    #lon2 = 139.90825559

    #実験棟の前
    lat2 = 35.9189778
    lon2 = 139.9071493 

    gps.open_gps()
    bmx055.bmx055_setup()
    motor.setup()

    drive(lon2, lat2, thd_distance=10, t_adj_gps=100)
