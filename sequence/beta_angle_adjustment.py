#モータの出力を自動的に最適化する

import motor
import calibration
import bmx055
import time

def motor_optimization(angle):
    #-----モータを最適化する-----#
    #-----引数は回転させたい角度-----#
    #各初期値を設定
    pwr = 25
    t_move = 0.15

    magx_off, magy_off = calibration.cal(40, -40, 40)

    #ループに入れてあげるためにangle_difを設定している。
    angle_dif_ratio = 50
    opt_count = 0
    #-----モータを最適化する-----#
    while 5 < angle_dif_ratio:
        if pwr >60:
            pwr = 30
        #最適化を10回以上したとき
        if opt_count > 30:
            print("最適化を30回以上行いました。")
            t_move += 0.1
            pwr += 10


        #最適化を何回したか表示
        print("最適化を" + str(opt_count) + "回行いました。")

        #回転前の角度を取得
        magx, magy, magz = bmx055.mag_dataRead()
        angle_bf = calibration.angle(magx, magy, magx_off, magy_off)
        
        #回転
        motor.move(pwr, -pwr, t_move)

        #回転後の角度を取得
        magx, magy, magz = bmx055.mag_dataRead()
        angle_af = calibration.angle(magx, magy, magx_off, magy_off)

        #回転した角度の計算
        rotated_angle = abs(angle_af - angle_bf)
        angle_dif = abs(angle - rotated_angle)
        print("理想の角度とのずれは" + str(angle_dif) + "度です。")

        #回転した角度の誤差が5%以内になるまでループ
        angle_dif_ratio = angle_dif / angle * 100

        #角度の最適化
        #cof_opt = 理論値 / 実測値
        cof_opt = angle / angle_dif

        if cof_opt < 1:
            #オーバー回転
            #t_move = t_move * cof_opt
            #pwr = pwr * cof_opt
            pwr -= 2
        elif cof_opt > 1:
            #アンダー回転
            pwr += 3

        opt_count += 1
        time.sleep(3)
    
    return pwr, t_move, opt_count
        


if __name__ == "__main__":
    a = int(input('回転させたい角度は？'))
    # pwr = int(input('モータの初期出力は？'))
    # t_move = float(input('モータの回転時間は？'))

    motor.setup()
    bmx055.bmx055_setup()
    magx_off, magy_off = calibration.cal(40, -40, 40)

    pwr, t_move, opt_count = motor_optimization(a)
    print(str(opt_count) + "回目で最適化が終了しました")
    print("pwr=" + str(pwr))
    print("t_move=" + str(t_move))

