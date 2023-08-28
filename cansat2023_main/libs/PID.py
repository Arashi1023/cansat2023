'''
PID制御モジュール
作成 by 田口 8/28
'''
import time
import datetime
import motor

theta_differential_array = []

def PID_control(theta, theta_array: list, Kp=0.1, Ki=0.04, Kd=2.5):
    '''
    作成 by 田口 8/28
    '''

    ###-----初期設定-----#
    ###-----角度情報の更新-----###
    del theta_array[0]
    theta_array.append(theta)

    ###-----P制御-----###
    theta_deviation = theta_array[-1]
    mp = Kp * theta_deviation

    ###-----I制御-----###
    theta_integral = sum(theta_array)
    mi = Ki * theta_integral

    ###-----D制御-----###
    for i in range(len(theta_array)):
        theta_differential_value = theta_array[i] - theta_array[i-1]
        theta_differential_array.append(theta_differential_value)

    ###-----最新のthetaの微分値を取得-----###
    theta_differential = theta_differential_array[-1]

    md = Kd * theta_differential

    ###-----PID制御-----###
    m = mp + mi - md

    return m

if __name__ == '__main__':
