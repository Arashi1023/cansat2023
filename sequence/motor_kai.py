#motor.pyがちょっとわかりにくい気がするから、こっちを使おう。

import time
from gpiozero import Motor



def setup():
    """
    モータのセットアップ
    """
    global motor_r, motor_l
    Rpin1, Rpin2 = 5, 6
    Lpin1, Lpin2 = 23, 18
    motor_r = Motor(Rpin1, Rpin2)
    motor_l = Motor(Lpin1, Lpin2)

def motor_move(strength_l, strength_r, t_moving):
    '''
    引数の定義
    strength_l == 左モータの強さ
    strength_r == 右モータの強さ
    t_moving == 動かす時間
    '''

    #モータのセットアップ
    setup()
    
    strength_l = 
    strength_r = 