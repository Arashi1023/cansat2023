'''
定数定義
'''
#-----放出判定-----#
RELEASE_THD_PRESS = 0.1
RELEASE_TIMEOUT = 60*5 #(秒)

#-----パラシュート回避-----#
PARA_CHECK_COUNT = 5
PARA_THD_AVOID = 0

#-----溶断回路-----#
MELT_PIN = 4
MELT_TIME = 4 #溶断回路に印加する時間

#-----GPS走行-----#
STUCK_JUDGE_THD_DISTANCE = 5
LOOP_NUM = 20 #0.05秒ごとに9軸センサを取得するので、20回のとき1秒間隔でGPSを取得する
t_run = 120 #キャリブレーションを行う間隔時間[sec]

#-----人検出-----#
LAT_HUMAN = 35.9243068
LON_HUMAN = 139.9124594

#-----ゴール地点-----#
LAT_GOAL = 35.9242411 #グランドのゴール前
LON_GOAL = 139.9120618 #グランドのゴール前
THD_DISTANCE_GOAL = 10 #画像誘導の範囲設定
THD_RED_AREA = 75 #画面を占める赤色の割合の閾値 この割合を超えるとゴールしたと判定する