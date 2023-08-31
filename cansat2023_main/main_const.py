'''
定数定義
'''
#-----放出判定-----#
RELEASE_THD_PRESS = 0.1
RELEASE_JUDGE_COUNT = 4
RELEASE_TIMEOUT = 60*5 #(秒)


#-----着地判定-----#
LAND_THD_PRESS = 0.1
LAND_JUDGE_COUNT = 4

#-----溶断回路-----#
MELT_PIN = 4
MELT_TIME = 4 #溶断回路に印加する時間

#-----パラシュート回避-----#
# PARA_CHECK_COUNT = 5
PARA_THD_RED_AREA = 0
PARA_THD_COVERED = 69120 #パラシュートが覆いかぶさっているか判定する閾値
PARA_PWR = 25 #パラシュートを見つけたときに回転するモーター出力
T_CHECK = 0.15
T_ROTATE = 0.25 #パラシュートを見つけたときに回転する時間
T_FORWARD = 3
THD_AVOID_ANGLE = 15
PARA_FORWARD_ANGLE = 45

SHORT_THD_DIST = 5 #これ以上離れたときPID制御により走行する
LONG_THD_DIST = 10 #これ以上離れたときPID制御により長く走行する


PARA_RUN_SHORT = 3
PARA_RUN_LONG = 10

#-----GPS走行-----#
STUCK_JUDGE_THD_DISTANCE = 5
LOOP_NUM = 20 #0.05秒ごとに9軸センサを取得するので、20回のとき1秒間隔でGPSを取得する
THD_DISTANCE_DEST = 5 #目的地に到達したと判定する距離
T_CAL = 120 #キャリブレーションを行う間隔時間[sec]


#-----人検出-----#
LAT_HUMAN = 35.9243068
LON_HUMAN = 139.9124594

#-----ゴール地点-----#
LAT_GOAL = 35.9242411 #グランドのゴール前
LON_GOAL = 139.9120618 #グランドのゴール前
THD_DISTANCE_GOAL = 5 #画像誘導の範囲設定
THD_RED_RATIO = 75 #画面を占める赤色の割合の閾値 この割合を超えるとゴールしたと判定する