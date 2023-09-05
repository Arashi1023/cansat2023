'''
定数定義
'''
#-----放出判定-----#
RELEASE_THD_PRESS = 1.5 #？秒あたりの気圧の変化量がこれ以上あった場合に放出判定を行う
RELEASE_JUDGE_COUNT = 4 #
RELEASE_JUDGE_TIME = 10 #気圧データを取得する間隔
RELEASE_TIMEOUT = 20 #(秒) プログラム開始時間からの経過時間


#-----着地判定-----#
LAND_THD_PRESS = 0.1
LAND_JUDGE_COUNT = 4
LAND_TIMEOUT = 60*5 #(秒) プログラム開始時間からの経過時間

#-----溶断回路-----#
MELT_PIN = 4
MELT_TIME = 4 #溶断回路に印加する時間

#-----パラシュート回避-----#
# PARA_CHECK_COUNT = 5
PARA_THD_RED_AREA = 0
PARA_THD_COVERED = 69120*0.7 #パラシュートが覆いかぶさっているか判定する閾値
PARA_PWR = 30 #パラシュートを見つけたときに回転するモーター出力
T_CHECK = 0.15
T_ROTATE = 0.35 #パラシュートを見つけたときに回転する時間
T_FORWARD = 3
THD_AVOID_ANGLE = 30
PARA_FORWARD_ANGLE = 45

SHORT_THD_DIST = 5 #これ以上離れたときPID制御により走行する
LONG_THD_DIST = 10 #これ以上離れたときPID制御により長く走行する


PARA_RUN_SHORT = 3
PARA_RUN_LONG = 10

#-----スタック回避用-----#
ADD_PWR = 10

#-----GPS走行-----#
STUCK_JUDGE_THD_DISTANCE = 10
LOOP_NUM = 20 #0.05秒ごとに9軸センサを取得するので、20回のとき1秒間隔でGPSを取得する
THD_DISTANCE_DEST = 5 #目的地に到達したと判定する距離
T_CAL = 35 #キャリブレーションを行う間隔時間[sec] 30の倍数+5秒ぐらいがおそらくベスト？？


#-----人検出-----#
LAT_HUMAN = 35.9243068
LON_HUMAN = 139.9124594
#LAT_HUMAN = 35.9243785
#LON_HUMAN = 139.9113644

JUDGE_PROBABILITY = 0.5 #人である確率がこれ以上のとき人がいると判定する
ADDITIONAL_JUDGE_COUNT = 3 #人がいると判定したとき、追加の確認を行う回数
ROTATE_COUNT = 10 # 1つの場所で回転する回数

HD_ROT_PWR = 25
HD_ROT_TIME = 0.15

#-----ゴール地点-----#
LAT_GOAL = 35.9242411 #グランドのゴール前
LON_GOAL = 139.9120618 #グランドのゴール前
#LAT_GOAL = 35.9240809
#LON_GOAL = 139.9113835
# LAT_GOAL = 35.9241007
# LON_GOAL = 139.9124145


THD_DISTANCE_GOAL = 5 #画像誘導の範囲設定
THD_RED_RATIO = 70 #画面を占める赤色の割合の閾値 この割合を超えるとゴールしたと判定する
