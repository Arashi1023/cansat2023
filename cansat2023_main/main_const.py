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

#-----人検出-----#
LAT_HUMAN = 35.9243068
LON_HUMAN = 139.9124594

#-----ゴール地点-----#
LAT_GOAL = 35.9242411 #グランドのゴール前
LON_GOAL = 139.9120618 #グランドのゴール前