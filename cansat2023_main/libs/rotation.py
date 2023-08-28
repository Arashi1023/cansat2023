import RPi.GPIO as GPIO
import time

# モーターのピン番号を設定します
left_motor_pin = 1  # 左モーターのピン番号
right_motor_pin = 2  # 右モーターのピン番号

# GPIOの設定
GPIO.setmode(GPIO.BOARD)
GPIO.setup(left_motor_pin, GPIO.OUT)
GPIO.setup(right_motor_pin, GPIO.OUT)

# モーターを回転させる関数
def rotate(duration):
    GPIO.output(left_motor_pin, GPIO.HIGH)
    GPIO.output(right_motor_pin, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(left_motor_pin, GPIO.LOW)
    GPIO.output(right_motor_pin, GPIO.LOW)

# メインの処理
try:
    gear_ratio = 30  # ギヤ比
    input_rpm = 7000  # 入力軸の回転速度（RPM）

    output_rpm = input_rpm / gear_ratio  # 出力軸の回転速度を計算します
    rotation_per_sec = output_rpm / 60  # 1秒あたりの回転数を計算します

    rotation_time = 60 / (1 * rotation_per_sec)  # 60°回転するために必要な時間を計算します

    rotate(rotation_time)  # 計算された時間だけ回転させます
except KeyboardInterrupt:
    GPIO.cleanup()
