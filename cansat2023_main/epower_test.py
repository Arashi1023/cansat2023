import motor
import time

def e_power_test(t_move):
    print("-----Start e_power_test-----")
    s_time = time.time()
    while True:
        motor.move(35, 35, t_move)
        time.sleep(10)

        #calibration
        motor.move(35, -35, 1.6)
        time.sleep(3)
        elapsed_time = time.time() - s_time
        print("elapsed_time:{0}".format(elapsed_time) + "[sec]")
        


if __name__ == '__main__':
    motor.setup()
    #2分間走らせて、10秒休憩
    e_power_test(120)