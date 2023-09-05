import serial
import time
import pigpio

pi = pigpio.pi()
sendPin = 22

def send_data(data, port='/dev/ttyAMA0', baudrate=19200):
    IM920Serial = serial.Serial(port, baudrate)
    IM920Serial.flushOutput()
    IM920Serial.write(("TXDU 0001,"+data + '\r\n').encode())
    IM920Serial.close()


def receive_data(port='/dev/ttyAMA0', baudrate=19200):
    global IM920Serial
    IM920Serial = serial.Serial(port, baudrate)
    # IM920Serial.flushInput()  # 入力バッファをクリア
    received_data = IM920Serial.readline().strip().decode()  # データを読み込み、改行文字を削除
    return received_data
    


def send_reset(t_reset = 10):
    """
	無線をリセットするための関数
	"""
    pi.write(sendPin, 1)
    time.sleep(3)
    pi.write(sendPin, 0)
    time.sleep(t_reset)
    pi.write(sendPin, 1)
    time.sleep(1)

def send_on(sendPin=22):
    '''
    無線をONにするための関数
    '''
    pi.write(sendPin, 1)

def send_off(sendPin=22):
    '''
    無線をOFFにするための関数
    '''
    pi.write(sendPin, 0)

if __name__ == '__main__':
    while 1:
        text = str(input())
        if text =="A":
            try:
                print("キルで")
                send_reset(t_reset = 10)
                print("finish")
            except:
                pi.write(sendPin, 0)

        
        send_data(text)
        print('送信しました')
        received_text = receive_data()
        receive_data
        print("受信",received_text)
        
    # シリアルポートのクローズ
    IM920Serial.close()