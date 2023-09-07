import serial
import time

def send_data(data, port='/dev/ttyAMA0', baudrate=19200):
    IM920Serial = serial.Serial(port, baudrate)
    IM920Serial.flushOutput()
    IM920Serial.write(("TXDU 0001,"+data + '\r\n').encode())
    IM920Serial.close()

if __name__ == '__main__':
    while 1:
        text = str(input())
        send_data(text)
        print('送信しました')
