import serial

def send_data(serial_port, data):
    serial_port.write((data + '\r\n').encode())

def receive_data(serial_port):
    received_data = serial_port.readline().strip().decode()
    return received_data

if __name__ == '__main__':
    port = '/dev/ttyAMA0'
    baudrate = 19200

    # シリアルポートの初期化
    serial_port = serial.Serial(port, baudrate)
    serial_port.timeout = 1

    while True:
        # 送信データの入力
        send_text = input("送信するデータを入力してください ('q'で終了): ")
        if send_text == 'q':
            break

        # データの送信
        send_data(serial_port, send_text)

        # データの受信
        received_text = receive_data(serial_port)

        # 結果の表示
        print("送信データ:", send_text)
        print("受信データ:", received_text)

    # シリアルポートのクローズ
    serial_port.close()
