import base64
import take
import send

if __name__ == '__main__':
    try:
        #画像の撮影
        take.picture('../../imgs/human_detect/image.jpg',320,240)
        # 画像ファイルを開く
        with open('../../imgs/human_detect/image.jpg', 'rb') as image_file:
            # 画像をBase64にエンコード
            encoded_image = base64.b64encode(image_file.read()).decode('utf-8')

        # 送信データの作成
        data = "TXDU 0001," + encoded_image
        print(encoded_image)
        # データの送信
        send.send_data(data)
        print('写真を送信しました')
    except KeyboardInterrupt:
       print("\r\nKeyboard Intruppted, Serial Closed")