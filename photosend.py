import time
from PIL import Image
import binascii
import base64
from jinja2 import Environment, FileSystemLoader
import math
import time
import pigpio
import numpy as np
import traceback
import send

def compress_image(image_path, output_path, quality):
    image = Image.open(image_path)
    image.save(output_path, optimize=True, quality=quality)


def image_file_to_base64(file_path):
    with open(file_path, "rb") as image_file:
        data = base64.b64encode(image_file.read())

    return data.decode('utf-8')


# 圧縮したい画像のパスと出力先のパスを指定します
input_image_path = 'test-1.jpg'
compressed_image_path = 'compressed_test.jpg'

# 圧縮率を指定します（0から100の範囲の整数）
compression_quality = 0

# 画像を圧縮します
compress_image(input_image_path, compressed_image_path, compression_quality)

# 圧縮後の画像をバイナリ形式に変換します
with open(compressed_image_path, 'rb') as f:
    compressed_image_binary = f.read()


data = compressed_image_binary  # バイナリデータを指定してください
chunk_size = 16  # 1回に表示するバイト数
delay = 0.1  # 表示間隔（秒）
output_filename = "output.txt"  # 保存先のファイル名

start_time = time.time()  # プログラム開始時刻を記録

send.send_data = ("wireless_start")

# バイナリデータを32バイトずつ表示し、ファイルに保存する
with open(output_filename, "w") as f:
    for i in range(0, len(data), chunk_size):
        chunk = data[i:i+chunk_size]
        chunk_str = "".join(format(byte, "02X") for byte in chunk)
        #chunk_strにデータがある
        #print(chunk_str)
        send.send_data = (chunk_str)
        # 表示間隔を待つ
        time.sleep(delay)

        # ファイルに書き込む
        f.write(chunk_str + "\n")

send.send_data = ("wireless_fin")

end_time = time.time()  # プログラム終了時刻を記録
execution_time = end_time - start_time  # 実行時間を計算

print("実行時間:", execution_time, "秒")
print("データを", output_filename, "に保存しました。")




