import cv2
import numpy as np
import matplotlib.pyplot as plt
from picamera import PiCamera
from time import sleep

def split_image_vertical(image, num_splits):
    height, width = image.shape[:2]
    split_width = width // num_splits

    split_images = []
    for i in range(num_splits):
        start_x = i * split_width
        end_x = start_x + split_width if i < num_splits - 1 else width
        split_images.append(image[:, start_x:end_x])

    return split_images

def calculate_histogram(image):
    hist = cv2.calcHist([image], [0], None, [256], [0, 256])
    hist = hist.flatten()
    return hist

def is_object_detected(hist):
    hist_valid = hist[:50]  # 50以下の輝度値の部分のヒストグラムを抽出
    if np.any((hist_valid >= 30)):
        return True
    else:
        return False

# カメラ初期化
camera = PiCamera()

# 画像撮影と保存
image_path = "image.jpg"  # 保存先のファイル名を指定
camera.start_preview()
sleep(5)  # カメラが安定するまで待機
camera.capture(image_path)
camera.stop_preview()
print(f"画像を {image_path} に保存しました。")

# 画像読み込み
image = cv2.imread(image_path, 0)  # グレースケールで読み込み

# 画像を縦方向に分割
num_splits = 9  # 分割数
split_images = split_image_vertical(image, num_splits)

# ヒストグラムを比較し特異な分布を判断
for i in range(1, num_splits):
    current_image = split_images[i]
    current_hist = calculate_histogram(current_image)
    
    if is_object_detected(current_hist):
        print(f"分割画像 {i} で物体が検知されました。")
    else:
        print(f"分割画像 {i} に物体は検知されませんでした。")

    # 分割画像を出力
    output_path = f"output_image_{i}.jpg"
    cv2.imwrite(output_path, current_image)
    print(f"分割画像 {i} を {output_path} として保存しました。")

# ヒストグラムを計算
histograms = []
for i in range(1, num_splits):
    current_image = split_images[i]
    current_hist = calculate_histogram(current_image)
    histograms.append(current_hist)

# ヒストグラムを表示
# for i, hist in enumerate(histograms):
#     plt.figure()
#     plt.plot(hist)
#     plt.title(f"分割画像 {i + 1} のヒストグラム")
#     plt.xlabel("輝度値")
#     plt.ylabel("頻度")

# すべてのヒストグラムを表示
# plt.show()
