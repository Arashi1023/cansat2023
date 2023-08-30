import tflite_runtime.interpreter as tflite
import numpy as np
from PIL import Image
import cv2
import time

import libs.take as take

class DetectPeople():
    def __init__(self, model_path,):
        # self.model_path = model_path
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.pix = 224 # モデルに入力するサイズ。変更しない方がよい。
        self.interpreter.allocate_tensors()
        # 入力と出力テンソルの情報を取得
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()   
        self.class_names = ["hito", "other"]

    def predict(self, image_path):
        image = cv2.imread(image_path)
        
        # 画像の前処理
        image = cv2.resize(image, (self.pix, self.pix))
        image = np.array(image) / 255.0
        image = np.expand_dims(image, axis=0)

        
        # 入力データのセットアップ
        input_data = image.astype(np.float32)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # 推論の実行
        self.interpreter.invoke()
        # 出力データの取得
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        # 予測結果の処理
        pro_people = output_data[0,0]
        print(f'人である確率:{pro_people}')
        # predicted_class = np.argmax(output_data, axis=1)
        # predicted_label = self.class_names[predicted_class[0]]

        # print("予測結果:", predicted_label)
        # print(output_data)

        # print(predicted_class)
        return pro_people
    
if __name__ == "__main__":
    ML_people = DetectPeople(model_path="../model_mobile.tflite" )
    # image_path = 'imgs/hiroyuki.jpg'
    # image_path = 'imgs/saru.jpg'
    # ML_people.predict(image_path)
    while 1:
        img_path = take.picture('../../imgs/human_detect/image', 320, 240)
        # img_path = './imgs/overhead_people.jpg'
        ML_people.predict(image_path=img_path)
        # time.sleep(3)