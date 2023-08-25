# import picamera2 as picamera
import  picamera2 as picamera
import time
import traceback
import logging
import os
#from PIL import Image
import cv2
import time

logging.getLogger('picmaera2').setLevel(logging.WARNING)
os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

   


def picture(path, width=320, height=240):
    """写真を取るための関数"""
    global latest_picture_path
    filepath = None
    def dir(path):
        """
        /dir/dir/dir/fileの時にfileの前にディレクトリが存在するか調べる関数
        引数は/dir/dir/dir/fileの形のパス
        """
        fd = path.rfind('/')
        dir = path[:fd]

        is_dir = os.path.isdir(dir)
        return is_dir
    def make_dir(path):
        """
        dir関数で調べた結果ディレクトリが存在しない場合はそのディレクトリを作成する
        """
        if not dir(path):
            fd = path.rfind('/')
            directory = path[:fd]
            os.mkdir(directory)
            print('******Directory is maked******')
        else:
            print('**Directory is exist**')
            
    def filename(f, ext):
        """
        ファイル名に番号をつけるための関数
        引数f:つけたいファイル名
        引数ext:ファイルの拡張子
        戻り値f:ファイル名+0000.拡張子
        戻り値の番号は増えていく
        """
        i = 0
        while 1:
            num = ""
            if len(str(i)) <= 4:
                for j in range(4 - len(str(i))):
                    num = num + "0"
                num = num + str(i)
            else:
                num = str(i)
            if not (os.path.exists(f + num + "." + ext)):
                break
            i = i + 1
        f = f + num + "." + ext
        return f
      
    
    try:
        make_dir(path)
        # picam2 =  Picamera2()

        with picamera.Picamera2() as camera:
            # camera.set_logging(camera.ERROR)
            camera.start_preview('')
            filepath = filename(path, 'jpg') # カメラのファイル名作成
            camera_config = camera.create_still_configuration(main={"size": (width, height)}, lores={"size": (width, height)}, display="lores")
            camera.configure(camera_config)
            t_cam_start = time.time()
            camera.start()
            #camera.rotation = 90 # カメラの画像回転
            # time.sleep(2)
            camera.capture_file(filepath) # 撮影した画像を保存
            print(time.time()-t_cam_start)
            latest_picture_path=filepath

            #画像を読み込んで回転させる
            t_start = time.time()
            image = cv2.imread(filepath)
            print('read' + time.time()-t_start)
            image = cv2.resize(image, (width, height))
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imwrite(filepath, image)
            print("time", time.time()-t_start)

            # image = Image.open(filepath)
            # rotated_image = image.rotate(90, expand=True)
            # rotated_image.save(filepath)
            
    except :
        print(traceback.format_exc())

    # # そのファイルに取得した画像を入れる
    # except picamera.exc.PiCameraMMALError:
    #     filepath = "Null"
    #     # パスが切れているときはNULL
    #     time.sleep(0.8)
    # except:
    #     print(traceback.format_exc())
    #     time.sleep(0.1)
    #     filepath = "Null"
    # # そのほかのエラーの時はNULL
    return filepath


if __name__ == '__main__':
    try:
        photoName = picture('photo/photo', 320, 240)
    except KeyboardInterrupt:
        print('stop')
    except:
        print(8888888888888888888)
        print(traceback.format_exc())
