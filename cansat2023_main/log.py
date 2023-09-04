'''
ログ関係のモジュール 田口作成 2023/8/25ß
'''

import os
import datetime
import time

class Logger:
    def __init__(self, dir, filename, t_start, columns: list):
        '''
        Parameters
        ----------
        dir : str
            ログを保存するディレクトリ
        filename : str
            ログのファイル名
        t_start : float
            ログを取り始めた時間（ミッション開始時間）
        *columns : list
            ログのカラム名
        '''
        if os.path.exists(dir):
            pass
        else:
            os.makedirs(dir)
        
        file_num = len(os.listdir(dir))
        self.path = dir + '/' + filename + '_' + str(file_num).zfill(4) + '.txt'
        self.t_start = t_start

        columns_txt = 'datetime(PDT),elapsed time,' + ','.join(columns) + '\n'
        with open(self.path, mode='w') as f:
            f.write(columns_txt)

    def save_log(self, *datas):
        '''
        ログを保存
        Parameters
        ----------
        *datas : list
        '''
        self.log_ = [datetime.datetime.now(), time.time()-self.t_start]
        for data in datas:
            if isinstance(data, list):
                for d in data:
                    self.log_.append(d)
            else:
                self.log_.append(data)
        log_txt = ",".join(map(str, self.log_)) + "\n"
        with open(self.path, mode='a') as f:
            f.write(log_txt)

if __name__ == '__main__':
    '''
    以下テスト用プログラム
    '''
    print(datetime.datetime.now())
    print(time.time())

    release = Logger(dir='../../logs/log_test2', filename='test', t_start=30, columns=['num', 'num2', 'num3'])

    release.save_log('a', [1,2])