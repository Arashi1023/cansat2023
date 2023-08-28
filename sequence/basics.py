'''
制御に必要な基本的な処理まとめ
'''

def standarize_angle(angle):
    '''
    角度を-180～180度に収める関数
    '''
    angle = angle % 360
    
    if angle >180:
        angle -= 360
    elif angle < -180:
        angle += 360

    return angle



if __name__ == "__main__":
    angle = int(input())

    print(standarize_angle(angle))  