import libs.gps as gps

while True:
    lat_test, lon_test = gps.location()
    if lat_test == 0 and lon_test == 0:
        print('Waiting for GPS...')
    elif lat_test != 0 and lon_test != 0: #0だった場合はGPSが取得できていないので再取得
        print('GPS received')
        break