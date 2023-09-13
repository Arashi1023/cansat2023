import time

try:
    while True:
        print("a")
        time.sleep(1)
except KeyboardInterrupt:
    print("kill")