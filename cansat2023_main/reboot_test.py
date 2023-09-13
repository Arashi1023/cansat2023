import time

# try:
#     while True:
#         print("a")
#         time.sleep(1)
# except KeyboardInterrupt:
#     print("kill")

import log

reboot_log = log.Log(dir='log/reboot_log', filename='reboot_test', t_start=0, columns=['reboot_test'])

reboot_checker = 0

a = time.time()

while time.time() - a < 30:
    reboot_log.save_log(reboot_checker)
    reboot_checker += 1
    time.sleep(1)



