import PID
import bmx055
import basics
import calibration
import motor

magx_off, magy_off = 0, 0

def main(magx_off=700, magy_off=1080):
    mag_data = bmx055.mag_dataRead()
    mag_x, mag_y = mag_data[0], mag_data[1]
    rover_azimuth = calibration.angle(mag_x, mag_y, magx_off, magy_off)
    rover_azimuth = basics.standarize_angle(rover_azimuth)
    target_azimuth = rover_azimuth

    theta_array = [0]*5
    PID.PID_run(target_azimuth, magx_off, magy_off, theta_array=theta_array, loop_num=20)
    motor.deceleration(15, 15)
    motor.motor_stop(0.2)

if __name__ == '__main__':
    motor.setup()
    main()