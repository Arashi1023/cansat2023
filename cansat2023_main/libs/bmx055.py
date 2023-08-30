# -*- coding: utf-8 -*-
import smbus
import time

ACC_ADDRESS = 0x19
ACC_REGISTER_ADDRESS = 0x02
GYR_ADDRESS = 0x69
GYR_REGISTER_ADDRESS = 0x02
MAG_ADDRESS = 0x13
MAG_REGISTER_ADDRESS = 0x42

i2c = smbus.SMBus(1)

def bmx055_setup():
	print("bmx055_setup")
	# --- BMX055ãSetup --- #
	#Initialize ACC
	try:
		i2c.write_byte_data(ACC_ADDRESS, 0x0F, 0x03)	#Acc Scale  datasheet p57
		time.sleep(0.1)
		i2c.write_byte_data(ACC_ADDRESS, 0x10, 0x0F)	#Acc PMU  datasheet p57
		time.sleep(0.1)
		i2c.write_byte_data(ACC_ADDRESS, 0x11, 0x00)	#datasheet p58
		time.sleep(0.1)
	except:
		time.sleep(0.1)
		print("BMX055 Setup Error")
		i2c.write_byte_data(ACC_ADDRESS, 0x0F, 0x03)	#Acc Scale  datasheet p57
		time.sleep(0.1)
		i2c.write_byte_data(ACC_ADDRESS, 0x10, 0x0F)	#Acc PMU  datasheet p57
		time.sleep(0.1)
		i2c.write_byte_data(ACC_ADDRESS, 0x11, 0x00)	#datasheet p58
		time.sleep(0.1)

	#Initialize GYR
	try:
		i2c.write_byte_data(GYR_ADDRESS, 0x0F, 0x00)	#Gyro Scale  datasheet p99
		time.sleep(0.1)
		i2c.write_byte_data(GYR_ADDRESS, 0x10, 0x07)	#Acc PMU  datasheet p100
		time.sleep(0.1)
		i2c.write_byte_data(GYR_ADDRESS, 0x11, 0x00)	#datasheet p100
		time.sleep(0.1)
	except:
		time.sleep(0.1)
		print("BMX055 Setup Error")
		i2c.write_byte_data(GYR_ADDRESS, 0x0F, 0x00)	#Gyro Scale  datasheet p99
		time.sleep(0.1)
		i2c.write_byte_data(GYR_ADDRESS, 0x10, 0x07)	#Acc PMU  datasheet p100
		time.sleep(0.1)
		i2c.write_byte_data(GYR_ADDRESS, 0x11, 0x00)	#datasheet p100
		time.sleep(0.1)

	#Initialize MAG
	try:
		data = i2c.read_byte_data(MAG_ADDRESS, 0x4B)	#datasheet p134
		if(data == 0):
			i2c.write_byte_data(MAG_ADDRESS, 0x4B, 0x83)
			time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4B, 0x01)	#datasheet p134
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4C, 0x38)	#datasheet p135
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4E, 0x84)	#datasheet p137
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x51, 0x04)	#datasheet p139
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x52, 0x0F)	#datasheet p139
		time.sleep(0.1)
	except:
		time.sleep(0.1)
		print("BMX055 Setup Error")
		data = i2c.read_byte_data(MAG_ADDRESS, 0x4B)	#datasheet p134
		if(data == 0):
			i2c.write_byte_data(MAG_ADDRESS, 0x4B, 0x83)
			time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4B, 0x01)	#datasheet p134
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4C, 0x38)	#datasheet p135
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x4E, 0x84)	#datasheet p137
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x51, 0x04)	#datasheet p139
		time.sleep(0.1)
		i2c.write_byte_data(MAG_ADDRESS, 0x52, 0x0F)	#datasheet p139
		time.sleep(0.1)

def acc_dataRead():
	print("acc_dataRead")
	# --- Read Acc Data --- #
	accData = [0, 0, 0, 0, 0, 0]
	value = [0.0, 0.0, 0.0]
	for i in range(6):
		try:
			accData[i] = i2c.read_byte_data(ACC_ADDRESS, ACC_REGISTER_ADDRESS+i)
		except:
			pass
			print(" acc_data_read_error")
		# accData[i] = i2c.read_byte_data(ACC_ADDRESS, ACC_REGISTER_ADDRESS+i)
		#accData[i] = i2c.read_byte_data(ACC_ADDRESS, ACC_REGISTER_ADDRESS)
		#i2c.close()

	for i in range(3):
		value[i] = (accData[2*i+1] * 16) + (int(accData[2*i] & 0xF0) / 16)
		value[i] = value[i] if value[i] < 2048 else value[i] - 4096
		value[i] = value[i] * 0.0098 * 1
	
	#print("acc_dataRead")

	return value

def gyr_dataRead():
	print("gyr_dataRead")
	# --- Read Gyro Data --- "
	gyrData = [0, 0, 0, 0, 0, 0]
	value = [0.0, 0.0, 0.0]
	for i in range(6):
		try:
			gyrData[i] = i2c.read_byte_data(GYR_ADDRESS, GYR_REGISTER_ADDRESS+i)
		except:
			pass
			print("gyr_data_read_error")

	for i in range(3):
		value[i] = (gyrData[2*i+1] * 256) + gyrData[i]
		value[i] = value[i] - 65536 if value[i] > 32767 else value[i]
		value[i] = value[i] * 0.0038 * 16

	return value

def mag_dataRead():
	print("mag_dataRead")
	# --- Read Mag Data --- #
	magData = [0, 0, 0, 0, 0, 0, 0, 0]
	value = [0.0, 0.0, 0.0]
	for i in range(8):
		try:
			magData[i] = i2c.read_byte_data(MAG_ADDRESS, MAG_REGISTER_ADDRESS + i)
		
		# except:
		# 	# pass
		# 	print("mag_data_read_error")
		except Exception as e:
			print(e)
			import traceback
			traceback.print_exc()

	for i in range(3):
		if i != 2:
			value[i] = ((magData[2*i+1] *256) + (magData[2*i] & 0xF8)) / 8
			if value[i] > 4095:
				value[i] = value[i] - 8192
		else:
			value[i] = ((magData[2*i+1] * 256) | (magData[2*i] & 0xF8)) / 2
			if value[i] > 16383:
				value[i] = value[i] - 32768

	return value

def bmx055_read():
	print("bmx055_read")
	# --- Read BMX055 Data --- #
	accx, accy, accz = acc_dataRead()
	gyrx, gyry, gyrz = gyr_dataRead()
	magx, magy, magz = mag_dataRead()

	#print("[%f, %f, %f] " % (accx, accy, accz), end="")
	#print("[%f, %f, %f] " % (gyrx, gyry, gyrz), end="")
	#print("[%f, %f, %f] " % (magx, magy, magz), end="")
	#print()

	value = [accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz]

	# --- Round Data --- #
	for i in range(len(value)):
		if value[i] is not None:
			value[i] = round(value[i], 4)

	return 	value

if __name__ == '__main__':
	try:
		bmx055_setup()
		time.sleep(0.2)
		while 1:
			bmxData = bmx055_read()
			print(bmxData)
			time.sleep(1)

	except KeyboardInterrupt:
		print()
	except Exception as e:
		print(e)
		
#i2c.close()
