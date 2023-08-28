# -*- coding: utf-8 -*-

import serial
import binascii
import signal
import sys
import platform
import pigpio
import time
portnumber = '/dev/ttyAMA0'

def signal_handler(signal, frame):
	# --- Command of "Ctrl + C" --- #
	#print('exit')
	sys.exit()

def setSerial(mybaudrate = 19200):
	# --- Configutation of serial.Serial --- #
	#global com
	com = serial.Serial(
		port	 = portnumber,
		baudrate = mybaudrate,
		bytesize = serial.EIGHTBITS,
		parity   = serial.PARITY_NONE,
		timeout  = 3,
		xonxoff  = True,
		rtscts   = False,
		writeTimeout = None,
		dsrdtr	   = False,
		interCharTimeout = None)

	# --- Clear Buffer --- #
	com.flushInput()
	com.flushOutput()
	return com

def Close(mybaudrate=19200):
	com=setSerial(mybaudrate)
	com.flushInput()
	com.flushOutput()
	com.close()

def Rdid(mybaudrate = 19200):
	# --- Read Own ID --- #
	com = setSerial(mybaudrate)
	#print(com)
	com.flushInput()
	com.write(b'RDID' + b'\r\n')
	com.flushOutput()
	data = com.readline().strip()
	#print(dir(com))
	#print('固有ID:' + str(com.readline().strip()))
	com.close()
	return data

def Rrid(mybaudrate = 19200):
	# --- Read Recieve Device ID --- #
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'RRID' + b'\r\n')
	com.flushOutput()
	data = []
	while 1:
		id = com.readline().strip()
		if id == b'':
			break
		data.append(id)
		#print('受信ID:' + str(com.readline().strip()))
	com.close()
	return data

def Stch(setch, mybaudrate = 19200):
	# --- Configuration of Communication Channel --- #
	'''
		01 920.6MHz	09 922.2MHz
		02 920.8MHz	10 922.4MHz
		03 921.0MHz	11 922.6MHz
		04 921.2MHz	12 922.8MHz
		05 921.4MHz	13 923.0MHz
		06 921.6MHz	14 923.2MHz
		07 921.8MHz	15 923.4MHz
		08 922.0MHz
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'ENWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'Stch ' + setch.encode('utf-8') + b'\r\n')
	com.flushOutput()
	data = com.readline()
	com.write(b'DSWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.close()
	return data

def Rdch(mybaudrate = 19200):
	# --- Read Communication Channel --- #
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'RDCH' + b'\r\n')
	com.flushOutput()
	ch = com.readline().strip()
	if ch in ['01']:
		pass
		#print('無線通信チャンネル:' + '01 920.6MHz')
	elif ch in ['02']:
		pass
		#print('無線通信チャンネル:' + '02 920.8MHz')
	elif ch in ['03']:
		pass
		#print('無線通信チャンネル:' + '03 921.0MHz')
	elif ch in ['04']:
		pass
		#print('無線通信チャンネル:' + '04 921.2MHz')
	elif ch in ['05']:
		pass
		#print('無線通信チャンネル:' + '05 921.4MHz')
	elif ch in ['06']:
		pass
		#print('無線通信チャンネル:' + '06 921.6MHz')
	elif ch in ['07']:
		pass
		#print('無線通信チャンネル:' + '07 921.8MHz')
	elif ch in ['08']:
		pass
		#print('無線通信チャンネル:' + '08 922.0MHz')
	elif ch in ['09']:
		pass
		#print('無線通信チャンネル:' + '09 922.2MHz')
	elif ch in ['10']:
		pass
		#print('無線通信チャンネル:' + '10 922.4MHz')
	elif ch in ['11']:
		pass
		#print('無線通信チャンネル:' + '11 922.6MHz')
	elif ch in ['12']:
		pass
		#print('無線通信チャンネル:' + '12 922.8MHz')
	elif ch in ['13']:
		pass
		#print('無線通信チャンネル:' + '13 923.0MHz')
	elif ch in ['14']:
		pass
		#print('無線通信チャンネル:' + '14 923.2MHz')
	elif ch in ['15']:
		pass
		#print('無線通信チャンネル:' + '15 923.4MHz')
	#com.readline()
	com.close()
	return ch

def Sbrt(setbaudrate, mybaudrate = 19200):
	# --- Configuration of Baudrate --- #
	'''
		0 1200bps
		1 2400bps
		2 4800bps
		3 9600bps
		4 19200bps
		5 38400bps
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'ENWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'SBRT ' + setbaudrate.encode('utf-8') + b'\r\n')
	com.flushOutput()
	data = com.readline()
	com.write(b'DSWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.close()
	return data

def Rdrs(mybaudrate = 19200):
	# --- Read RSSI --- #
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'RDRS' + b'\r\n')
	com.flushOutput()
	#print('信号強度:' + str(com.readline().strip()))
	#com.readline()
	com.close()

def read(mybaudrate=19200):
	re=""
	try:
		com = setSerial(mybaudrate)
		com.flushInput()
		re = str(com.readline().decode('utf-8').strip())
		com.flushOutput()
	except Exception:
		re = ""
		#print("no data")
	return re

def Strt(setspeed, mybaudrate = 19200):
	#Configuration of Communication Speed --- #
	'''
		1 高速通信モード(無線通信速度 50kbps)
		2 長距離モード(無線通信速度 1.25kbps)
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'ENWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'STRT ' + setspeed.encode('utf-8') + b'\r\n')
	com.flushOutput()
	data = com.readline()
	com.write(b'DSWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.close()
	return data

def Rdrt(mybaudrate = 19200):
	'''
	無線通信速度の読み出し
	mybaudrate:現在のボーレート
		1 高速通信モード(無線通信速度 50kbps)
		2 長距離モード(無線通信速度 1.25kbps)
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'RDRT' + b'\r\n')
	com.flushOutput()
	sp = com.readline().strip()
	if sp in ['1']:
		print('1:fastmode')
	elif sp in ['2']:
		print("2:distancemode")
	#com.readline()
	com.close()
	return sp

def Srid(args, mybaudrate = 19200):
	'''
	ペアリング
	mybaudrate:ボーレート
	args:ペアリングしたいID(文字列にすること)
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'ENWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'SRID ' + args.encode('utf-8') + b'\r\n')
	com.flushOutput()
	data = com.readline()
	com.write(b'DSWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.close()
	return data

def Erid(mybaudrate = 19200):
	'''
	ペアリングの削除
	全て削除されるため注意!
	mybaudrate:ボーレート
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'ENWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'ERID' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.write(b'DSWR' + b'\r\n')
	com.flushOutput()
	com.readline()
	com.close()

def Send(args, mybaudrate = 19200):
	'''
	送信
	mybaudrate:ボーレート
	args:送信したい文字列 (数字の場合も文字列型にすること)
	'''
	global com
	#print(binascii.b2a_hex(args.encode('utf-8')))
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'TXDA' + binascii.b2a_hex(args.encode('utf-8')) + b'\r\n')
	data = com.readline()
	com.flushOutput()
	#print(com.readline().strip())
	#com.close()
	return data

def IMSend(byte, mybaudrate = 19200):
	'''
	送信
	mybaudrate:ボーレート
	args:送信したい文字列 (数字の場合も文字列型にすること)
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'TXDA' + binascii.b2a_hex(byte) + b'\r\n')
	data = com.readline()
	#time.sleep(0.08)
	com.flushOutput()
	#print(str(binascii.b2a_hex(byte)))
	#print(com.readline().strip())
	com.close()
	return data


def Reception(mybaudrate = 19200):
	'''
	受信
	アスキーコードから文字列に変換したものを返す
	mybaudrate:ボーレート
	'''
	com = setSerial(mybaudrate)
	com.flushInput()

	text = ""
	cngtext = ""

	try:
		text = com.readline().decode('utf-8').strip() #受信と空白の削除
		com.close()
		text = text.replace("\r\n","")
		text = text.split(":")[1]
		text = text.split(",")

		for x in text:
			cngtext += chr(int(x,16))

	except Exception:
		cngtext = ""
		#print("not input data")

	return cngtext
	com.close()

def Repeater(mybaudrate = 19200):
	'''
	中継機化
	mybaudrate:ボーレート
	'''
	signal.signal(signal.SIGINT, signal_handler)

	while True:
		data = Reception(mybaudrate)
		if len(data) != 0:
			print("input data:", data)
			Send(19200, data)

def Rprm(mybaudrate = 19200):
	'''
	パラメータ一括読み出し
	mybaudrate:現在のボーレート
	'''
	com = setSerial(mybaudrate)
	com.flushInput()
	com.write(b'RPRM' + b'\r\n')
	com.flushOutput()
	for i in range(10):
		print(com.readline().strip())

	com.readline()
	com.close()


if __name__ == '__main__':
	pi=pigpio.pi()
	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)
	time.sleep(2)
	i = 0
	while 1:
		print("P" + str(i))
		data = Send("P" + str(i))
		#print(data)
		#if data == b'OK\r\n':
			#print("OK")
		i = i + 1
		#time.sleep(0.5)
		if i == 10:
			i = 0
		time.sleep(1)
	#Reception()
