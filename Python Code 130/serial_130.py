import serial
import struct
from time import time
from time import sleep

import random

# A delay to reduce CPU usage, but will make serial comms slower
fast_wait = 0.01
# fast_wait = 0.0
main_wait = 0.1

# Acknowledge byte from Arduino
# This should be something not in normal ASCII, so above 127
s_ack = b'\x06'
s_err = b'\x15'

verbose = True

error_count = 0


# ---------------------------------
# Serial data handling
def send_int8(int_var):
	# TODO
	#  We might be sending bytes from an already generated profile?
	s.write(int_var.to_bytes(1, byteorder='little'))


def send_int16(int_var):
	s.write(int_var.to_bytes(2, byteorder='little'))


def send_int32(int_var):
	s.write(int_var.to_bytes(4, byteorder='little'))


def send_float(float_var):
	def float_binary(num):
		# https://stackoverflow.com/questions/16444726/binary-representation-of-float-in-python-bits-not-hex
		return ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))

	def bitstring_to_bytes(bits):
		# https://stackoverflow.com/questions/32675679/convert-binary-string-to-bytearray-in-python-3
		return int(bits, 2).to_bytes((len(bits) + 7) // 8, byteorder='little')

	float_bits = float_binary(float_var)
	s.write(bitstring_to_bytes(float_bits))


def receive_int8():
	while s.in_waiting < 1:
		sleep(fast_wait)
	return ord(s.read(1))


def receive_int16():
	while s.in_waiting < 2:
		sleep(fast_wait)
	return struct.unpack('>H', s.read(2))[0]


def receive_int32():
	while s.in_waiting < 4:
		sleep(fast_wait)
	return struct.unpack('<I', s.read(4))[0]


def receive_float():
	while s.in_waiting < 4:
		sleep(fast_wait)
	return struct.unpack('f', s.read(4))[0]


def acknowledge():
	# Function to print remaining serial buffer and wait for Arduino acknowledge
	# Return True if 's_ack' byte received
	# Return False if 's_err' byte received
	# print("Clearing...")

	global error_count

	timeout_time = time() + 3

	while True:
		if s.in_waiting > 0:
			read_byte = s.read()
			# print(read_byte)
			if read_byte == s_ack:
				# print("Acknowledged")
				if verbose:
					print(" .", end='')
				return True
			elif read_byte == s_err:
				error_count += 1
				if verbose:
					print(" X", end='')
				return False
			else:
				print(read_byte.decode('ascii'), end='')
		if time() > timeout_time:
			if verbose:
				print("Timeout!")
			return False

		sleep(fast_wait)


def send_func(num):
	# Function to send the function number to trigger serial command function
	# Error checking adds following to make 255:
	#   check byte = 255 - num
	#   func byte  = num
	send_int8(255 - num)
	send_int8(num)
	if acknowledge():
		return True
	else:
		return False


# ---------------------------------
# Functions to send data

def cmd_halt():
	# Function to halt the Arduino and make it safe to reset (temp fix for PCB issue)
	if send_func(1):
		print("Halted!")


def cmd_alive():
	# Function to get uptime of Arduino in milliseconds

	print("Get Arduino uptime", end='')

	if not send_func(2):
		return -1

	uptime = receive_int32()

	# print("Uptime received!")
	print("\nUptime:", uptime)


def cmd_load_config():
	# Function to reload Arduino config from FRAM
	if send_func(3):
		print("Config loaded!")


def cmd_write_fram(address, data):
	if verbose:
		print("Writing FRAM", end='')

	if not send_func(4):
		return False

	# Send data
	for i in range(len(data)):
		send_int16(address + i)
		send_int8(data[i])
		if not acknowledge():
			break

	# End data transfer
	send_int16(65535)
	send_int8(255)
	if not acknowledge():
		return False

	if verbose:
		print("FRAM written")
	return True


def cmd_write_temp(temperatures):
	# Function to set temperatures 5 - 8 on Arduino

	if not send_func(5):
		return False

	for i in range(4):
		send_int8(temperatures[i])  # Send temperature

	if not acknowledge():
		return False
	return True


def cmd_write_rgb(prof, link, use_random, smooth, brightness, seq_len, rate, red, green, blue):
	# Function to send RGB profile

	print("Sending RGB profile", end='')

	if not send_func(6):
		return False

	# Send profile number (1 to 3)
	send_int8(prof)
	if not acknowledge():
		return False

	# Send options
	send_int8(link)
	send_int8(use_random)
	send_int8(smooth)
	send_float(brightness)
	send_int8(seq_len)
	send_float(rate)

	# Arduino will now send 255, wait for this to be sent
	if not acknowledge():
		return False

	# Send sequences
	for i in range(10):
		send_int16(red[i])
		send_int16(green[i])
		send_int16(blue[i])

		if not acknowledge():
			return False

	print("\nRGB profile sent!")
	return True


def cmd_write_rtc(timestamp):
	print("Updating time", end='')
	if not send_func(7):
		return False

	send_int32(timestamp)
	# receive_time = receive_int32()
	if not acknowledge():
		return False

	# print("Receive timestamp:", receive_time)
	print("RTC set")
	return True


def cmd_write_fan(prof, pwm, off, volt, change, hysteresis, default, factor, temp, speed):
	if not send_func(8):
		return False

	send_int8(prof)
	if not acknowledge():
		return False

	send_int8(pwm)
	send_int8(off)
	send_int16(volt)
	send_int8(change)
	send_int8(hysteresis)
	send_int8(default)

	if not acknowledge():
		return False

	for i in range(8):
		send_int8(factor[i])
	if not acknowledge():
		return False

	for i in range(4):
		send_int8(temp[i])
	if not acknowledge():
		return False

	for i in range(4):
		send_int16(speed[i])
	if not acknowledge():
		return False

	return True


def cmd_write_lcd(timeout, string_1, string_2):
	# Function to write 16 chars from two strings to LCD
	# Will pad with blank chars if strings are too short

	print("Update LCD", end='')

	if not send_func(9):
		return False

	send_int8(timeout)
	if not acknowledge():
		return False

	# https://stackoverflow.com/questions/20309255/how-to-pad-a-string-to-a-fixed-length-with-spaces
	# print('{:<16}'.format(string_1[:16]))
	# s.write(('{:<16}'.format(string_1[:16])).encode())
	# if not acknowledge():
	# 	return False

	string_bytes = string_1.encode('ascii')
	print(string_bytes)
	# print(len(string_bytes))
	s.write(string_bytes[0:16])
	for i in range(16 - len(string_bytes)):
		s.write(b' ')
	if not acknowledge():
		return False

	string_bytes = string_2.encode('ascii')
	print(string_bytes)
	# print(len(string_bytes))
	s.write(string_bytes[0:16])
	for i in range(16 - len(string_bytes)):
		s.write(b' ')
	if not acknowledge():
		return False

	# print('{:<16}'.format(string_2[:16]))
	# s.write(('{:<16}'.format(string_2[:16])).encode())
	# if not acknowledge():
	# 	return False

	return True


def cmd_read_rpm():
	# Function to get chosen fan channel RPM reading

	rpms = []

	# print("RPM channel:", channel)
	if not send_func(10):
		return [-1]

	for i in range(8):
		rpms.append(receive_int32())
	# print("Read int32")
	# if not acknowledge():
	# 	return [-1]

	return rpms


def cmd_read_temp():
	# Function to get all temperature values from Arduino
	# Returns all temperatures, 8 x int8
	temps = []

	if not send_func(11):
		return [-1]

	for i in range(8):
		temps.append(receive_int8())
		if temps[i] > 127:  # Simple trick to get signed int 8
			temps[i] -= 256
	# print(temperature)
	if not acknowledge():
		return [-1]

	return temps


def cmd_read_fram(address, num_bytes):
	# Function to read arbitrary bytes from FRAM

	if not send_func(12):
		return []

	data = []

	for i in range(num_bytes):
		send_int16(address + i)
		# print("Read addr", str(address + i))
		data.append(receive_int8())
		if not acknowledge():
			return []

	# End data transfer
	send_int16(65535)
	if not acknowledge():
		return []

	return data


# -------------------------------------
# Threads for handling stuff

# Wait until time for next command, and print any incoming serial data
def wait(seconds):
	wait_time = time() + seconds

	while time() < wait_time:
		# while s.in_waiting > 0:
		if s.in_waiting > 0:
			print(s.read().decode('ascii'), end='')
		else:
			sleep(main_wait)


def update_time():
	cmd_write_rtc(int(time() + 3600))


# Creates an RGB profile to send
def prep_rgb_profile():
	# random = bool(input("Random (0/1): "))
	# smooth = bool(input("Smooth (0/1): "))
	# length = int(input("Length (<10): "))
	# rate = float(input("Rate: "))
	#
	# red = []
	# green = []
	# blue = []
	# for i in range(10):
	# 	print("Sequence", i)
	# 	red.append(input("Red: "))
	# 	green.append(input("Green: "))
	# 	blue.append(input("Blue: "))

	profile = random.randint(0, 2)

	# random = 1
	# smooth = 1
	# length = 2
	# rate = 0.1
	# red = [0, 0, 100, 250, 0, 0, 0, 0, 0, 0]
	# green = [0, 0, 0, 100, 0, 0, 0, 0, 0, 0]
	# blue = [0, 0, 100, 250, 0, 0, 0, 0, 0, 0]

	rando = random.randint(0, 1)
	smooth = random.randint(0, 1)
	if rando:
		length = 2
	else:
		length = random.randint(1, 10)
	rate = 1 / random.randint(1, 10)

	red = []
	green = []
	blue = []

	# Possible issue - if random second value (position 3 from 0), might stick LED
	for i in range(10):
		red.append(random.randint(0, 4095))
		green.append(random.randint(0, 4095))
		blue.append(random.randint(0, 4095))

	# cmd_write_rgb(prof, link, use_random, smooth, brightness, seq_len, rate, red, green, blue)
	result = cmd_write_rgb(profile, 0, rando, smooth, 0.4, length, rate, red, green, blue)
	if not result:
		print("RGB profile failed!")


# Creates a set of random temperatures to send
def prep_send_temp():
	temps = []
	for i in range(4):
		temps.append(random.randint(0, 100))
	print("Update temp", end='')
	if cmd_write_temp(temps):
		print("Temp sent!")


# Creates a random fan profile to send
def prep_fan_profile():
	prof = random.randint(0, 4)
	pwm = random.randint(0, 1)
	off = random.randint(0, 1)
	volt = random.randint(0, 4095)
	change = random.randint(0, 250)
	hysteresis = random.randint(0, 10)
	default = random.randint(30, 60)

	factor = []
	for i in range(8):
		factor.append(random.randint(1, 10))

	temp = [30]
	for i in range(3):
		temp.append(random.randint(temp[i], temp[i] + 20))

	speed = [1000]
	for i in range(3):
		speed.append(random.randint(speed[i], speed[i] + 1000))

	print("Sending fan profile", end='')
	result = cmd_write_fan(prof, pwm, off, volt, change, hysteresis, default, factor, temp, speed)
	if result:
		print("Fan profile sent!")


# Creates some random data to send to imitate FRAM write
def prep_fram_write():
	address = random.randint(0, 32600)

	data = []
	for i in range(random.randint(3, 20)):
		data.append(random.randint(50, 255))

	print(data)

	cmd_write_fram(address, data)


# Reads a random number of bytes from a random address
def prep_fram_read():
	num_bytes = random.randint(1, 20)
	address = random.randint(0, 32500)

	print("Read FRAM", end='')
	cmd_read_fram(address, num_bytes)
	print("FRAM read!")


# Establish serial connection
s = serial.Serial('COM3', 115200)


if __name__ == "__main__":
	print("Fan controller serial test")

	# s = serial.Serial('COM3', 115200, timeout=5)  # Windows serial port used for Uno and Nano
	# s = serial.Serial('COM3', 115200)

	# wait(5)

	while True:
		try:
			# Print serial until a task is requested
			# Once the task is requested, run the task function

			update_time()
			cmd_halt()
			# cmd_alive()
			# cmd_load_config()
			# prep_fram_write()
			# prep_send_temp()
			# prep_rgb_profile()
			# cmd_write_rtc(int(time() + 3600))
			# prep_fan_profile()
			# cmd_write_lcd(3, "Presenting...", "  Madness")
			# print(cmd_read_rpm())
			# print(cmd_read_temp())
			# prep_fram_read()

			# print("Tasks completed!\n")
			print("Error count:", error_count)
			print("~~~~~~~~~")

			wait(5)

		except KeyboardInterrupt:
			break

	s.close()

	print("Done.")
