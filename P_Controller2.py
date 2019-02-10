import serial
import os
import time


ser = serial.Serial('COM3', 115200)
ser.flushInput()
ser.flushOutput()


text_file = open("./position4.txt", 'a')
flip = 0
while True:
	dataString = str(ser.readline())
	#print (dataString)
	timestamp = int(dataString.split(",")[1])
	encoder_position = int(dataString.split(",")[2])
	CurrentCartPosition =int(dataString.split(",")[4])
	current_cart_velocity = int(dataString.split(",")[5])
	last_commanded_velocity = int(dataString.split(",")[7])
	#P Controller
	velocity = encoder_position * 200;
	#print (str(velocity))
	
	if(velocity>0):
		velocity_direction_byte = 1
	else:
		velocity_direction_byte = 0
		
	velocity = abs(velocity)
	velocity_high_byte = int((velocity & 0xFF00) >> 8)
	velocity_low_byte = int(velocity & 0xFF)
	velocity_direction_byte = int(velocity_direction_byte & 0xFF)

	command_string = bytearray([velocity_high_byte,velocity_low_byte,velocity_direction_byte,65])

	#Write the command to the serial port then log 
	ser.write(command_string)
	time_clock = time.perf_counter()
	
	print(CurrentCartPosition)
	
	text_file.write(str(timestamp) + "," + str(encoder_position) +"," + str(current_cart_velocity) + "," + str(CurrentCartPosition) +"," + str(time_clock) + '\n')
	text_file.flush()