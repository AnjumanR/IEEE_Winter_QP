import serial, time
ser = serial.Serial('/dev/cu.wchusbserial1450', 9600, timeout=1)
time.sleep(1)
i = input("Enter character to start: ")
ser.write(i.encode("ascii"))
while True:
	while not ser.inWaiting():
		pass
	with open('gyro_data.txt', 'a') as my_file :
		d = ser.readline()
		data = str(d,'utf-8')
		print(data);
		my_file.write(data)
		my_file.close()

#650-300 correct
#