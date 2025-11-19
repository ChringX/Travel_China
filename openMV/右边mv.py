import sensor, image, time,struct
from pyb import UART
import json
yellow_threshold=[(42, 69, -78, 46, 27, 71),(72, 99, -62, 46, 24, 102)]
red_threshold=[(7, 79, 2, 43, -5, 59),(10, 38, 24, 61, -63, 75)]
green_threshold=[(19, 36, -112, -23, -1, 39),(16, 58, -128, -27, -73, 41)]
size_threshold = 8000
def find_max(blobs):
	max_size=0
	for blob in blobs:
		if blob[2]*blob[3] > max_size:
			max_blob=blob
			max_size = blob[2]*blob[3]
	return max_size
def find_max1(blobs):
	max_size=0
	for blob in blobs:
		if blob[2]*blob[3] > max_size:
			max_blob=blob
			max_size = blob[2]*blob[3]
	return max_blob
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, 120000)
sensor.set_contrast(3)
sensor.set_brightness(0)
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)
temp=0
model=0
while True:
	img = sensor.snapshot()
	G_blobs = img.find_blobs(green_threshold,roi=(57,48,38,66))
	Y_blobs = img.find_blobs(yellow_threshold,roi=(57,48,38,66))
	R_blobs = img.find_blobs(red_threshold,roi=(57,48,38,66))
	time.sleep_ms(5)
	if(uart.any()):
	   temp  = uart.read(1)
	if(temp == b'U'):
	   model = 1
	elif(temp == b'f'):
	   model = 0
	if G_blobs:
		G_max_blob = find_max(G_blobs)
	else:
		G_max_blob=0
	if Y_blobs:
		Y_max_blob = find_max(Y_blobs)
	else:
		Y_max_blob=0
	if R_blobs:
		R_max_blob = find_max(R_blobs)
	else:
		R_max_blob=0
	if R_blobs:
		R_max_blob1 = find_max1(R_blobs)
		img.draw_rectangle(R_max_blob1.rect())
		img.draw_cross(R_max_blob1.cx(),R_max_blob1.cy())
	if Y_blobs:
		Y_max_blob1 = find_max1(Y_blobs)
		img.draw_rectangle(Y_max_blob1.rect())
		img.draw_cross(Y_max_blob1.cx(),Y_max_blob1.cy())
	if G_blobs:
		G_max_blob1 = find_max1(G_blobs)
		img.draw_rectangle(G_max_blob1.rect())
		img.draw_cross(G_max_blob1.cx(),G_max_blob1.cy())
	if model==1:
		if((G_max_blob>Y_max_blob)&(G_max_blob>R_max_blob)):
			data = bytearray([0xFF,0XFF,0X01,0X0a])
			uart.write(data)
		if((Y_max_blob>G_max_blob)&(Y_max_blob>R_max_blob)):
			data = bytearray([0xFF,0XFF,0X02,0X0a])
			uart.write(data)
		if((R_max_blob>G_max_blob)&(R_max_blob>Y_max_blob)):
			data = bytearray([0xFF,0XFF,0X03,0X0a])
			uart.write(data)