import sensor,image,lcd,time
import KPU as kpu
from machine import UART
from fpioa_manager import fm
lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(1)
sensor.run(1)
task = kpu.load("/sd/yolov5.kmodel")
f=open("anchors.txt","r")
anchor_txt=f.read()
L=[]
for i in anchor_txt.split(","):
	L.append(float(i))
anchor=tuple(L)
f.close()
f=open("lable.txt","r")
lable_txt=f.read()
lable = lable_txt.split(",")
f.close()
fm.register(9, fm.fpioa.UART1_TX, force=True)
fm.register(10, fm.fpioa.UART1_RX, force=True)
uart_A = UART(UART.UART1, 9600, 8, 1, 0, timeout=1000, read_buf_len=4096)
anchor = (0.1766, 0.1793, 0.4409, 0.3797, 0.6773, 0.5954, 1.0218, 0.9527, 2.158, 1.6841)
sensor.set_windowing((224, 224))
a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)
classes=["9","1","4","2","3","8","5","6","7" ]
temp = 0
model = 0
time.sleep(2)
List_score01 = [0]*8
while(True):
	img = sensor.snapshot()
	if(uart_A.any()):
		temp  = uart_A.read(1)
	if(temp == b'U'):
		model = 1
	elif(temp == b'f'):
		model = 0
	if model == 1:
		code = kpu.run_yolo2(task, img)
		if code:
			for i in code:
				a = img.draw_rectangle(i.rect())
				lcd.display(a)
				List_score01[int(classes[i.classid()]) - 1] += 1
				if (List_score01[0] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X01,0X0a])
					uart_A.write(data)
					print(919)
				if (List_score01[1] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X02,0X0a])
					uart_A.write(data)
					print(929)
				if (List_score01[2] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X03,0X0a])
					uart_A.write(data)
					print(939)
				if (List_score01[3] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X04,0X0a])
					uart_A.write(data)
					print(949)
				if (List_score01[4] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X05,0X0a])
					uart_A.write(data)
					print(959)
				if (List_score01[5] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X06,0X0a])
					uart_A.write(data)
					print(969)
				if (List_score01[6] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X07,0X0a])
					uart_A.write(data)
					print(979)
				if (List_score01[7] >= 5):
					List_score01 = [0]*8
					data = bytearray([0xFF,0XFF,0X08,0X0a])
					uart_A.write(data)
					print(989)
		else:
			lcd.display(img)
	else:
		lcd.display(img)
