import sensor, image,time,utime#获取库
from fpioa_manager import fm
from board import board_info
import lcd
#from pyb import UART         #开启串口
from Maix import GPIO
from machine import UART
yellow_threshold =(94, 27, -86, 12, -5, -40)#括号里面是颜色阈值(53, 83, 22, 53, -7, 18)
brown_threshold = (10, 100, 77, -14, 25, 115)
num = 0
#摄像头初始化
sensor.reset()
sensor.set_hmirror(True) #镜像
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)   #后置拍摄模式
sensor.skip_frames(10) # Let new settings take affect.
#sensor.set_auto_gain(False) # 关闭自动自动增益。默认开启的，在颜色识别中，一定要关闭白平衡。
#sensor.set_auto_whitebal(False)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(8, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
lcd.init()

blue_threshold = (26, 70, -6, 28, -128, -15)

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

#sensor.snapshot(1.8)#去鱼眼化
#LCD初始化
clock = time.clock() # Tracks FPS.
img = sensor.snapshot()
K=5000#the value should be measured K=length*Lm # 实际的大小=K2*直径的像素
K2=10.5/101#QQVGA模式下K2=10.5/139  #QVGA模式下K2=10.5/279
#blobs = img.find_blobs([yellow_threshold], x_stride=5, y_stride=5, invert=False, area_threshold=10, pixels_threshold=25, merge=False, margin=0, threshold_cb=None, merge_cb=None)#调用颜色阈值
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([brown_threshold],pixels_threshold = 400,area_threshold = 200,margin=5,merge = False)

    if blobs:

    #如果找到了目标颜色
        max_temp = 0
        max_obj = blobs[0]
        for b in blobs: #循环效果不好，会有很多误识别，采用单个矩形采样方便返回坐标
        ##迭代找到的目标颜色区域
            if max_temp<=b.area():
                max_temp = b.area()
                max_obj = b
            if max_temp>=b.area():
                pass
            max_blob = find_max(blobs)
            x = max_blob[0]
            y = max_blob[1]
            width = max_blob[2]
            height = max_blob[3]
            img.draw_rectangle([x,y,width,height]) # rect
                #用矩形标记出目标颜色区域
            img.draw_cross(max_blob[5],max_blob[6]) # cx, cy
                #在目标颜色区域的中心画十字形标记
            lcd.display(img)#lcd屏幕显示
        num=10*100000000+max_temp*1000+max_blob[5]
        #uart.write("{}".format(ret))
        #uart.write("{}".format(num)+'\r\n')
        #print(b[5])

        #print(ret)
    if len(blobs)==0:
        uart.write('\n'+'0000000000'+'\r')
        time.sleep(0.001)
        #print(0)

    else:
        uart.write('\n'+"{}".format(num)+'\r')
        time.sleep(0.001)
        #print(num)
        #print(clock.fps())



