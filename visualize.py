from socket import *
import time
import cv2
import numpy as np

# 窗口属性----白底黑字，640*640
shape = (640,640)
color_black = (0,0,0)
window = np.ones(shape)*255
window = np.array(window, np.uint8)

# 字体设置----字体，大小，粗细
font = cv2.FONT_HERSHEY_SIMPLEX
size = 0.6
thick = 2

# 初始化窗口
positions = [(2,80,80), (3,400,80), (4,80,400), (5,400,400)]
ty, w, h = 40, 240, 240
dx, dy = 40, 40
for (id,x,y) in positions:
    cv2.putText(window, "4-{}".format(id), (x+80,y), font, size, color_black, thick)
    cv2.putText(window, "NODE:", (x,y+ty*0), font, size, color_black, thick)
    cv2.putText(window, "TEMP:", (x,y+ty*1), font, size, color_black, thick)
    cv2.putText(window, "HUMD:", (x,y+ty*2), font, size, color_black, thick)
    cv2.putText(window, "ILLU:", (x,y+ty*3), font, size, color_black, thick)
    cv2.putText(window, "ADC :", (x,y+ty*4), font, size, color_black, thick)
    cv2.rectangle(window, (x-dx,y-dy), (x-dx+w,y-dy+h), color_black, 1)

# TCP传输设置
host = '192.168.175.46'
port = 8888
buffsize = 1024
ADDR = (host,port)
tctime = socket(AF_INET,SOCK_STREAM)
tctime.bind(ADDR)
tctime.listen(3)
# Node类
class Node():
    def __init__(self, id):
        self.x = positions[id-2][1] + 80
        self.y = positions[id-2][2]
        self.wdu = '0'
        self.sdu = '0'
        self.gzd = '0'
        self.adc = '0'

    def Update(self, datastr):
        strlist = datastr.split(',')
        datalist = []
        for i in strlist:
            i = i.strip()
            data = i.split(':')[-1]
            datalist.append(data)
        self.wdu = datalist[1]
        self.sdu = datalist[2]
        self.gzd = datalist[3]
        self.adc = datalist[4]

# 存储变量
nodes = []
for i in range(4):
    node = Node(i+2)
    nodes.append(node)

while True:
    print('Wait for connection ...')
    tctimeClient,addr = tctime.accept()
    print("Connection from :",addr)
    time.sleep(1)
    while True:
        data = tctimeClient.recv(buffsize)
        data1 = str(data.decode())
        print(data1)

        node_id = int(data1.split(',')[0][-1])
        nodes[node_id-2].Update(data1)

        img = window.copy()
        for node in nodes:
            cv2.putText(img, "{}".format(node.wdu), (node.x,node.y+ty*1), font, size, color_black, thick)
            cv2.putText(img, "{}".format(node.sdu), (node.x,node.y+ty*2), font, size, color_black, thick)
            cv2.putText(img, "{}".format(node.gzd), (node.x,node.y+ty*3), font, size, color_black, thick)
            cv2.putText(img, "{}".format(node.adc), (node.x,node.y+ty*4), font, size, color_black, thick)

        if not data:
            break
        cv2.imshow('data', img)
        if cv2.waitKey(1000) & 0xff == ord('q'):
            cv2.destroyAllWindows()
            break
    tctimeClient.close()

tctimeClient.close()
