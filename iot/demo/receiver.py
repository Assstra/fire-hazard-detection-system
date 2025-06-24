import serial
import os

SerChoice = 2
fileAddress = r'receiveData/data001.txt'
dataList: list[bytes] = []

if SerChoice == 2:
    receiverDevice = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

elif SerChoice == 1:
    receiverDevice = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

while True:
    data = receiverDevice.read(1024)
    if data.decode()!='OK\r\n' and data:

        data = data.decode().replace('\r\n', '').encode()
        print('Message received: "'+data.decode()+'"')
        while len(data) != 0:
            if len(data) > 48:
                dataList.append(data[0:48])
                data = data[48:]
            else:
                dataList.append(data)
                data = b''

    for i in dataList:
        if os.path.isfile(fileAddress):
            with open(fileAddress, 'a') as data:
                data.write(i.decode())

        else:
            with open(fileAddress, 'w') as data:
                data.write(i.decode())
        
    dataList = []
