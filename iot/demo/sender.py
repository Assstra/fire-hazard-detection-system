import serial

SerChoice = 1
fileAddress = r'sendData/data001.txt'
ackFlag = 0
dataList: list[bytes] = []

if SerChoice == 1:
    senderDevice = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

elif SerChoice == 2:
    senderDevice = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

with open(fileAddress, 'rb')as dataSource:
    data = dataSource.read()
    
    while len(data) != 0:
        if len(data) > 48:
            dataList.append(data[0:48])
            data = data[48:]
        else:
            dataList.append(data)
            data = b''

for i in dataList:
    while ackFlag != 1:
        print('Sending message (direction 2): "'+i.decode()+'"')
        print('Data length: '+str(len(i))+'byte')
        ack = b''

        senderDevice.write(i+b'\r\n')
        ack = senderDevice.read(1024)

        ack = ack.decode().replace('\r\n', '').encode()

        print('ACK:'+ack.decode()+':')

        if ack.decode() == 'OK':
            print('Transmission OK')
            ackFlag = 1

        elif ack.decode() in 'NG':
            print('Transmission error')
            ackFlag = 2

    ackFlag = 0

senderDevice.close()