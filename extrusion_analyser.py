import serial
import time

ARD_NORMAL_BYTE = 0x00
ARD_ERROR_BYTE = 0x40
ARD_SYNC_BYTE = 0x80
ARD_MESSAGE_BYTE = 0xC0
GOOD_ACK_BYTE = 0x00
BAD_ACK_BYTE = 0x40
BUFFER_EMPTY_BYTE = 0x80
DATA_BUFFER_SIZE = 16

ser = serial.Serial("COM4",250000, timeout=1)
print('waiting for arduino', end=". ")
while (ser.inWaiting() == 0):
    time.sleep(0.5)
    print("", end=". ")
print('')

while ser.inWaiting() > 0 :
    ser.read(size=1)

error_count=0

rxIndex= 0x00



# modes: 0-normal, 1-recovery/sync
mode = 0
while (True):
    try:
        if mode == 0 :
            if ser.inWaiting() == 0 :
                ser.write(BUFFER_EMPTY_BYTE | rxIndex)
            elif ser.inWaiting() >= 5 :
                msg = ser.read(size=5)
                print(msg)
                header = msg[0]
                encoderBytes = {msg[1], msg[2]}
                stepperBytes = {msg[3], msg[4]}
                if header == ARD_NORMAL_BYTE | (rxIndex + 1):
                    if rxIndex == (DATA_BUFFER_SIZE -1) :
                        rxIndex = 0
                    else: 
                        rxIndex += 1
                    ack = bytes({GOOD_ACK_BYTE | rxIndex})
                    ser.write(ack[0])
                    print(ack)
                    encoderPos = int.from_bytes(encoderBytes, byteorder='big', signed=True)
                    stepperPos = int.from_bytes(encoderBytes, byteorder='big', signed=True)
                    print(int(rxIndex), end=", ")
                    print(encoderPos, end=(", "))
                    print(stepperPos, end=(", "))
                    print(error_count)
                elif header & 0xc0 == ARD_MESSAGE_BYTE :
                    break
                elif header & 0xc0 == ARD_SYNC_BYTE :
                    rxIndex = header & 0x3f
                else :
                    ser.write(BAD_ACK_BYTE & rxIndex)
                    mode = 1
            else: continue
        elif mode == 1 :
            print('beginning sync attempt')
            syncPadCount = 0
            syncAttempts = 0
            while (syncPadCount < 4 | syncAttempts <= 20):
                pad = ser.read(size=1)
                print('sync padding: ',end="")
                print(pad)
                if pad == 0xff :
                    syncPadCount =+ 1
                    syncAttempts =+ 1
                else:
                    syncPadCount = 0
                    syncAttempts =+ 1
            mode = 0
    except:
        ser.close()
        print("exit")
        break
