import serial
import time
import datetime
import csv
import math

# user input
COM_PORT             = 'COM10'
SAMPLE_INTERVAL      = 10         # desired sample interval in milliseconds. Valid values are 1-250.
AVG_SAMPLES          = 10         # number of data points for rolling averages
REVERSE_ENCODER_DIR  = False
REVERSE_STEPPER_DIR  = False
VERBOSE              = False
RESEND_ON_ERROR      = False      # setting to zero may enable faster sampling, but crc and indexing errors will not be corrected. They will still be noted in the csv.
FILAMENT_DIAMETER    = 1.75
STEPPER_SCALING      = 0.0023624  # mm of filament per microstep; equivalent to step distance
ENCODER_SCALING      = 0.00097656 # mm of filament per encoder tick; depends on sensor construction and encoder resolution
# end of user input

GOOD_ACK_BYTE = b'\xaa'
BAD_ACK_BYTE = b'\x55'
BREAK_MSG = b'\xaa\x7f\xff\x7f\xff\xaa'
ARD_BUFFER_SIZE = 128
CRC_POLYNOMIAL = 0x83

#build crc lookup table
crc_lookup = []
for i in range(0, 256):
    current_byte = i
    for bit in range (0, 8) :
        if current_byte & 0x80 != 0:
            current_byte = current_byte << 1
            current_byte = current_byte ^ CRC_POLYNOMIAL
        else:
            current_byte = current_byte << 1
    crc_lookup.append(current_byte & 0xff)
    
# set and get parameters for MCU
ser = serial.Serial(COM_PORT,250000, timeout=1)
print('waiting for arduino', end=". ")
ser.read(ser.inWaiting())
while (ser.inWaiting() == 0):
    time.sleep(0.5)
    print("", end=". ")
print("arduino connected")
while (ser.inWaiting() > 0) :
    ser.read()
ser.write(int.to_bytes(SAMPLE_INTERVAL & 0xff, length=1, byteorder='big', signed=False))
ser.write(int.to_bytes(int(RESEND_ON_ERROR), length=1, byteorder='big', signed=False))
while (ser.inWaiting() < 1) :
    pass
sample_interval_get = ser.read(size=1)
sample_interval_get = sample_interval_get[0]
while (ser.inWaiting() > 0) :
    print(ser.read())


filename = 'extrusion_analyzer-' + datetime.datetime.now().strftime('%y%m%d%H%M%S' +'.csv')
with open(filename, 'a', newline='') as f:
    writer = csv.writer(f,delimiter=',')
    writer.writerow(['sample interval (milliseconds): ', str(sample_interval_get)])
    writer.writerow(['points included in rolling average: ', str(AVG_SAMPLES)])
    writer.writerow(['filament diameter (mm): ', str(FILAMENT_DIAMETER)])
    writer.writerow(['stepper scaling (mm/microstep): ', str(STEPPER_SCALING)])
    writer.writerow(['encoder scaling (mm/count): ', str(ENCODER_SCALING)])
    writer.writerow('')    
    writer.writerow(['sample#','time','encoder','stepper','encoder_avg','stepper_avg','velocity_cmd','velocity_act','flowrate_cmd','flowrate_act','pcnt_diff','error_on_sample','msg_hex','exp_crc'])
    
    rx_index = 0
    sample_num = 0
    bad_msg_count = 0
    encoder_avg_list = []
    stepper_avg_list = []
    
    filament_area = (math.pi * pow(FILAMENT_DIAMETER / 2, 2))
    
    while True:
        try:
            if ser.inWaiting() >= 6 :
                msg = ser.read(size=6)
                if msg == BREAK_MSG : break
            
                crc=0
                for x in range(0, 5): #calculate crc with lookup table
                    crc = crc_lookup[msg[x] ^ crc]
                    
                good_crc = crc == msg[5]
                good_index = msg[0] == rx_index
                good_msg = good_crc and good_index
                
                if good_msg and RESEND_ON_ERROR:
                    ser.write(GOOD_ACK_BYTE)
                elif not good_msg : 
                    bad_msg_count += 1
                
                if good_msg or not RESEND_ON_ERROR:
                    
                    encoder_bytes = bytes([msg[1], msg[2]])
                    stepper_bytes = bytes([msg[3], msg[4]])
                    encoder_pos = int.from_bytes(encoder_bytes, byteorder='big', signed=True)
                    stepper_pos = int.from_bytes(stepper_bytes, byteorder='big', signed=True)
                    
                    if REVERSE_ENCODER_DIR :
                        encoder_pos *= -1
                    if REVERSE_STEPPER_DIR :
                        stepper_pos *= -1
                    
                    encoder_avg_list.append(encoder_pos)
                    stepper_avg_list.append(stepper_pos)
                    if len(encoder_avg_list) > AVG_SAMPLES :
                        encoder_avg_list.pop(0)
                        stepper_avg_list.pop(0)
                    
                    encoder_avg_pos = sum(encoder_avg_list) / len(encoder_avg_list)
                    stepper_avg_pos = sum(stepper_avg_list) / len(stepper_avg_list)
                    
                    velocity_cmd = stepper_avg_pos / (sample_interval_get / 1000)
                    velocity_act = encoder_avg_pos / (sample_interval_get / 1000)
                    
                    flowrate_cmd = filament_area * velocity_cmd
                    flowrate_act = filament_area * velocity_act
                    
                    time_at_sample = sample_num * sample_interval_get / 1000
                    
                    pcnt_diff = 0
                    if flowrate_cmd != 0:
                        pcnt_diff = (flowrate_act - flowrate_cmd) / flowrate_cmd * 100
                    
                    row_data = [str(sample_num), str(time_at_sample), str(encoder_pos),str(stepper_pos), str(encoder_avg_pos), str(stepper_avg_pos), str(velocity_cmd), str(velocity_act), str(flowrate_cmd), str(flowrate_act), str(pcnt_diff)]
                    if not good_msg :                
                        row_error_data =  [str(not good_msg), ':'.join((hex(x)) for x in msg), hex(crc)]
                        for i in row_error_data:
                            row_data.append(i)
                    
                    writer.writerow(row_data)
                    
                    if VERBOSE :
                        print(encoder_pos, end=(", "))
                        print(stepper_pos, end=(", "))
                        # print(':'.join((hex(x)) for x in msg), end=', ')
                        print(bad_msg_count, end=', ')
                        # print(hex(crc), end='')
                        print('')
                    
                    rx_index = msg[0]
                    if rx_index == ARD_BUFFER_SIZE - 1:
                        rx_index = 0
                    else:
                        rx_index += 1
                    
                    sample_num += 1
                    
                else:
                    ser.write(BAD_ACK_BYTE)
                    print('sent bad ack on msg: ', end='')
                    print(':'.join((hex(x)) for x in msg), end='\t')
                    crc = 0
                    print(hex(crc))
        except:
            ser.close()
            print("exit")
            break
if ser.isOpen() :
    while ser.inWaiting() != 0 :
        print(ser.read_until())
        time.sleep(.1)
    ser.close()
