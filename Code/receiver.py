#written by Don McLaine, xBee
#edited by Eric Yee April 2015, decode and Database


import serial
import binascii
import sys
import time
import datetime
import mysql.connector
import math
from get_node_data import *


temp = 980
#DB

config = { 'host': 'localhost', 'port': 3306,
	'database': 'dronedb', 'user': 'root', 'password':
	'password', 'charset': 'utf8', 'use_unicode': True,
	'get_warnings': True, }

##
##config = { 'host': 'us-cdbr-azure-west-b.cleardb.com', 'port': 3306,
##	'database': 'dronedb', 'user': 'b58dc498c64f9a', 'password':
##	'7ab3b404', 'charset': 'utf8', 'use_unicode': True,
##	'get_warnings': True, }
##




cnx = mysql.connector.connect(**config)
cur = cnx.cursor()
last = 0

# for native windows something like: 'COM9'; for cygwin: '/dev/com9'
#port_id = '/dev/com9'
port_id = 'com18'

if sys.version_info[0] < 3:
    print("Use version 3!")
    sys.exit()

serial_port = serial.Serial(port_id, 9600, timeout=10)
print(serial_port.name)

file_handles = {}
initialized = False
while True:
	try:
		data = get_node_data(serial_port)
	except Exception as e:
		print(e)
	else:
		#print("Packet         :", binascii.hexlify(data['whole_packet']))
		#print("length         :", data['length'])
		#print("Frame type     :", hex(data['frame_type']))
		#if data['frame_type'] == 0x90:
		#	print("address_64     :", hex(data['address_64']))
		#elif data['frame_type'] == 0x81:
		#	print("RSSI       :", hex(data['RSSI']))
		#print("address_16     :", hex(data['address_16']))
		#print("receive_options:", binascii.hexlify(data['receive_options']))
		#print("the_rest       :", data['the_rest'])
		#print("contents       :", data['contents'])
		#print("checksum       :", binascii.hexlify(data['checksum']))
		
                
		if data['contents'][0] < 9:
                    #if data['contents'][0] not in file_handles:
                      #      file_handles[data['contents'][0]] = open( "Drone/" + hex(data['contents'][0]), "a+b")

                    #file_handles[data['contents'][0]].write(data['contents'])

                    #file_handles[data['contents'][0]].flush()

    #DB
                    

                    
                    sens = data['contents'][0]
                    if not initialized and sens is 1:
                        initialized = True
                        
                    if initialized:
                        last = last + 1
                        if sens is not last:
                            cur.close()
                            cnx.close()
                            initialized = False
                            print("recieved non consecutive sensor")
                            last = 0
                            cnx = mysql.connector.connect(**config)
                            cur = cnx.cursor()
                        else:
                            sendata = data['contents'][1:21]
                            now = datetime.datetime.now()
                            timep = str(now)
                            #time_id = int((time.mktime(datetime.datetime.timetuple(now))*1000 + (now.microsecond/1000))*10 + sens
                            #use db Bear Drone
                            cur.execute("insert into sen_time (SEN_ID, SEN_TIME) values (" + str(sens) + ", \"" + timep + "\")")
                            time_id = cur.lastrowid
                            print(str(sens) + "Sensor number")
                            for i in range(0, 10):
                                    val = sendata[2*i] + (sendata[2*i+ 1] << 8)
                                    bits = 16;
                                    
                                    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
                                            val = val - (1 << bits)        # compute negative value
                                    #print(val)
                                    if sens is 1:
                                        temp = 970
                                    if sens is 2:
                                            calc_data2 = 2570 - ((val/4095)*3.3)*1000
                                            if calc_data2 < 0:
                                                calc_data2 = 0
                                            airden = (1000*(101.325/(287.058*(273+(temp/4095)*100))))
                                            print(math.sqrt(2*(calc_data2)/airden) + "meters per second")
                                            print(math.sqrt(2*(calc_data2)/airden)*2.23694 + "miles per hour")
                                    cur.execute("insert into sen_data values (" + str(time_id) + ", " + str(i) + ", " + str(val) + ")")



                            #print(timep)
                            if last is 8:
                                cnx.commit()
                                last = 0
                                
                      

