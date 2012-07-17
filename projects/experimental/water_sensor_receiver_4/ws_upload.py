#!/usr/bin/env python

import httplib
import socket
import random
import sys
import time
import commands
import re
import os
import serial
import math


auth_token = ""

def google_auth():
	global auth_token

	loginstring = str("accountType=GOOGLE&Email=createlabtds@gmail.com"
	+ "&Passwd=conductivity&service=fusiontables"
	+ "&source=python-app")

	host = "www.google.com"
	conn = httplib.HTTPSConnection(host)
	conn.putrequest("POST","/accounts/ClientLogin")
	conn.putheader("Content-Type","application/x-www-form-urlencoded")
	conn.putheader("Content-Length",str(len(loginstring)))
	conn.endheaders()
	conn.send(loginstring)

	response = conn.getresponse()
	html_page = response.read()
	auth_token = html_page[html_page.find("Auth=") + 5: -1]	


def create_table(num):
	
	stmt = str("sql=CREATE TABLE sensor" + str(num) 
	+ " (num: NUMBER, time: DATETIME)")
	conn = httplib.HTTPConnection("www.google.com")
	conn.putrequest("POST","/fusiontables/api/query")
	conn.putheader("Content-Type","application/x-www-form-urlencoded")
	conn.putheader("Content-Length",str(len(stmt)))
	conn.putheader("Authorization","GoogleLogin auth=" + auth_token)
	conn.endheaders()
	conn.send(stmt)

	response = conn.getresponse()
	html_page = response.read()
	print stmt
	print html_page

	return html_page


def sql_write(stmt):
	stmt = "sql=" + stmt
	conn = httplib.HTTPConnection("www.google.com")
	conn.putrequest("POST","/fusiontables/api/query")
	conn.putheader("Content-Type","application/x-www-form-urlencoded")
	conn.putheader("Content-Length",str(len(stmt)))
	conn.putheader("Authorization","GoogleLogin auth=" + auth_token)
	conn.endheaders()
	conn.send(stmt)

	response = conn.getresponse()
	html_page = response.read()
	if(html_page and (html_page[0:5] != "rowid")):
		print "---UPLOAD ERROR: FTABLES MISCONFIGURED---"

	return html_page


def sql_read(stmt):
	host = "www.google.com"
	conn = httplib.HTTPConnection(host)
	conn.putrequest("GET","/fusiontables/api/query?sql="+stmt)
	conn.putheader("Authorization","GoogleLogin auth=" + auth_token)
	conn.endheaders()
	response = conn.getresponse()
	values = response.read()

	return values


def get_sql_var(stmt):
	result = sql_read(stmt)
	lines = result.split('\n')
	if(len(lines) == 3):
		return lines[1]
	else:
		return ""


def update_ip():
	ipaddr = commands.getoutput("hostname -i").split(" ")[2]
	ipinfo = sql_read("SELECT+ROWID,+ip+FROM+592409+WHERE+name='g0'").split("\n")[1].split(",")
	print "IP UPDATED: " + ipaddr
	if(ipaddr != ipinfo[1]):
		sql_write("UPDATE 592409 SET ip='"+ipaddr+"' WHERE ROWID='"+ipinfo[0]+"'")


def storepart(pagelist, line, part):	
	if(line[0] == chr(0x12)):
		#print "WARNING: 0x12 VALUE. SAFE TO IGNORE."
		return 0
	
	arr = line[0:-4].split(",")
	#print line
	for i in range(8):
		try:
			pagelist.append((int(arr[2*i+4]),int(arr[(2*i)+5])))
		except:
			#print "WARNING: APPEND. SAFE TO IGNORE."
			return 0
	return 1


def getpage(ser, page):
	pagelist = []
	for i in range(8):
		line = ser.readline()
		if(line and (line.find("\n") == -1)):
                        time.sleep(0.01)
                        line += ser.readline()
		if(line):
			r = re.findall("pg[0-9]+", line)
			if(r and (int(r[0][2:]) == page)):
				r = re.findall("pt[0-9]+", line)
				if(r and (int(r[0][2:]) == i)):
					if(not storepart(pagelist, line, i)):
						return 0
				else:
					return 0
			else:
				return 0
		else:
			return 0
	return pagelist


def getallpages(ser, iden, points_promised):
	points = []
	for i in range(2048):
		points.append(0)

	ser.timeout = 0.01
	starttime = int(time.time()) 
	i = 0
	pages_promised = (((points_promised-1)/64)+1)
	while(i < pages_promised):
		
		time.sleep(0.03)
		ser.flushInput()
		ser.write("md,id" + iden + ",pg" + str(i) + ";\n")
		time.sleep(0.03)
		
		#print "calling getpage()"
		pagelist = getpage(ser, i)
		if(pagelist):
			print ("downloaded page [" + str(i+1) + "/"
					+ str(pages_promised) + "]")
			j=0
			for point in pagelist:
				points[(64*i)+j] = point
				j+=1
			i += 1
		if(int(time.time()) > starttime+30):
			print "SERIAL TIMEOUT FAILURE"
			ser.timeout = 1
			return 0
	
	ser.timeout = 1
	return points


def eraseall(ser, iden):
	ser.timeout = 0.01
	starttime = int(time.time())
	while(1):
		time.sleep(0.03)
		ser.flushInput()
		ser.write("me,id" + iden + ";\n")
		time.sleep(0.03)
		line = ser.readline()
		if(line[0:9] == ("se,id" + iden)):
			ser.timeout = 1
			return 1
		if(int(time.time()) > starttime+5):
			ser.timeout = 1
			return 0


def retrieve_data(ser):
	conn = 0
	points_promised = 0
	startpos = 0
	iden = ""

	while(1):
		line = ser.readline()

		if(not line):
			if(conn == 1):
				print "NODE LOST"
				conn = 0
				points_promised = 0
				iden = ""
				startpos = 0
			continue

		line = line[0:-3]
		#print '['+line+']'
		
		# handle ping
		if(line[0:5] == "sp,id"):
			r = re.findall("dc[0-9]+", line)
			if (r and (int(r[0][2:]) > 0)):
				ser.write("mi,id" + line[5:9] + ";\n")
			if(r and conn == 0):
				conn = 1
				print ("NODE DETECTED: [ID:" + line[5:9]
						+ ", POINTS:" + r[0][2:] + "]")
		
		#handle node info
		if(line[0:5] == "si,id"):
			iden = line[5:9]
			r = re.findall("dc[0-9]+", line)
			if(r):
				points_promised = int(r[0][2:])
			r = re.findall("st[0-9]+", line)
			if(r):
				startpos = int(r[0][2:])
			if(points_promised > 0):
				points = getallpages(ser, iden, points_promised)
				if(points):
					for i in range(2048-points_promised):
						points.pop()
					if((points_promised == 2048) and (startpos > 0)):
						for i in range(startpos-1):
							tempoint = points[0]
							points.pop(0)
							points.append(tempoint)
					
					print "DOWNLOAD COMPLETE"
					print "SENDING ERASE COMMAND"
					if(eraseall(ser, iden)):
						print "ERASE SUCCESS"
						return (iden, points)
					else:
						print "ERASE FAILURE, MUST RETRY"
		

def main():

	print "SCRIPT STARTED"

	while(1):
		try:
			google_auth()
			response = sql_read("SHOW+TABLES")
			break
		except IOError:
			print "No connection. Retry in 5s."
			time.sleep(5)

	try:
		###update_ip()
		pass
	except IOError:
		print "No connection. IP not updated."

	if(sys.platform == "linux2"):
		ser = serial.Serial("/dev/ttyUSB0",115200,timeout=1)
	elif(sys.platform == "win32"):
                try:
                        ser = serial.Serial("COM9",115200,timeout=1)
                except:
                        print "ERROR: WRONG SERIAL PORT"
                        print "RESTART LAPTOP PLEASE"
                        while(1):
                                continue
	else:
		print "ERROR: UNKNOWN PLATFORM"
		sys.exit(0)
	print "RECEIVING FROM SERIAL PORT: [" + ser.portstr + "]"

	while(1):
		print "LISTENING FOR NODES"
		(iden, points) = retrieve_data(ser)
		#commands.getoutput("mplayer "+os.getcwd()+"/collection_beep.wav")

		table = get_sql_var("SELECT+tableid+FROM+870286+WHERE+node='"+iden+"'")
		if(table == ""):
			table = '870464'

		currtime = int(time.time())
		points.reverse()

		print ""
		print "STARTING UPLOAD TO GOOGLE FUSION TABLES"
		print "UPLOADING: [ID:"+iden+", POINTS: " + str(len(points)) + "]"

		stmt = ""
		pages = (((len(points)-1)/64)+1)
		for i in range(len(points)):
			ppm = int( 11*math.exp(0.005 * int(points[i][0])) )
			temp = int( (0.973*int(points[i][1])) - 225.7)

			stmt += str("INSERT INTO " + table
			+ "(ppm, temp, time) VALUES (" + str(ppm) + ", " 
			+ str(temp) + ", " + str(currtime) + ");\n")

			if((((i+1) % 64) == 0) or (i == (len(points)-1))):
				while(1):
					try:
						#print stmt
						sql_write(stmt)
						print ("uploading page [" + str((i/64)+1) 
								+ "/" + str(pages) + "]")
						break
					except IOError:	
						print "No connection. Retry in 5s."
						time.sleep(5)
				stmt = ""
			currtime -= (10 * 60)

	
		print "UPLOAD TO GOOGLE FUSION TABLES COMPLETE"
		print "DATA HAS BEEN COLLECTED"
		print ""
		print "-------------------------------------------"
		#commands.getoutput("mplayer "+os.getcwd()+"/upload_beep.wav")

		try:
			#update_ip()
			pass
		except IOError:
			print "No connection. IP not updated."

		print ""
		points = []

if(__name__ == "__main__"):
	main()


