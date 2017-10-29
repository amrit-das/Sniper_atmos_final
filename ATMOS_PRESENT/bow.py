#!/usr/bin/env python

import pypot.dynamixel
import time
import itertools
import numpy as np
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from math import pi,atan,sin,cos,degrees
import rospy
#import rospy
#from std_msgs.msg import String
flag = 0
#ang=(91.38, 87.34, 6.81, -47.16, 79.87, -80.31, -94.9, 124.18, -0.31, -2.68, 11.47, -12.7, -15.78, 14.55, -8.48, 3.91, -0.13, -4.26, 46.99)
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}

path = "/home/amrit/catkin/src/walk/scripts/data.xml"
abc =True
y,u,v = 0,142,56

cap = cv2.VideoCapture(4)

def sniper():

	rec=True	
	while rec:
		rec,img = cap.read()
		img_yuv = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
	
		blur = cv2.GaussianBlur(img_yuv,(11,11),2)
		ball = cv2.inRange(blur, (np.array([0,u-30,v-30])), (np.array([255,u+30,v+30])))
		im_floodfill = ball.copy()
		h, w = ball.shape[:2]
		mask = np.zeros((h+2, w+2), np.uint8)
		cv2.floodFill(im_floodfill, mask, (0,0), 255)
		fill = cv2.bitwise_and(im_floodfill,im_floodfill,mask = ball)

		if cv2.waitKey(25)&0xff==27:
		    break

		cv2.rectangle(img, (310,230), (330,250), (255,255,255),2)
		crop_img = fill[230:250, 310:330]
	
		images,s_contour,hierarchy = cv2.findContours(crop_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		images,contour,hierarchy = cv2.findContours(fill,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		cv2.drawContours(img, contour, -1, (0,255,0), 2)

		cv2.imshow("",img)
		#cv2.imshow("mask",im_floodfill)
		

		if len(contour)>=1:
			cnt = contour[0]
			(x,y),radius = cv2.minEnclosingCircle(cnt)
			center = (int(x),int(y))
			#point = cv2.circle(img,center,radius,(0,0,255),2)
			'''if center[0]>320 and center[1]>240:
				print "1,1"
			if center[0]<320 and center[1]>240:
				print "0,1"
			if center[0]>320 and center[1]<240:
				print "1,0"
			if center[0]<320 and center[1]<240:
				print "0,0"'''
			return center
		
		else:
			return (-1,-1)
		

class Dynamixel(object) :
	def __init__(self) :
		ports = pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("port bhakchodi pel rahe hain")

		print "Is port se judna hai",ports[0]

		self.dxl = pypot.dynamixel.DxlIO(ports[0])
		self.ids = self.dxl.scan(range(25))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<20 :
			raise RuntimeError("kuch motor bhakchodi pel rahe hain")
		self.dxl.set_moving_speed(dict(zip(self.ids,itertools.repeat(100))))


	def setSpeed(self,speed,ids) :
		self.dxl.set_moving_speed(dict(zip(ids,itertools.repeat(speed))))

	def setPos(self,pose) :
		pos = {ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)
		print pos

	def listWrite(self,list) :
		pos = dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)

	def dictWrite(self,dicti) :
		
		self.dxl.set_goal_position(dicti)

	def angleWrite(self,ids,pose) :
		self.dxl.set_goal_position({ids:pose})
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	



class XML(object) :
	def __init__(self,file) :
		try :
			tree = ET.parse(file)
			self.root = tree.getroot()
		except :
			raise RuntimeError("File nahi mil rahi")

	def parse(self,motion) :
		find = "PageRoot/Page[@name='" +motion+ "']/steps/step"
		steps = [x for x in self.root.findall(find)]
		p_frame = str()
		p_pose = str()
		for step in steps :
			Walk(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose)
			p_frame = step.attrib['frame']
			p_pose = step.attrib['pose']
			
	
xml = XML(path)
x=Dynamixel()

class Walk(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame = int(frame)
		self.begin = {}
		self.end = {}
		if not(p_pose) :
			self.frame_diff = 1
			p_pose = pose
		else :
			self.frame_diff = self.frame-int(p_frame) 

		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1]=pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1]=pos
		
		self.set(offsets=[darwin,hand])

	def Offset(self,offset) :
		
		for key in offset.keys() :
			if offset[key] == 'i' :
				self.begin[key] = -self.begin[key]
				self.end[key] = -self.end[key]
			else :
				self.begin[key] += offset[key]
				self.end[key] += offset[key]
		
		

	def set(self,offsets=[]) :
		for offset in offsets :
			self.Offset(offset)
		self.motion() 

	def motion(self) :
		print self.begin
		print self.end
		write=[]
		ids=[]
		for key in self.end.keys() :
			linp=np.linspace(self.end[key],self.begin[key],self.frame_diff)
			write.append(linp)
			ids.append(key)	

		print "out"
		for pose in zip(*write) :
			print "in"
			x.setPos(dict(zip(ids,pose)))
			time.sleep(0.01)

class Motion(object) :
	def __init__(self) : 
		
		
		self.fire_left = [2,4,6]
		self.fire_right = [1,3,5]
		self.armangle = 90
		self.d = 0.6
		#self.shoot()

	def center(self,center):

		x1,y1 = sniper()
		print x1,y1
		print x1,y1
		if x1>320:
			u = -2
		else:
			u = +2
		if y1<240:
			v=+2
		else:
			v=-2
		while x1<315  or x1>325:
			pan = list(x.returnPos(19))[0]
			pan+=u
			x1 = sniper()[0]
			x.angleWrite(19,pan)
			time.sleep(0.01)
			print pan
		while y1<235 or y1>245:
			tilt = list(x.returnPos(20))[0]
			tilt+=v
			y1 = sniper()[1]
			x.angleWrite(20,tilt)
			time.sleep(0.01)
			print "tilt"

	def shoot(self) :
		print "Shooting"
		head = list(x.returnPos(19))[0]
		tilt = list(x.returnPos(20))[0]
		head_rad = head*pi/180
		if head > 0 :
			theta = atan((1*sin(head_rad)-0.1)/(1*cos(head_rad)))
			write=[-30-tilt,45-(degrees(theta)),-15]
			pos=dict(zip(self.fire_left,write))
			x.dictWrite(pos)
			time.sleep(0.1)
			x.angleWrite(6,-48)
			time.sleep(2)
			x.angleWrite(6,-15)
			time.sleep(1)
			x.angleWrite(2,0)
			time.sleep(0.1)
			x.angleWrite(4,0.92)
			time.sleep(0.01)
			x.angleWrite(6,-48)
			print pos

		elif head < 0 :
			theta = atan((1*sin(head_rad)-0.1)/(1*cos(head_rad)))
			write = [30+tilt,-45-(degrees(theta)),-15]
			pos = dict(zip(self.fire_right,write))
			x.dictWrite(pos)
			time.sleep(0.1)
			x.angleWrite(5,-48)
			time.sleep(2)
			x.angleWrite(5,-15)
			time.sleep(1)
			x.angleWrite(1,0)
			time.sleep(0.1)
			x.angleWrite(3,0.92)
			time.sleep(0.01)
			x.angleWrite(5,-48)
			print pos
						

def walk() :
	w1 = xml.parse("32 F_S_L")
	time.sleep(0.01)
	w2 = xml.parse("33 ")
	time.sleep(0.01)
	while True :
		w3 = xml.parse("38 F_M_R")
		time.sleep(0.01)	
		w4 = xml.parse("39 ")
		time.sleep(0.01)
		w5 = xml.parse("36 F_M_L")
		time.sleep(0.01)
		w6 = xml.parse("37 ")
		time.sleep(0.01)

if __name__=="__main__" :
	m_20 = 90
	x.angleWrite(19,0)
	x.angleWrite(20,m_20)
	balance = xml.parse("152 Balance")
	raw_input("Proceed?")
	

	
	#walk()
	#ghost_final = sniper()
	#detected = False

	'''while True :
		x.angleWrite(20,m_20)
		for pose in range(-90,90,1) :
			x.angleWrite(19,pose)
			
			time.sleep(0.01)
			sniper_val = sniper()
			if not sniper_val == (-1,-1):
				print "Detected"
				flag = 1
				break
			else:
				print "Not detected"
		if flag == 1:
			break
		
		for pose in range(90,-90,-1) :
			x.angleWrite(19,pose)
			time.sleep(0.01)
			sniper_val = sniper()
			if not sniper_val == (-1,-1):
				print "Detected"
				flag = 1
				print abc
				break
			else:
				print "Found Center"
			print "Not Detected"

		if flag == 1:
			break

		m_20 = m_20-10
		
	m = Motion()
	m.center(sniper_val)
	print "Now shoot"
	m.shoot()'''

	#balance = xml.parse("23 Defence")
	balance = xml.parse("1 Bow")
	#time.sleep(1)
	
	'''
	balance = xml.parse("8 Push up")
	time.sleep(.8)
	balance = xml.parse("9 ")
	time.sleep(.8)
	balance = xml.parse("10 ")'''



			
