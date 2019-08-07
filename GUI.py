#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import pygame
import sys
import time
import numpy as np

pygame.init()
controlWindowWidth  = 750
controlWindowHeight = 750
controlWindow = pygame.display.set_mode((controlWindowWidth,controlWindowHeight))
pygame.display.set_caption("Controller")

joystickWindowWidth = 500
joystickWindowHeight= 500

#color
RED 	= (250,  0,  0)
GREEN	= (  0,250,  0)
BLUE	= (  0,  0,255)
BLACK	= (  0,  0,  0)
WHITE	= (250,250,250)

#circle params
center=[controlWindowWidth/2,controlWindowHeight/2]
radius = 10

#tracking circle params
centerTrack=[controlWindowWidth/2,controlWindowHeight/2]
radiusTrack = 8

#line params
lineWidth = 3

run = True

vel_msg = Twist()
arm	= False

class joystickBackground:
	def __init__(self, width, height):
		self.size   = [width, height]
		self.center = [controlWindowWidth/2, controlWindowHeight/2]
		self.datum  = [(controlWindowWidth-width)/2, (controlWindowHeight-height)/2]

	def drawBorder(self):
		pygame.draw.rect(controlWindow, RED, (self.datum[0], self.datum[1], self.size[0], self.size[1]), lineWidth)

	def drawMidLine(self):
		#vertical line
		pygame.draw.line(controlWindow, RED, (self.center[0], self.datum[1]), (self.center[0], self.datum[1]+50), lineWidth)
		pygame.draw.line(controlWindow, RED, (self.center[0], self.size[1]+50), (self.center[0], self.size[1]+self.datum[1]), lineWidth)
		pygame.draw.line(controlWindow, RED, (self.center[0], self.center[1]-25), (self.center[0], self.center[1]+25), lineWidth)

		pygame.draw.line(controlWindow, RED, (self.datum[0], self.center[1]), (self.datum[1]+50, self.center[1]), lineWidth)
		pygame.draw.line(controlWindow, RED, (self.size[0]+50, self.center[1]), (self.size[0]+self.datum[0], self.center[1]), lineWidth)
		pygame.draw.line(controlWindow, RED, (self.center[0]-25, self.center[1]), (self.center[0]+25, self.center[1]), lineWidth)

def drawBackground():
	controlWindow.fill(WHITE)
	joystick = joystickBackground(joystickWindowWidth, joystickWindowHeight)
	joystick.drawBorder()
	joystick.drawMidLine()
	#pygame.draw.circle(controlWindow, RED, (controlWindowWidth/2,controlWindowHeight/2), controlWindowWidth/2, lineWidth)

def drawOutputCircle():
	pygame.draw.circle(controlWindow, RED  , (center[0],center[1]), radius)

def drawTrackingCircle():
	pygame.draw.circle(controlWindow, BLACK, (centerTrack[0],centerTrack[1]), radiusTrack)
	pygame.draw.line  (controlWindow, BLACK, (controlWindowWidth/2, controlWindowHeight/2), (centerTrack[0], centerTrack[1]), 4)	

def drawToScreen():
	drawBackground()
	drawOutputCircle()
	drawTrackingCircle()

def globalToLocal(posX, posY):
	return posX-(controlWindowWidth-joystickWindowWidth)/2,  posY-(controlWindowHeight-joystickWindowHeight)/2

def transform(posX, posY):
	#transform from frame coordinate to "remote control" coordinate
	return posX-250, -(posY-250)

def constraint(pos, min, max):
	posCalc = pos
	if pos > max: posCalc = max
	elif pos < min: posCalc = min

	return posCalc

def display():
	pygame.time.delay(100)
	for e in pygame.event.get():
		if e.type == pygame.QUIT:
			run = False
			pygame.quit()
			sys.exit()
		if e.type == pygame.MOUSEBUTTONDOWN:
			center[0], center[1] = pygame.mouse.get_pos()
			center[0] = constraint(centerTrack[0], 125, 625)
			center[1] = constraint(centerTrack[1], 125, 625)

			myPosX, myPosY = globalToLocal(center[0], center[1])
			myPosX, myPosY = transform(myPosX, myPosY)

			vel_msg.angular.z = myPosX/10
			vel_msg.linear.x = myPosY/10

	centerTrack[0], centerTrack[1] = pygame.mouse.get_pos()
	centerTrack[0] = constraint(centerTrack[0], 125, 625)
	centerTrack[1] = constraint(centerTrack[1], 125, 625)

	drawToScreen()
	pygame.display.update()

def publisher():
	rospy.init_node('controller', anonymous = True)
	pub 	= rospy.Publisher('/positionControl', Twist, queue_size=1)
	pubArm 	= rospy.Publisher('/armCommand'	, Bool , queue_size=1)

	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

	while not rospy.is_shutdown():
		display()
		pub.publish(vel_msg)
		pubArm.publish(arm)

	rospy.spin()

if __name__ == '__main__':
	publisher()

###Below here are probably useful functions, "probably"

#def getDistance(posX, posY):
#	return np.sqrt(posX*posX + posY*posY)

#def maxPos(posX, posY):
#	radiusMax = 250
#
#	posX = posX - 250
#	posY = -(posY - 250)
#	middleX = controlWindowWidth/2 - 250
#	middleY = controlWindowHeight/2 - 250
#	angle = np.arctan2(middleY - posY, middleX - posX)

	#print(np.rad2deg(angle))
#	radiusReal = getDistance(posX,posY)
#
#	if (radiusReal > 250):
#		nowX = (-radiusMax * np.cos(angle)) +250
#		nowY = (-radiusMax * np.sin(angle)) - 250
#	else:
#		nowX = posX + 250
#		nowY = (posY -250)
#
#	return int(nowX), -int(nowY)
