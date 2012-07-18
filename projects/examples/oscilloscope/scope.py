#! /usr/bin/env python

# Python Oscilloscope
# by Maxim Buevich
#
# Requires PyGame Library


import sys
import pygame
pygame.init() 


serial = open("/dev/ttyUSB0","r")

#create the screen
window = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Mobile Oscilloscope")

black = (0,0,0)
white = (255,255,255)

signal = list()
draw = 1

while(1):
	
	while(1):
		try:
			val = int(serial.readline())
		except ValueError:
			continue
		break
	signal.append(val)	

	if(len(signal) > 110):
		for i in range(0,len(signal)-110):
			signal.pop(0)

	if(draw):
		print val
		window.fill(black)
		pygame.draw.aaline(window, white, (40,40), (40,440))
		pygame.draw.aaline(window, white, (40,440), (600,440))
		i = 0
		while(i < (len(signal)-1)):
			pygame.draw.aaline(window, white, ((5*i)+40,440-(2*signal[i])), 
			((5*(i+1))+40,440-(2*signal[i+1])))
			i = i + 1
		pygame.display.flip()

	for event in pygame.event.get(): 
		if event.type == pygame.QUIT: 
			sys.exit(0)
		elif event.type == pygame.MOUSEBUTTONDOWN:
			draw = not draw
				


