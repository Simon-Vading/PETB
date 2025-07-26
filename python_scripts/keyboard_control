import RPi.GPIO as GPIO
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

def drive_forward(left_in1,left_in2,right_in1,right_in2):
	GPIO.output(left_in1,GPIO.LOW)
	GPIO.output(left_in2,GPIO.HIGH)
	
	GPIO.output(right_in1,GPIO.HIGH)
	GPIO.output(right_in2,GPIO.LOW)
	
def drive_backward(left_in1,left_in2,right_in1,right_in2):
	GPIO.output(left_in1,GPIO.HIGH)
	GPIO.output(left_in2,GPIO.LOW)
	
	GPIO.output(right_in1,GPIO.LOW)
	GPIO.output(right_in2,GPIO.HIGH)
	
def turn_clockwise(left_in1,left_in2,right_in1,right_in2):
	GPIO.output(left_in1,GPIO.LOW)
	GPIO.output(left_in2,GPIO.HIGH)
	
	GPIO.output(right_in1,GPIO.LOW)
	GPIO.output(right_in2,GPIO.HIGH)
	
def turn_anti_clockwise(left_in1,left_in2,right_in1,right_in2):
	GPIO.output(left_in1,GPIO.HIGH)
	GPIO.output(left_in2,GPIO.LOW)
	
	GPIO.output(right_in1,GPIO.HIGH)
	GPIO.output(right_in2,GPIO.LOW)
def stop(left_in1,left_in2,right_in1,right_in2):
	GPIO.output(left_in1,GPIO.LOW)
	GPIO.output(left_in2,GPIO.LOW)
	
	GPIO.output(right_in1,GPIO.LOW)
	GPIO.output(right_in2,GPIO.LOW)
	
	
pygame.init()

left_in1 = 22
left_in2 = 27
left_ena = 17

right_in1 = 20
right_in2 = 21
right_ena = 16

GPIO.setmode(GPIO.BCM)

GPIO.setup(left_in1,GPIO.OUT)
GPIO.setup(left_in2,GPIO.OUT)
GPIO.setup(left_ena,GPIO.OUT)
GPIO.setup(right_in1,GPIO.OUT)
GPIO.setup(right_in2,GPIO.OUT)
GPIO.setup(right_ena,GPIO.OUT)

power = 100

left_power = GPIO.PWM(left_ena,100)
left_power.start(power)

right_power = GPIO.PWM(right_ena,100)
right_power.start(power)

screen = pygame.display.set_mode( (640,480) )
pygame.display.set_caption('Python numbers')
screen.fill((159, 182, 205))

while(True):
	pygame.event.pump()
	user_input = pygame.key.get_pressed()
	if user_input[pygame.K_s]:
		drive_backward(left_in1,left_in2,right_in1,right_in2)
	elif user_input[pygame.K_w]:
		drive_forward(left_in1,left_in2,right_in1,right_in2)
	elif user_input[pygame.K_d]:
		turn_clockwise(left_in1,left_in2,right_in1,right_in2)
	elif user_input[pygame.K_a]:
		turn_anti_clockwise(left_in1,left_in2,right_in1,right_in2)
	elif user_input[pygame.K_e]:
		left_power = None
		right_power = None
		GPIO.cleanup()
		pygame.quit()
		break
	else:
		stop(left_in1,left_in2,right_in1,right_in2)

