#!/usr/bin/env python3

from bottle import route, run, static_file
from gpiozero import CamJamKitRobot
import time

# Change these for your setup.
IP_ADDRESS = '127.0.0.1' # of your Pi

rover = CamJamKitRobot()

@route('/left')
def action_left():
	rover.left()
	time.sleep(0.2)
	rover.stop()
	return "LEFT TURN"

@route('/right')
def action_right():
	rover.right()
	time.sleep(0.2)
	rover.stop()
	return "RIGHT TURN"

@route('/forward')
@route('/forwards')
def action_forward():
	rover.value(0.7, 0.9)
	return "FORWARDS"

@route('/back')
@route('/backward')
def action_back():
	rover.value = (-0.7, -0.9)
	return "BACKWARDS"

@route('/stop')
def action_back():
	rover.stop()
	return "STOP"

@route('/')
def index():
    return static_file('index.html', root='public')

@route('/style.css')
def index():
	return static_file('style.css', root='public')
        
# Start the webserver running on port 5000
try: 
    run(host=IP_ADDRESS, port=5000)
finally:  
    rover.cleanup()
