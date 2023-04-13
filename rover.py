#!/usr/bin/env python3

# Import all of the necessary libraries. 
from bottle import route, run, static_file
from gpiozero import CamJamKitRobot, DistanceSensor
import time
import threading

automatic_mode = False
turn_speed = 0.4
left_speed = 0.4
right_speed = 0.45

# Change these for your setup.
IP_ADDRESS = '192.168.0.93' # of your Pi

# Set pins 17 and 18 to trigger and echo.
pinTrigger = 17
pinEcho = 18

# Set some basic constants for use in automatic control.
howNear = 65.0
reverseTime = 1

# Initialize the rover and sensor.
rover = CamJamKitRobot()
sensor = DistanceSensor(echo = pinEcho, trigger = pinTrigger)

# This is the home, or index, route. It displays the index.html page.
@route('/')
def index():
    return static_file('index.html', root='./')

# This route is for the style sheet.
@route('/style.css')
def index():
	return static_file('style.css', root='./')

# This route is for the script.
@route('/script.js')
def index():
	return static_file('script.js', root='./')

# This is the forward route. It is typically called via AJAX. 
@route('/forward')
@route('/forwards')
def action_forward():
        
    # This sets the left motor to 70% power and the 
    # right motor to 90% power. We do this for calibration 
    # purposes.
	rover.value = (left_speed, right_speed)
	return "FORWARDS"

# This is the backward route. It is typically called via AJAX.
@route('/back')
@route('/backward')
def action_back():
        
    # This sets the left motor to 70% reverse power and the 
    # right motor to 90% reverse power. We do this for calibration 
    # purposes.
	rover.value = (-left_speed, -right_speed)
	return "BACKWARDS"

# This is the left route. It is typically called via AJAX.
@route('/left')
def action_left():
        
    # Turn the robot left by setting custom motor speeds.
    rover.value = (-turn_speed, turn_speed)

    return "LEFT TURN"

# This is the right route. It is typically called via AJAX.
@route('/right')
def action_right():
        
    # Turn the robot right by setting custom motor speeds.
    rover.value = (turn_speed, -turn_speed)

    return "RIGHT TURN"

# This is the stop route. It is typically called via AJAX.
@route('/stop')
def action_stop():
    global automatic_mode

    # Disable the automatic mode if it is active.
    if automatic_mode:
        automatic_mode = False

    # Stop the rover.
    rover.stop()
    return "STOP"

# This is the automatic route. It is typically called via AJAX.
@route('/automatic')
def action_automatic():
    global automatic_mode

    if not automatic_mode:
        automatic_mode = True
        automatic_thread = threading.Thread(target=automatic_control)
        automatic_thread.start()
        return "AUTOMATIC MODE STARTED"
    else:
        automatic_mode = False
        return "AUTOMATIC MODE STOPPED"

def automatic_control():
    global automatic_mode

    # Set the rover to go forwards.
    rover.value = (left_speed, right_speed)

    # Loop until the automatic mode is stopped.
    while automatic_mode:

        # Check if the rover is near an obstacle.
        if isNearObstacle(howNear):

            # Call the avoidObstacle() function.
            avoidObstacle()

        # Add a small sleep to reduce CPU usage.
        time.sleep(0.1)

    # Stop the rover when automatic mode is disabled.
    rover.stop()

# Returns True if rover is less than 
# localHowNear centimeters from object.
def isNearObstacle(localHowNear):

    # Convert sensor distance to centimeters.
    distance = sensor.distance * 100

    # Print the distance to the log for debugging
    # purposes.
    print("Distance: " + str(distance))

    # If the distance is less than the 
    # localHowNear distance, return True.
    if distance < localHowNear:
        return True

    # Return False otherwise.
    else:
        return False
    
# This function called to avoid obstacles. It
# moves backwards for the reverseTime and then 
# turns right for 0.35 seconds.
def avoidObstacle():

    # If near an object, stop the rover.
    rover.stop()

    # Sleep for 0.35 seconds.
    time.sleep(0.75)

    # As above, the rover is placed in reverse. The
    # different values are for calibration.
    rover.value = (-left_speed, -right_speed)

    # Sleep for the reverseTime.
    time.sleep(reverseTime)

    # Stop the rover.
    rover.stop()

    # Sleep for 0.35 seconds.
    time.sleep(0.35)

    # Turn the robot right by setting custom motor speeds.
    rover.value = (turn_speed, -turn_speed)

    # Sleep for 0.35 seconds.
    time.sleep(.35)

    rover.stop()

    time.sleep(0.75)

    # Start the rover in new direction.
    rover.value = (left_speed, right_speed)

    return
        
# Start the webserver running on port 8080
try: 
    run(host=IP_ADDRESS, port=8080)
finally:  
    rover.cleanup()
