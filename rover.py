#!/usr/bin/env python3

import base64
import cv2
import json
import Adafruit_DHT
from bottle import Bottle, request, response, static_file, view
from geventwebsocket import WebSocketError
from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer

from gpiozero import CamJamKitRobot, DistanceSensor
import time
import threading

automatic_mode = False
turn_speed = 0.4
left_speed = 0.4
right_speed = 0.45

# Set pins 17 and 18 to trigger and echo.
pinTrigger = 17
pinEcho = 18

# DHT22 sensor connected to pin 25.
dht22_pin = 25

# Set some basic constants for use in automatic control.
howNear = 65.0
reverseTime = 1

# Initialize the rover and sensor.
rover = CamJamKitRobot()
sensor = DistanceSensor(echo = pinEcho, trigger = pinTrigger)

app = Bottle()

@app.route('/')
def index():
    return static_file('index.html', root='./')

# This route is for the style sheet.
@app.route('/style.css')
def index_style():
	return static_file('style.css', root='./')

# This route is for the script.
@app.route('/script.js')
def index_javascript():
	return static_file('script.js', root='./')

# This is the forward route. It is typically called via AJAX. 
@app.route('/forward')
@app.route('/forwards')
def action_forward():
        
    # This sets the left motor to 70% power and the 
    # right motor to 90% power. We do this for calibration 
    # purposes.
	rover.value = (left_speed, right_speed)
	return "FORWARDS"

# This is the backward route. It is typically called via AJAX.
@app.route('/back')
@app.route('/backward')
def action_back():
        
    # This sets the left motor to 70% reverse power and the 
    # right motor to 90% reverse power. We do this for calibration 
    # purposes.
	rover.value = (-left_speed, -right_speed)
	return "BACKWARDS"

# This is the left route. It is typically called via AJAX.
@app.route('/left')
def action_left():
        
    # Turn the robot left by setting custom motor speeds.
    rover.value = (-turn_speed, turn_speed)

    return "LEFT TURN"

# This is the right route. It is typically called via AJAX.
@app.route('/right')
def action_right():
        
    # Turn the robot right by setting custom motor speeds.
    rover.value = (turn_speed, -turn_speed)

    return "RIGHT TURN"

# This is the stop route. It is typically called via AJAX.
@app.route('/stop')
def action_stop():
    global automatic_mode

    # Disable the automatic mode if it is active.
    if automatic_mode:
        automatic_mode = False

    # Stop the rover.
    rover.stop()
    return "STOP"

# This is the automatic route. It is typically called via AJAX.
@app.route('/automatic')
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

@app.route('/video_feed_thread')
def video_feed():
    ws = request.environ.get('wsgi.websocket')
    if not ws:
        abort(400, 'Expected WebSocket request.')

    face_cascade = cv2.CascadeClassifier(
        "./haarcascade_frontalface_default.xml"
    )

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while True:
            ret, frame = camera.read() 
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )

            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            _, buffer = cv2.imencode('.jpg', frame)
            frame_base64 = base64.b64encode(buffer).decode('utf-8')

            ws.send(json.dumps({'image': frame_base64}))

    except WebSocketError:
        pass
    finally:
        camera.release()

@app.route('/temperature')
def read_temperature():
    humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, dht22_pin)
    if humidity is not None and temperature is not None:
        return "{0:.2f},{1:.2f}".format(temperature, humidity)
    else:
        return "Failed to get reading. Try again!"

@app.route('/temperature_feed_thread')
def temperature_feed():
    ws = request.environ.get('wsgi.websocket')
    if not ws:
        abort(400, 'Expected WebSocket request.')

    try:
        while True:
            temperature_data = read_temperature()
            if temperature_data:
                ws.send(json.dumps({'temperature_data': temperature_data}))
            time.sleep(1)

    except WebSocketError:
        pass

def start_video_feed_server():
    server = WSGIServer(('0.0.0.0', 8081), app, handler_class=WebSocketHandler)
    server.serve_forever()

def start_temperature_feed_server():
    server = WSGIServer(('0.0.0.0', 8082), app, handler_class=WebSocketHandler)
    server.serve_forever()


if __name__ == '__main__':
    video_thread = threading.Thread(target=start_video_feed_server)
    video_thread.start()

    temperature_thread = threading.Thread(target=start_temperature_feed_server)
    temperature_thread.start()

    app.run(host='0.0.0.0', port=8080)