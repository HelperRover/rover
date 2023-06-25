#!/usr/bin/env python3

import base64
import cv2
import json
import Adafruit_DHT
from bottle import Bottle, request, response, static_file, view
from geventwebsocket import WebSocketError
from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer
from gevent import sleep

import amg8833_i2c
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from scipy import interpolate
import scipy.ndimage as ndi

from gpiozero import CamJamKitRobot, DistanceSensor
import time
import threading

import sounddevice as sd
from scipy.io import wavfile
from queue import Queue
from io import BytesIO
from pydub import AudioSegment
from pydub.playback import play
import speech_recognition as sr

from pid import PID

automatic_mode = False
turn_speed = 0.4
left_speed = 0.4
right_speed = 0.5

# Set pins for trigger and echo.
pinTriggerForward = 4
pinEchoForward = 17

pinTriggerRight = 18
pinEchoRight = 27

pinTriggerLeft = 22
pinEchoLeft = 23

# DHT22 sensor connected to pin 25.
dht22_pin = 25

# Set some basic constants for use in automatic control.
howNear = 65.0
reverseTime = 1

# Initialize the rover and sensor.
rover = CamJamKitRobot()
sensorForward = DistanceSensor(echo = pinEchoForward, trigger = pinTriggerForward)
sensorRight = DistanceSensor(echo = pinEchoRight, trigger = pinTriggerRight)
sensorLeft = DistanceSensor(echo = pinEchoLeft, trigger = pinTriggerLeft)

# Global sensor values
max_temp = 0
num_voices = 0
num_faces = 0
num_thermals = 0

app = Bottle()

@app.hook('after_request')
def enable_cors():
    response.headers['Access-Control-Allow-Origin'] = '*'

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

@app.route('/recording.wav')
def index_style():
	return static_file('recording.wav', root='./')

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

@app.route('/data')
def action_analyze():
    # Return dictionary of sensor values
    global max_temp
    global num_voices
    global num_faces
    global num_thermals

    # Put values in dictionary
    values = {
        "max_temp": max_temp,
        "num_voices": num_voices,
        "num_faces": num_faces,
        "num_thermals": num_thermals
    }

    # Return the dictionary as a JSON object
    return json.dumps(values)

@app.route('/clear')
def action_clear():
    global max_temp
    global num_voices
    global num_faces
    global num_thermals

    max_temp = 0
    num_voices = 0
    num_faces = 0
    num_thermals = 0

    return "CLEARED"

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

    def audio_feed_thread():
        global num_voices
        while automatic_mode:
            if audio_feed():
                num_voices += 1

    def start_audio_feed_thread():
        audio_thread = threading.Thread(target=audio_feed_thread)
        audio_thread.start()

    start_audio_feed_thread()

    # Initialize PID Controller for both left and right sensors
    pidLeft = PID(P=0.006,I=0,D=0.004)  # adjust PID parameters as needed
    pidRight = PID(P=0.005,I=0,D=0.0025)  # adjust PID parameters as needed

    # Set the rover to go forwards.
    rover.value = (left_speed, right_speed)

    # Convert sensor distance to centimeters.
    distanceForward = sensorForward.distance * 100
    distanceRight = sensorRight.distance * 100
    distanceLeft = sensorLeft.distance * 100

    # Determine which wall to follow based on which sensor is closer at the start
    followRightWall = distanceRight < distanceLeft

    if followRightWall:
        desired_distance = distanceRight
    else:
        desired_distance = distanceLeft

    # Loop until the automatic mode is stopped.
    while automatic_mode:

        # Convert sensor distance to centimeters.
        distanceForward = sensorForward.distance * 100
        distanceRight = sensorRight.distance * 100
        distanceLeft = sensorLeft.distance * 100

        # # Call audio feed every 10 seconds
        # if int(time.time()) % 10 == 0:
        #     print("Audio feed called")
        #     if audio_feed():
        #         num_voices += 1

        # Check if there's a wall in front of the robot
        if distanceForward < desired_distance:
            if followRightWall: 
                # make a sharp left turn, you might need to adjust the speed values
                rover.value = (-turn_speed, turn_speed)
            else:  
                # make a sharp right turn, you might need to adjust the speed values
                rover.value = (turn_speed, -turn_speed)
                
            # pause for a bit, allowing the rover to make the turn
            time.sleep(0.3)
            rover.stop()
            time.sleep(0.5)
            rover.value = (left_speed, right_speed)
            continue

        if followRightWall:
            # calculate error using the right sensor
            error = desired_distance - distanceRight

            # update PID controller
            pidRight.update(error)

            # get the PID output
            pid_output = pidRight.output

            # calculate the new speed values
            left_speed_new = left_speed + pid_output
            right_speed_new = right_speed - pid_output

        else:  # follow left wall
            # calculate error using the left sensor
            error = desired_distance - distanceLeft

            # update PID controller
            pidLeft.update(error)

            # get the PID output
            pid_output = pidLeft.output

            # calculate the new speed values
            left_speed_new = left_speed - pid_output
            right_speed_new = right_speed + pid_output

        # ensure the speeds are within acceptable range
        left_speed_new = min(max(left_speed_new, -1), 1)
        right_speed_new = min(max(right_speed_new, -1), 1)

        # update the rover speeds
        rover.value = (left_speed_new, right_speed_new)

        # pause for a bit
        time.sleep(0.001)

    # Stop the rover when automatic mode is disabled.
    rover.stop()

@app.route('/video_feed_thread')
def video_feed():
    global num_faces
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

            num_faces = num_faces + len(faces)

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
    global max_temp
    ws = request.environ.get('wsgi.websocket')
    if not ws:
        abort(400, 'Expected WebSocket request.')

    try:
        while True:
            temperature_data = read_temperature()
            if temperature_data:
                ws.send(json.dumps({'temperature_data': temperature_data}))
                # Add temperature data to global max_temp if it is higher than the current max_temp
                if float(temperature_data.split(',')[0]) > max_temp:
                    max_temp = float(temperature_data.split(',')[0])
            time.sleep(1)

    except WebSocketError:
        pass

# Interpolation Properties
pix_res = (8,8) # pixel resolution
xx,yy = (np.linspace(0,pix_res[0],pix_res[0]),
                    np.linspace(0,pix_res[1],pix_res[1]))
zz = np.zeros(pix_res) # set array with zeros first
# new resolution
pix_mult = 6 # multiplier for interpolation 
interp_res = (int(pix_mult*pix_res[0]),int(pix_mult*pix_res[1]))
grid_x,grid_y = (np.linspace(0,pix_res[0],interp_res[0]),
                            np.linspace(0,pix_res[1],interp_res[1]))

# interp function
def interp(z_var):
    # cubic interpolation on the image
    # at a resolution of (pix_mult*8 x pix_mult*8)
    f = interpolate.interp2d(xx,yy,z_var,kind='cubic')
    return f(grid_x,grid_y)
grid_z = interp(zz) # interpolated image

@app.route('/thermal_feed_thread')
def thermal_feed():
    global num_thermals

    ws = request.environ.get('wsgi.websocket')
    if not ws:
        abort(400, 'Expected WebSocket request.')

    sensor = amg8833_i2c.AMG8833(addr=0x69)

    plt.rcParams.update({'font.size':16})
    fig_dims = (12,9) # figure size
    fig,ax = plt.subplots(figsize=fig_dims) # start figure
    pix_res = (8,8) # pixel resolution
    zz = np.zeros(pix_res) # set array with zeros first
    im1 = ax.imshow(zz,vmin=15,vmax=40) # plot image, with temperature bounds
    ax.axis('off') # this line removes the axes
    # cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
    # cbar.set_label('Temperature [C]',labelpad=10) # temp. label
    fig.canvas.draw() # draw figure

    ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs

    pix_to_read = 64
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0) # this line removes the white space
    try:
        while True:
            status, pixels = sensor.read_temp(pix_to_read)
            if status:
                continue
            new_z = interp(np.reshape(pixels, pix_res)) # interpolated image

            # Create a binary image where pixels above the threshold are marked as 'True'
            binary_img = np.reshape(pixels,pix_res) > thresh_temp

            # Label connected components in the binary image
            labeled_img, num_labels = ndi.label(binary_img)

            num_thermals = num_thermals + num_labels

            T_thermistor = sensor.read_thermistor()
            fig.canvas.restore_region(ax_bgnd)
            im1.set_data(new_z)
            ax.draw_artist(im1)
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()

            # Convert the matplotlib figure to a PIL Image
            pil_image = Image.frombytes('RGBA', fig.canvas.get_width_height(), 
                                        fig.canvas.tostring_argb())

            # Convert the PIL Image to a numpy array
            numpy_image = np.array(pil_image)

            # Now you can encode the numpy array
            _, buffer = cv2.imencode('.jpg', numpy_image)
            thermal_image_base64 = base64.b64encode(buffer).decode('utf-8')

            ws.send(json.dumps({'thermal_image': thermal_image_base64}))

    except WebSocketError:
        pass

def recognize_google(audio, sample_rate):
    # Create a recognizer object
    r = sr.Recognizer()

    # Convert the audio data to AudioData object
    audio_data = sr.AudioData(
        audio.tobytes(), sample_rate=sample_rate, sample_width=audio.dtype.itemsize
    )

    # Convert the audio data to text
    try:
        text = r.recognize_google(audio_data, show_all=False)
        return text
    except sr.UnknownValueError:
        return None

@app.route('/audio_feed_thread')
def audio_feed():
    device = sd.default.device

    # Set the sample rate and number of channels
    sample_rate = 44100
    channels = 1

    duration = 0.1 # adjust as needed

    # Record audio from the microphone
    print("Listening...")
    audio = sd.rec(
        int(sample_rate * 5),
        samplerate=sample_rate,
        channels=channels,
        device=device,
    )
    sd.wait()

    # Save the recorded audio to a WAV file
    wavfile.write("recording.wav", sample_rate, audio)

    # Convert the audio data to text
    text = recognize_google(audio, sample_rate)

    # Check if speech is recognized or not
    if text:
        print("Voice recognized:", text)
        return text
    else:
        print("No voice recognized.")
        return None

def start_video_feed_server():
    server = WSGIServer(('0.0.0.0', 8081), app, handler_class=WebSocketHandler)
    server.serve_forever()

def start_temperature_feed_server():
    server = WSGIServer(('0.0.0.0', 8082), app, handler_class=WebSocketHandler)
    server.serve_forever()

def start_thermal_feed_server():
    server = WSGIServer(('0.0.0.0', 8083), app, handler_class=WebSocketHandler)
    server.serve_forever()

def start_audio_feed_server():
    server = WSGIServer(('0.0.0.0', 8084), app, handler_class=WebSocketHandler)
    server.serve_forever()

if __name__ == '__main__':
    video_thread = threading.Thread(target=start_video_feed_server)
    video_thread.start()

    temperature_thread = threading.Thread(target=start_temperature_feed_server)
    temperature_thread.start()

    thermal_thread = threading.Thread(target=start_thermal_feed_server)
    thermal_thread.start()

    audio_thread = threading.Thread(target=start_audio_feed_server)
    audio_thread.start()

    app.run(host='0.0.0.0', port=8080)