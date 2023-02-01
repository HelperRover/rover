# Import all of the necessary libraries. 
from flask import Flask, request, render_template
import time
from gpiozero import CamJamKitRobot, DistanceSensor

# Set pins 17 and 18 to trigger and echo.
pinTrigger = 17
pinEcho = 18

# Initialize the flask app, robot, and sensor.
app = Flask(__name__)
robot = CamJamKitRobot()
sensor = DistanceSensor(echo = pinEcho, trigger = pinTrigger)

# Set some basic constants for use in automatic control.
howNear = 65.0
reverseTime = 0.5

# This is the home, or index, route. It displays the index.html page.
@app.route('/')
def index():
    return render_template('index.html')

# This is the forward route. It is typically called via AJAX. 
@app.route('/forward')
def forward():

    # This sets the left motor to 70% power and the 
    # right motor to 90% power. We do this for calibration 
    # purposes.
    robot.value = (0.7, 0.9)

    # Sleep for 1 second.
    time.sleep(1)

    # Stop the rover.
    robot.stop()

    # Return a descriptive string.
    return 'Forward'

# This is the backward route. It is typically called via AJAX.
@app.route('/backward')
def backward():

    # This sets the left motor to 70% reverse power and the 
    # right motor to 90% reverse power. We do this for calibration 
    # purposes.
    robot.value = (-0.7, -0.9)

    # Sleep for 1 second.
    time.sleep(1)

    # Stop the rover.
    robot.stop()

    # Return a descriptive string.
    return 'Backward'

# This is the right route. It is typically called via AJAX.
@app.route('/right')
def right():

    # Turn the robot right (left wheel turns forwards,
    # right wheel turns backwards).
    robot.right()

    # Sleep for 0.35 seconds (different from the left
    # turn due to calibration).
    time.sleep(0.35)

    # Stop the rover.
    robot.stop()

    # Return a descriptive string.
    return 'Right'

# This is the left route. It is typically called via AJAX.
@app.route('/left')
def left():

    # Turn the robot left (right wheel turns forwards,
    # left wheel turns backwards).
    robot.left()

    # Sleep for 0.31 seconds (different from the right
    # turn due to calibration).
    time.sleep(0.31)

    # Stop the rover.
    robot.stop()

    # Return a descriptive string.
    return 'Left'

# This is the automatic route. It is typically called via AJAX.
@app.route('/automatic')
def automatic():

    # Automatic control should last 10 seconds so
    # we set a variable to the time that is 10
    # seconds from now.
    timeEnd = time.time() + 10

    # Continue automatic control for 10 seconds.
    while time.time() < timeEnd:

        # As above, this causes the rover to move
        # forwards.
        robot.value = (0.7, 0.9)

        # Sleep for 0.1 seconds before checking for
        # obstacles.
        time.sleep(0.1)

        # Check for obstacle by calling the 
        # isNearObstacle() function with the 
        # paramter from above.
        if isNearObstacle(howNear):

                # If near an object, stop
                # the rover.
                robot.stop()

                # Call the avoidObstacle()
                # function.
                avoidObstacle()

    # After 10 seconds stop the rover.
    robot.stop()

    # Return a descriptive string.
    return "Automatic"

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

    # As above, the rover is placed in reverse. The
    # different values are for calibration.
    robot.value = (-0.7, -0.9)

    # Sleep for the reverseTime.
    time.sleep(reverseTime)

    # Stop the rover.
    robot.stop()

    # Turn the rover to the right.
    robot.right()

    # Sleep for 0.35 seconds.
    time.sleep(0.35)

    # Stop the rover.
    robot.stop()

    return

# This route is for debugging purposes.
@app.route('/stop')
def stop():
    robot.stop()
    return 'Stop'