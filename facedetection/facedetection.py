import cv2

# Load the Haar Cascade Classifier for face detection
# String of local path to the file haarcascade_frontalface_default.xml
face_cascade = cv2.CascadeClassifier(
    "/Users/richelo/Desktop/facedetection/opencv/data/haarcascades/haarcascade_frontalface_default.xml"
)


# Start the video capture
video_capture = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)


while True:
    # Capture each frame of the video
    ret, frame = video_capture.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame using the Haar Cascade Classifier
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
    )

    # Draw a rectangle around each detected face
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the resulting frame with the detected faces
    cv2.imshow("Video", frame)

    # Exit the program when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the video capture and close the window
video_capture.release()
cv2.destroyAllWindows()
