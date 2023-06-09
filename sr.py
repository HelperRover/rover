import sounddevice as sd
import speech_recognition as sr
from scipy.io import wavfile


def recognize_speech():
    # Set the audio input device to the default system microphone
    device = sd.default.device

    # Set the sample rate and number of channels
    sample_rate = 44100
    channels = 1

    while True:
        # Record audio from the microphone
        print("Listening... Press 'q' to quit.")
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
        else:
            print("No voice recognized.")

        # Check if the user wants to quit
        if input("Press 'q' to quit, or any other key to continue: ") == "q":
            break


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


# Call the function to start recognizing speech
recognize_speech()
