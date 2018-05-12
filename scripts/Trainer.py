# Author: SiliconSloth 18/1/2018

import bluetooth, threading, serial, time, os
import cv2
# PyCharm seems to need this to work properly.
# try:
#     from cv2 import cv2
# except:
#     pass


# This script is used to record training videos, by manually controlling the robot with the RobleyRemote app.
# Connect the laptop to the camera and Prizm controller via USB and pair the phone with the laptop via Bluetooth.
# Start this script on the laptop then start the phone app to begin recording video and control the robot.
# Press the stop button in the app to stop recording.
#
# Remember to set the correct path, port and index below before use.


# Power serialization format:
#
# Motor powers can range between -100 and 100.  If a special value of 101 is set for either motor the program running on the Prizm
# controller shuts down and restarts, which also causes the robot to stop moving.
#
# In order to send motor powers over Bluetooth and USB, they must first be converted into bytes.
# One byte can represent any integer between 0 and 255, so the first step of serialization is to round the powers to the nearest integer
# and add 100 so that they are in the range 0 to 201.
#
# In Python the function serial.to_bytes() is used to convert these powers to strings of bytes. These are just normal text strings, in which each
# byte corresponds to the ASCII code of a character in the string.  Since two powers are always sent at once (for the left and right motors)
# these strings are created using two bytes and contain two characters.  C++ and Java are able to send/receive bytes directly, without the use of strings.


videoPath = "/home/someone/RobleyVision/Recordings/" # Where video files will be saved to.
prizmPort = "/dev/ttyUSB0" # Check for the value of this in the Arduino IDE.
cameraIndex = 2 # 0,1,2 etc. determines which camera to use.


# Function to handle connecting to the remote control app via Bluetooth and relaying commands to the Prizm controller.
def serverLoop(serverSocket):
    global recording
    while True:
        # Wait for the app to request a connection.
        clientSocket = serverSocket.accept()[0]
        print("Remote Connected.")
        # Begin recording video when the remote app is first connected.
        recording = True
        while True:
            # Try to receive all the bytes in the queue and reject all but the two most recent bytes.
            # This means that if bytes are being sent faster than this script can process them the software will drop
            # some bytes rather than lagging behind, which should prevent the robot's steering from seeming slow to respond.
            message = clientSocket.recv(9999998)
            # An empty message is sent if the connection ends unexpectedly, in which case the robot should stop moving and
            # wait for the connection to be reestablished.
            if len(message) == 0:
                prizm.write("dd")   # "dd" is the string representation of powers 0,0
                print("Remote Disconnected.")
                break   # Go back to waiting for another connection.
            message = message[-2:]  # Take the last two bytes and discard the rest.

            # Deserialize the two motor powers from the two message bytes.  See above for details on the format.
            powerLeft, powerRight = ord(message[0])-100, ord(message[1])-100

            # Setting the motors to powers that are too low to actually move the robot can stall the motors and damage them.
            # If the user tries to use such low powers, set the powers to zero until they use a higher and safer power.
            if abs(powerLeft) < 20:
                powerLeft = 0
            if abs(powerRight) < 20:
                powerRight = 0
            # Serialize the powers in a byte string.  See above.
            message = serial.to_bytes([powerLeft+100, powerRight+100])

            # Send the serialized powers to the motor controller.
            prizm.write(message)
            # print(message)
            # Add a delay between sending messages so that consecutive byte pairs won't get mixed up in transmission.
            time.sleep(0.02)
            # When the user presses the stop button in the app, the app sends powers of 101 for both motors.
            # Detect when this happens and close the connection to the app.
            if powerLeft == 101 or powerRight == 101:
                clientSocket.close()
                serverSocket.close()
                print("Stopped.")
                # Exit the function, terminate the script and stop recording.
                return


# Uncomment this to print debug messages sent from the controller.
# def printLoop():
#     while True:
#         print(prizm.readline()[:-1])


# Function to record video from the camera to a file.
def videoLoop():
    global recording
    capture = cv2.VideoCapture(cameraIndex)
    print("Camera ready!")
    # Width and height.
    dimensions = (int(capture.get(3)), int(capture.get(4)))

    # The video files produced have numbered names, e.g. Training5.avi.
    # Find the highest number out of the existing files so we can make the number of
    # the new file one greater than that.
    lastNo = 0
    for file in os.listdir(videoPath):
        if file.startswith("Training") and file.endswith(".avi"):
            try:
                lastNo = max(lastNo, int(file.replace("Training", "").replace(".avi", "")))
            except:
                pass

    # Wait until the user starts the remote control app before recording starts (recording is set to True when a Bluetooth connection is opened).
    while not recording:
        time.sleep(0.01)
    writer = cv2.VideoWriter(videoPath+"/Training"+str(lastNo+1)+".avi",
                             cv2.VideoWriter_fourcc(*"MJPG"), 20, dimensions, True)
    print("Recording...")
    # When the remote control app is closed recording is set to False and recording stops.
    while recording:
        writer.write(capture.read()[1])
    print("Recording stopped.")
    writer.release()


# Connect to the Prizm controller.
prizm = serial.Serial(prizmPort)

# Start the thread that records video from the camera to a file.
recording = False
videoThread = threading.Thread(target=videoLoop)
videoThread.start()

# Uncomment this to print debug messages sent from the controller.
# printThread = threading.Thread(target=printLoop)
# printThread.daemon = True
# printThread.start()

# Set up a Bluetooth server and call serverLoop to start listening for client connections to it.
uuid = "dfdde353-e79e-49f0-bff0-43d12ca0fbec"   # Same as one in the app.
serverSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
serverSocket.bind(("", 0))
serverSocket.listen(1)
bluetooth.advertise_service(serverSocket, "RobleyRemote", uuid,
                            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE])
serverLoop(serverSocket)

# Tell the video thread to stop recording, and save the file before the script terminates.
recording = False
