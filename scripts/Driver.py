# Author: SiliconSloth 18/1/2018

from threading import Thread, Timer
import numpy as np
import cPickle, serial, os, time, subprocess
import cv2
# PyCharm seems to need this to work properly.
# try:
#     from cv2 import cv2
# except:
#     pass


# This script is responsible for actually driving the robot automatically.  It uses the reduced video and feature files
# produced by Reducer.py for guidance and outputs debug video and log files which can be viewed with Viewer.py.


videoPath = "/home/someone/RobleyVision/Recordings/"  # Where video files will be saved to.
# beepFile = "/home/someone/RobleyVision/Beep.wav"  # File to play when beeping on Linux. (optional)
prizmPort = "/dev/ttyUSB0"  # Check for the value of this in the Arduino IDE.
cameraIndex = 2  # 0,1,2 etc. determines which camera to use.

# Files to load training video and features from.
reducedVideoFile = "TrainingReduced.avi"
reducedFeatureFile = "ReducedFeatures.pkl"

# Determines how quickly the robot moves forwards/backwards and rotates.  Bigger is faster.
forwardCoeff, rotationCoeff = 0.5, 0.1

# This is the minimum motor power required to make the robot move at all, which is
# added onto the powers calculated while driving to overcome friction.
basePower = 10


# Function to set the left and right motor powers on the robot.
def setPowers(power1, power2):
    # print(power1, power2)
    # See Trainer.py for more details on the format used to send powers to the Prizm controller.
    prizm.write(serial.to_bytes([int(power1)+100, int(power2)+100]))


# Function to find matching key points between two images.
def match(keypoints1, descriptors1, keypoints2, descriptors2):
    try:
        matches = bf.match(descriptors1, descriptors2)
        goodMatches = []
        for i,m in enumerate(matches):
            # Get the coordinates of the two key points in this match.
            point1, point2 = keypoints1[m.queryIdx].pt, keypoints2[m.trainIdx].pt
            # Realistically we know that key points will not have moved very far across the image.
            # If a matching is found between two points that are very far away from each other it is probably an
            # incorrect matching, and should be excluded from the results.
            if abs(point1[1] - point2[1]) < 100 and abs(point1[0] - point2[0]) < 100:
                goodMatches.append(m)

        return goodMatches
    except:
        # It is possible for bf.match() to throw an exception if it failed to find a match due to an insufficient number of key points.
        # If this happens, return an empty list of matches.
        return []


# Function to calculate the median optical flow between two images.
def calculateFlow(keypoints1, descriptors1, keypoints2, descriptors2):
    # Find matches between the key points of the two images.
    matches = match(keypoints1, descriptors1, keypoints2, descriptors2)
    # If the matching failed and returned an empty list the optical flow cannot be found.
    # If the matching contains less than 20 matches it is too poor to be reliable so don't use it.
    if not matches or len(matches) < 20:
        return None

    # Create a 2D array in which each row contains the x and y components of one optical flow vector.
    # Do this by subtracting the coordinates of the first key point of each match from the second key point's coordinates.
    flowVectors = np.array([(keypoints2[m.trainIdx].pt[0] - keypoints1[m.queryIdx].pt[0],
                             keypoints2[m.trainIdx].pt[1] - keypoints1[m.queryIdx].pt[1]) for m in matches])
    # Find the medians of all the x and y components.
    return np.median(flowVectors, 0)


# Called if the camera takes too long to respond when the next frame is requested.
def cameraTimeoutFunc():
    global timedOut
    print("Timeout!")
    timedOut = True
    # Stop the robot until the camera responds, so that the robot doesn't drive off course without being able to see what it is doing.
    setPowers(0,0)


# This loop is constantly running in the background, ready to make a beeping sound if the robot is obstructed or drives off course.
# def beepLoop():
#     while True:
#         if fails >= 10: # If matching fails 10 times or more in a row the robot is probably obstructed or has driven wildly off course.
#             # On Linux, call an external program for beeping.
#             subprocess.call(["paplay", beepFile])
#             # On Windows, use winsound.
#             # winsound.Beep(500,600)
#             time.sleep(0.4) # Add a delay between beeps.
#         else:
#             time.sleep(0.01)    # Keep waiting until beeping is needed.


# Connect to the Prizm controller.
prizm = serial.Serial(prizmPort)

# Initialize the camera and load the target video.
capture = cv2.VideoCapture(cameraIndex)
print("Camera ready...")
width, height = int(capture.get(3)), int(capture.get(4))
targetCapture = cv2.VideoCapture(videoPath+reducedVideoFile)

# Load the target key points and descriptors from the feature file.
with open(videoPath+reducedFeatureFile, "rb") as file:
    targetFeatures = cPickle.load(file)
# Convert the key point tuples back to OpenCV KeyPoints.
for keypoints, descriptors in targetFeatures:
    for i in range(len(keypoints)):
        point = keypoints[i]
        keypoints[i] = cv2.KeyPoint(point[0][0], point[0][1], point[1], point[2], point[3], point[4], point[5])

# The video files produced have numbered names, e.g. Debug17.avi.
# Find the highest number out of the existing files so we can make the number of
# the new file one greater than that.
lastNo = 0
for file in os.listdir(videoPath):
    if file.startswith("Debug") and file.endswith(".avi"):
        try:
            lastNo = max(lastNo, int(file.replace("Debug", "").replace(".avi", "")))
        except:
            pass
# The debug video shows the current camera frame and target frame side by side, so requires double width.
writer = cv2.VideoWriter(videoPath+"Debug"+str(lastNo+1)+".avi", cv2.VideoWriter_fourcc(*"MJPG"), 20, (width*2, height), True)
# This list will contain tuples containing the values of various variables at each iteration of the main loop.
# You can change which variables are logged by modifying the code that adds values to the list.
debugLog = []

# Create the key point detector, descriptor extractor and matcher.
fast = cv2.FastFeatureDetector_create(threshold=16, nonmaxSuppression=True)
brief = cv2.xfeatures2d.BriefDescriptorExtractor_create(bytes=16, use_orientation=False)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# The index of the current target video frame, starts at 0.
featureInd = 0
# The frame that the robot is currently trying to match with, initialized as the first video frame/
targetFrame = targetCapture.read()[1]
# The number of successive times that the software has failed to find a matching between the current camera frame
# and the target frame.  If there are 10 successive failures that probably means that the robot has gone off course
# or had its view obstructed so should stop moving and start beeping.
fails = 0

# Wait for the user to press enter before the robot starts moving.
try:
    input("Press enter to start...")
except:
    pass

# This function runs on a background thread, and sets stop to True when the user presses enter again.
# This allows the robot to be stopped mid-drive if things are going wrong.
def interruptFunc():
    global stop
    try:
        input("Press enter to stop...\n")
    except:
        pass
    stop = True

stop = False    # Stops driving and terminates the script when set to True.
# Call interruptFunc() on a background thread, so that the script will not hang until enter is pressed.
interruptThread = Thread(target=interruptFunc)
interruptThread.daemon = True
interruptThread.start()

# Start a background thread to handle beeping when matching fails too many times in a row.
# beepThread = Thread(target=beepLoop)
# beepThread.daemon = True
# beepThread.start()

flow = None
while capture.isOpened() and not stop:  # Stop driving and terminate if stop is set to True or the camera closes unexpectedly.
    # Sometimes the camera can lag a bit (especially if RightLight is enabled) so the robot will stop moving if retrieving the next
    # frame from the camera takes too long, so that it doesn't drive off course while it can't see where it is going.
    timedOut = False
    # cameraTimeoutFunc() will stop the robot after 0.1 seconds, unless cancel() is called before then.
    cameraTimeoutThread = Timer(0.1, cameraTimeoutFunc)
    cameraTimeoutThread.daemon = True
    cameraTimeoutThread.start()
    camTime = time.time()
    frame = capture.read()[1]
    cameraTimeoutThread.cancel()    # Cancel the timeout if the frame is grabbed before the time is up.
    camTime = time.time() - camTime # Record the time taken for debugging purposes.

    # Detect key points and their descriptors in the camera frame.
    camKeypoints = fast.detect(frame, None)
    camKeypoints, camDescriptors = brief.compute(frame, camKeypoints)
    targetKeypoints, targetDescriptors = targetFeatures[featureInd]

    # Calculate the median optical flow between the camera frame and the target frame.  Will be None if matching failed.
    flow = calculateFlow(camKeypoints, camDescriptors, targetKeypoints, targetDescriptors)

    # If the flow vector has small x and y components the robot is close enough to the location that the target frame was
    # taken at to move onto the next frame.
    if flow is not None and abs(flow[0]) < 20 and abs(flow[1]) < 10:
        featureInd += 1
        if featureInd == len(targetFeatures):
            print("Arrived!")
            break   # Exit the driving loop.
        else:
            # Move onto the next frame.
            targetFrame = targetCapture.read()[1]
            # Add a frame to the debug video and log to record the frame change.
            # The debug video shows the camera frame and target frame side by side.
            # The variables written to the log can be changed here for debugging purposes.
            writer.write(np.concatenate((frame, targetFrame), axis=1))
            debugLog.append(("Target change", flow, camTime, timedOut))

    # If matching failed make the robot stop moving until there is a successfully matched frame,
    # as failed matches usually mean the robot is wildly off course or has been obstructed by something.
    if flow is None:
        setPowers(0,0)
        # Count up the number of successive fails, so that the robot can start beeping after 10.
        if fails < 10:
            fails += 1

        # Record the failed matching in the debug log.  The variables written to the log can be changed here.
        writer.write(np.concatenate((frame, targetFrame), axis=1))
        debugLog.append((flow, camTime))
        continue    # Skip the power calculations if we don't have a flow vector to use for them.
    fails = 0 # Reset the counter after a successful matching.

    # Calculate the desired forwards/backwards and rotational speeds of the robot.
    # The forward and rotation coefficients can be used to control how fast the robot moves and steers.
    #
    # If the median flow vector points downwards (and so has a positive y component as y values increase downwards)
    # the robot is going backwards, if it points upwards (and has a negative component) it is going forwards.
    forwardVelocity = -flow[1]*forwardCoeff # Positive for forwards, negative for backwards.
    # The robot is rotating in the opposite direction to the flow vectors, hence the - sign.
    rotationVelocity = -flow[0]*rotationCoeff   # Positive to turn right, negative for left.

    # The greater the forwards speed the faster both motors should turn.
    # Turning to the right requires the left motor to turn faster and the right slower, vice versa for a left turn.
    powerLeft = forwardVelocity + rotationVelocity
    powerRight = forwardVelocity - rotationVelocity

    # Add on the base power in whichever direction the motor is turning to overcome friction.
    # If the motor is supposed to be staying still don't add it.
    powerLeft += basePower if powerLeft > 0 else -basePower if powerLeft != 0 else 0
    powerRight += basePower if powerRight > 0 else -basePower if powerRight != 0 else 0
    # Ensure both powers are in the valid range -100 to 100.
    powerLeft, powerRight = max(min(powerLeft, 100), -100), max(min(powerRight, 100), -100)
    setPowers(powerLeft, powerRight)

    # Add this frame to the debug video and log.
    # The debug video shows the camera frame and target frame side by side.
    # The variables written to the log can be changed here for debugging purposes.
    writer.write(np.concatenate((frame,targetFrame), axis=1))
    debugLog.append((flow, camTime, timedOut))

# Runs when the route is completed or the user presses enter to stop the robot.
setPowers(101, 101)  # Stop the robot (101 is a stop command).
# Save the debug video and log to their respective output files.
writer.release()
with open(videoPath+"DebugLog"+str(lastNo+1)+".pkl", "wb") as file:
    cPickle.dump(debugLog, file, 2)
