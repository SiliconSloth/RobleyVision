# Author: SiliconSloth 18/1/2018
import cv2
# PyCharm seems to need this to work properly.
try:
    from cv2 import cv2
except:
    pass
import numpy as np
import cPickle


# In a raw training video there is often very little difference between consecutive frames, so using every single frame of the video
# is a waste of memory and processing time.  This script compares consecutive frames of a training video using the key points produced
# by FeatureDetector.py to see how similar they are, and if there is very little movement between them some frames will be removed from the video
# so that each frame shows noticeable change since the last.

videoPath = "/home/someone/RobleyVision/Recordings/"  # Where video files will be saved to.

# Input file names.
videoFile = "Training.avi"
featureFile = "TrainingFeatures.pkl"

# The filenames to which the reduced video and feature files should be saved.
reducedVideoFile = "TrainingReduced.avi"
reducedFeatureFile = "ReducedFeatures.pkl"

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
    if not matches:
        return None

    # Create a 2D array in which each row contains the x and y components of one optical flow vector.
    # Do this by subtracting the coordinates of the first key point of each match from the second key point's coordinates.
    flowVectors = np.array([(keypoints2[m.trainIdx].pt[0] - keypoints1[m.queryIdx].pt[0],
                             keypoints2[m.trainIdx].pt[1] - keypoints1[m.queryIdx].pt[1]) for m in matches])
    # Find the medians of all the x and y components.
    return np.median(flowVectors, 0)


# Create the brute-force matcher.
bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

# Load the video and feature list.
capture = cv2.VideoCapture(videoPath + videoFile)
dimensions = int(capture.get(3)), int(capture.get(4))   # Find the width and height of the video.
with open(videoPath+featureFile, "rb") as file:
    features = cPickle.load(file)

# Lists to store the reduced video frames and feature tuples.
outputVideo = []
outputFeatures = []

frame2, keypoints2, descriptors2 = None, None, None
for i, (pointTuples, descs) in enumerate(features):
    # frame1 is the previous video frame and frame2 is the one just read from the video capture.
    # Set frame1 to hold the frame taken from the capture on the last iteration, or None if this is the first frame.
    frame1 = frame2
    keypoints1 = keypoints2
    descriptors1 = descriptors2

    # Read the next video frame.
    frame2 = capture.read()[1]
    # Convert the point tuples made by FeatureDetector.py back to OpenCV KeyPoint objects.
    keypoints2 = []
    for point in pointTuples:
        keypoints2.append(cv2.KeyPoint(point[0][0], point[0][1], point[1], point[2], point[3], point[4], point[5]))
    descriptors2 = descs

    # If keypoints1 is None this is the first frame.  For some reason the first frame of the video always
    # seems to just contain rubbish, so ignore it.
    if keypoints1 is None:
        # Skip the code that adds this frame to the output list, removing it from the output video.
        continue

    # Calculate the median optical flow between the two frames.
    flow = calculateFlow(keypoints1, descriptors1, keypoints2, descriptors2)
    # If flow is None that means matching failed and the flow could not be found, implying that the frames are very different
    # and so should both be kept.
    # If this is the last frame in the video, it should always be kept to ensure that the robot drives to the very end of the route.
    # If neither of the above is true and the x and y components of the optical flow are very small, this suggests there was very little movement
    # between the two frames so the second one should be removed from the video.
    if flow is not None and i < len(features)-1 and abs(flow[0]) < 20 and abs(flow[1]) < 30:
        # Make sure frame2 contains the latest frame in the video, ready for the next iteration.
        frame2 = frame1
        keypoints2 = keypoints1
        descriptors2 = descriptors1
        continue    # Skip adding the frame.

    # Add the frame and its features to the output lists, if the frame was not skipped.
    outputVideo.append(frame2)
    outputFeatures.append((pointTuples, descs))

# Save the reduced video to the output file.
writer = cv2.VideoWriter(videoPath+reducedVideoFile, cv2.VideoWriter_fourcc(*"MJPG"), 20, dimensions, True)
for frame in outputVideo:
    writer.write(frame)
writer.release()

# Save the reduced feature lists to the output file.
with open(videoPath+reducedFeatureFile, "wb") as file:
    cPickle.dump(outputFeatures, file, 2)