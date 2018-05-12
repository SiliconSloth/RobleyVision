# Author: SiliconSloth 18/1/2018
import cv2
# PyCharm seems to need this to work properly.
try:
    from cv2 import cv2
except:
    pass
import cPickle


# This script finds key points and descriptors for each frame of a training video, for use in feature matching
# by other scripts.  Remember to set the path and filename variables before use.


videoPath = "/home/someone/RobleyVision/Recordings/" # Where video files will be saved to.

videoFile = "Training.avi"    # The video to extract features from.
featureFile = "TrainingFeatures.pkl"  # The name of the output file.

# The maximum number of key points to detect, if any more are found they will be discarded.
maxFeatures = 2000


# Load the video and create the feature detector.
capture = cv2.VideoCapture(videoPath + videoFile)
fast = cv2.FastFeatureDetector_create(threshold=16, nonmaxSuppression=True)
brief = cv2.xfeatures2d.BriefDescriptorExtractor_create(bytes=16, use_orientation=False)
# Each item of this list will contain the key points and their descriptors for one frame of the video.
features = []

while True:
    status, frame = capture.read()
    if not status:  # Stop when the video ends.
        break

    keypoints = fast.detect(frame, None)
    # The FAST detector can detect a very large number of keypoints, and processing this many points can slow down the software.
    # We only retain the best 2000 key points to reduce processing time with minimal impact on matching quality.
    keypoints.sort(key=lambda x: x.response, reverse=True)  # Sort key points from best to worst quality.
    keypoints = keypoints[:maxFeatures] # Discard all but the first 2000 key points.

    keypoints, descriptors = brief.compute(frame, keypoints)    # Compute BRIEF descriptors.
    # The key point format output by fast.detect() can not be serialized by cPickle, so we must store all the properties of
    # each key point in a normal Python tuple.
    pointTuples = []
    for point in keypoints:
        pointTuples.append((point.pt, point.size, point.angle, point.response, point.octave, point.class_id))
    # Add the list of key point tuples and descriptor array to features as a tuple.
    features.append((pointTuples, descriptors))

# Save the feature list to the output file.
with open(videoPath+featureFile, "wb") as file:
    cPickle.dump(features, file, 2)