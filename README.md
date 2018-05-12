# RobleyVision

RobleyVision is a suite of Python scripts that allow a robot to automatically follow a route contained in a video file, without the need for fiducial markers or other modifications to the environment.  The [RobleyRemote](https://github.com/SiliconSloth/RobleyRemote) app can be used to manually drive the robot during training, capturing videos that the robot can play back later in order to retrace the route.

There are likely more sophisticated and robust visual navigation solutions out there, however RobleyVision uses a fairly simple algorithm that even novice programmers should be able to understand and run on widely-available hardware.

RobleyVision works best in environments where there are lots of distinctive visual features; large blank walls may make it difficult for the robot to orient itself.  The software may also be confused by environments where there are many features that look very similar, such as bushes or a room full of identical chairs.

Video: https://www.youtube.com/watch?v=apFsauh_jL8

## How it Works

RobleyVision works by comparing the images it is receiving from the camera to the current frame of the training video, to see how it needs to move so that the view from the camera matches the target frame.  Once the target and camera frames look sufficiently similar RobleyVision moves on to the next frame of the training video, gradually making its way through every frame of the video until it reaches its destination. 

The software compares two images by looking for distinctive feature points in them and seeing how these keypoints have moved between the images.  This allows it to calculate a median flow vector that summarizes how the scene in front of the camera has moved between the images, which tells the robot which direction it should move in to make the images look more similar.  For a more detailed explanation, see [Explanation.md](Explanation.md).

## Hardware Requirements

I tested this software on a differential wheeled robot that uses a laptop to run RobleyVision, connected via USB to a Tetrix Prizm controller running [RobleyMotor](https://github.com/SiliconSloth/RobleyMotor).  RobleyVision controls the robot by sending pairs of motor powers over USB to the controller, however it should be fairly easy to modify the code to accommodate other control methods.

RobleyVision requires a camera to be attached to the front of the robot, angled upwards so that the vanishing point (horizon) is at the bottom of the image (as explained in [Explanation.md](Explanation.md)).  There should not be any parts of the robot visible in the image, as this could confuse the median flow calculations, and the camera should stay in a fixed position relative to the robot at all times.  I used a USB webcam on the test robot, however the code will accept any camera that can be recognised by OpenCV, and could easily be modified to use those that can’t.  
**The camera must be angled upwards, otherwise the software will not work.**

Although RobleyVision was only tested on a laptop, it uses OpenCV’s fast FAST keypoint detector internally, so should hopefully be capable of running on less powerful hardware like a Raspberry Pi 2 or 3 at an acceptable framerate.

RobleyVision’s `Trainer.py` script requires the use of an Android phone running [RobleyRemote](https://github.com/SiliconSloth/RobleyRemote), however you can use some other method to generate training videos if you so wish.

RobleyMotor: https://github.com/SiliconSloth/RobleyMotor  
RobleyRemote: https://github.com/SiliconSloth/RobleyRemote

## Usage

Before using these scripts the configuration variables at the start of each file must be set correctly.

1. Use `Trainer.py` in conjunction with the RobleyRemote app to record a training video to the specified output file.
2. Use `FeatureDetector.py` to create a file containing the feature points and descriptors for each frame of the video.
3. Use `Reducer.py` to remove very similar looking consecutive frames from the video.  This reduces file size and processing time.
4. Use `Driver.py` with the files output by `Reducer.py` to make the robot retrace the route.  The robot should start about a metre behind the first frame of the training video, otherwise it may drive backwards to match the first frame.
5. `Viewer.py` can be used to view the log files and videos generated by `Driver.py`.

## Dependencies

- [Python 2.7](https://www.python.org/) (not 3)  
Installed with `pip`:
- [OpenCV 3](https://pypi.org/project/opencv-contrib-python/) with Python bindings and contrib modules
- [NumPy](https://pypi.org/project/numpy/)
- [PyBluez](https://pypi.org/project/PyBluez/) (If using `Trainer.py` and RobleyRemote)
- [PySerial](https://pypi.org/project/pyserial/) (If communicating with controller via USB)
