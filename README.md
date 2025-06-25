# MARC — My Automated Robotic Companion 

**MARC** is an intelligent robotic arm system that listens to voice commands, identifies objects in cluttered environments, and autonomously performs pick-and-place and handover tasks. Designed as a home/industrial assistant, MARC combines voice recognition, computer vision, and robotic manipulation into a single modular ROS2 framework.

## Features

-  **Voice-Controlled Operation**: Natural language commands like “pick up the hammer” or “hand me the screwdriver”
-  **Object Detection in Clutter**: Real-time scene segmentation and object classification
-  **Autonomous Manipulation**: Pick and place using inverse kinematics and motion planning
-  **Modular ROS2 Nodes**: Easy to extend and test individual subsystems
-  **Robot-Agnostic Control**: Works with physical or simulated arms (MoveIt-compatible)

## System Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- OpenCV ≥ 4.5
- MoveIt2
- Microphone and camera access for voice and vision

### Dependencies

```bash
sudo apt install \
  ros-humble-moveit \
  ros-humble-image-transport ros-humble-cv-bridge \
  ros-humble-audio-common \
  python3-speechrecognition \
  python3-pyaudio \
```
```bash
pip install opencv-contrib-python numpy vosk faster-whisper
```
install TensorFlow from the official Site: https://www.tensorflow.org/install

```bash
cd src
```
follow the instructions in this repo to have pymoveit2: https://github.com/AndrejOrsula/pymoveit2

## Running the scripts:
In separate terminals, run the following (after sourcing your ROS2 environment):
### Terminal 1: Video feed publisher
```bash
python3 video_publisher.py
```
### Terminal 2: Voice command listener
```bash
python3 voice_det.py
```
### Terminal 3: Arm controller
```bash
python3 marc_mover.py
```
### Terminal 4: Video feedback
```bash
python3 video_subscriber.py
```

## Click-Based Pick and Place
If you'd prefer to click on a point in the image to move the robot there:
### Terminal 1: Click detector
```bash
python3 detector_click.py
```
### Terminal 2: Arm controller
```bash
python3 marc_mover.py
```

### topics: 
- voice cmds: cmdpickplace
- wrapped image: wrapped_image
- coordinates: click_coordinates
## Autonomous Keyboard typing

Run 
``` 
python3 key.py
```
**NOTE:** This script was written to test out the arm's control accuracy and planning speed according to various configurations. So, hardcoding the keyboard key distances was what we chose since it was apt for our use case. So feel free to chage the values according to your keyboard or better, use OCR based typing.

## More about Video Feed
Our setup included a mobile phone mounted on a phone holder attached to the desk. 

The phone faced down showing the working desk with a printed aruco grid-map.
[GRID MAP](src/stuff/map.png)
This grid map is used to rectify skewed views due to angled perspectives, thus always provifding a rectangular and consistent workarea coordinates. 
[WS](src/stuff/ws.jpg)
[wrapped](src/stuff/output_wrapped.jpg)
We used Droid Cam: https://droidcam.app/ to stream the video feed in realTime.
If such a setup is used, then, replace the source to the url.

In video_publisher.py and detector_click.py, make sure to change:
```
cv2.VideoCapture(0)
```
to 
```
cv2.VideoCapture("192.168.2.11/video_feed")
```

## Detection
Thes setup currently involves the use of Roboflow API  and inference model for the tool detection. (https://universe.roboflow.com/)
To get it to work 

```bash
self.client = InferenceHTTPClient(
            api_url="API URL HERE",
            api_key="API KEY HERE"
        )
```
replace API URL and API KEY here

The inference model can be changed here to suit your needs too.
```
result = self.client.infer(temp_path, model_id="mechanical-tools-10000/3")
```

## NOTE
If a different arm is being used, replace marc_urdf with the custom urdf.
generate new moveit config with [MoveitSetupAssistant](https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)