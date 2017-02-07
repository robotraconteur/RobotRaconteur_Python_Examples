# Robot Raconteur Python Examples

This repository contains several examples demonstrating how to use the Robot Raconteur library in Python programs. The services are design to control an iRobot Create robot and read images from webcams. There is also a simple simulation that can be used if a physical robot is not available.

### Installation

The examples require Python 2.7, opencv, NumPy, PySerial and Robot Raconteur. They can be acquired at the following links:

#### Windows

Python: https://www.python.org/downloads/release/python-2712/
OpenCV: http://docs.opencv.org/trunk/d5/de5/tutorial_py_setup_in_windows.html
NumPy: See the OpenCV page
pyserial: https://pypi.python.org/pypi/pyserial/2.7
Robot Raconteur: http://robotraconteur.com/download

#### Linux

    sudo apt-get install python2.7 python-opencv python-serial
    
Robot Raconteur: http://robotraconteur.com/download

#### OSX

    sudo easy_install pip
    sudo pip install python-opencv
    sudo pip install pyserial
    
Robot Raconteur: http://robotraconteur.com/download

### SimpleCreateSimulation.py service

SimpleCreateSimulation.py provides a simple simulation of the Create robot and webcams. The Create is simulated as a circle with an arrow pointing toward the front of the robot. It will respond to input commands in a similar manner to the real robot. The webcam service will transmit the generated image in place of the real image that a webcam would provide. The right camera is flipped to demonstrate the difference between cameras.

Unlike the other example services, both the Create and webcam services are run on the same node. The URLs are as follows if running on the same computer:

Create: `rr+tcp://localhost:62354?service=Create`
    
Webcam: `rr+tcp://localhost:62354?service=Webcam`

To run the service:

    python SimpleCreateSimulation.py

The various example clients can be executed with the following commands:

    python iRobotCreateClient.py rr+tcp://localhost:62354?service=Create
    python iRobotCreateClient_joystick.py rr+tcp://localhost:62354?service=Create
    python SimpleWebcamClient.py rr+tcp://localhost:62354?service=Webcam
    python SimpleWebcamClient_streaming.py rr+tcp://localhost:62354?service=Webcam
    python SimpleWebcamClient_memory.py rr+tcp://localhost:62354?service=Webcam

If running the client on a different computer, relpace "localhost" with the target IP address.

### iRobotCreateService.py

iRobotCreateService.py controls the iRobot Create robot through a serial port, and provides some basic sensor information.

    python iRobotCreateService.py /dev/ttyUSB0

Replace /dev/ttyUSB0 with the appropriate port name.

To run the clients:

    python iRobotCreateClient.py rr+tcp://localhost:2354?service=Create
    python iRobotCreateClient_joystick.py rr+tcp://localhost:2354?service=Create

### iRobotCreateService_broadcaster.py

iRobotCreateService_broadcaster.py controls the iRobot Create robot through a serial port, and provides some basic sensor information. This service is similar to iRobotCreateService.py, except it uses a different method to transmit "wire" data.

    python iRobotCreateService_broadcaster.py /dev/ttyUSB0

Replace /dev/ttyUSB0 with the appropriate port name.

To run the clients:

    python iRobotCreateClient.py rr+tcp://localhost:2354?service=Create
    python iRobotCreateClient_joystick.py rr+tcp://localhost:2354?service=Create
    
### SimpleWebcamService.py

SimpleWebcamService.py provides access to two webcams.

    python SimpleWebcamService.py
    
To run the clients:

    python SimpleWebcamClient.py rr+tcp://localhost:2355?service=Webcam
    python SimpleWebcamClient_streaming.py rr+tcp://localhost:2355?service=Webcam
    python SimpleWebcamClient_memory.py rr+tcp://localhost:2355?service=Webcam
