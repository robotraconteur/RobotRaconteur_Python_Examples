# Robot Raconteur Python Examples

This repository contains several examples demonstrating how to use the Robot Raconteur library in Python programs. The services are design to control an iRobot Create robot and read images from webcams. There is also a simple simulation and a Gazebo
simulation that can be used if a physical robot is not available.

- [Robot Raconteur Python Examples](#robot-raconteur-python-examples)
  - [Installation](#installation)
    - [Windows](#windows)
    - [Ubuntu](#ubuntu)
    - [Debian](#debian)
    - [Raspbian](#raspbian)
    - [OSX](#osx)
  - [SimpleCreateSimulation.py service](#simplecreatesimulationpy-service)
  - [iRobotCreateService.py](#irobotcreateservicepy)
  - [iRobotCreateService_broadcaster.py](#irobotcreateservicebroadcasterpy)
  - [SimpleWebcamService.py](#simplewebcamservicepy)
  - [Gazebo Simulator](#gazebo-simulator)


## Installation

### Windows

Install Python 3.7 or greater: https://www.python.org/downloads/

Install the required packages using pip:

    python -m pip install --user robotraconteur numpy opencv-contrib-python pygame pyserial

### Ubuntu

Install the PPA for Robot Raconteur. *This only needs to be done once*

    sudo add-apt-repository ppa:robotraconteur/ppa
    sudo apt-get update

Install the required packages:

    sudo apt-get install python3-robotraconteur python3-opencv python3-serial python3-numpy python3-pygame
    
**Note: Use python3 instead of Python on Linux**

### Debian

Install the private repository for Robot Raconteur. *This only needs to be done once*

    sudo apt install apt-transport-https dirmngr gnupg ca-certificates
    wget -O - https://robotraconteur.github.io/robotraconteur-apt/wasontech-apt.gpg.key | sudo apt-key add -
    echo "deb https://robotraconteur.github.io/robotraconteur-apt/debian buster main" | sudo tee /etc/apt/sources.list.d/robotraconteur.list
    sudo apt update

Install the required packages:

    sudo apt-get install python3-robotraconteur python3-opencv python3-serial python3-numpy python3-pygame
    
**Note: Use python3 instead of Python on Linux**

### Raspbian

Install the private repository for Robot Raconteur. *This only needs to be done once*

**Raspberry Pi OS (raspbian) for 64-bit uses the standard debian repository. This is only for 32-bit ARMv6 installation.**

    sudo apt install apt-transport-https dirmngr gnupg ca-certificates
    wget -O - https://robotraconteur.github.io/robotraconteur-apt/wasontech-apt.gpg.key | sudo apt-key add -
    echo "deb https://robotraconteur.github.io/robotraconteur-apt/raspbian buster main" | sudo tee /etc/apt/sources.list.d/robotraconteur.list
    sudo apt update

Install the required packages:

    sudo apt-get install python3-robotraconteur python3-opencv python3-serial python3-numpy python3-pygame
    
**Note: Use python3 instead of Python on Linux**

### OSX

Install Python 3.7 or greater: https://www.python.org/downloads/

Install the required packages using pip:

    python3 -m pip install --user robotraconteur numpy opencv-contrib-python pygame pyserial

**Note: Use python3 instead of Python on Mac OS**
    
## SimpleCreateSimulation.py service

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

## iRobotCreateService.py

iRobotCreateService.py controls the iRobot Create robot through a serial port, and provides some basic sensor information.

    python iRobotCreateService.py /dev/ttyUSB0

Replace /dev/ttyUSB0 with the appropriate port name.

To run the clients:

    python iRobotCreateClient.py rr+tcp://localhost:2354?service=Create
    python iRobotCreateClient_joystick.py rr+tcp://localhost:2354?service=Create

## iRobotCreateService_broadcaster.py

iRobotCreateService_broadcaster.py controls the iRobot Create robot through a serial port, and provides some basic sensor information. This service is similar to iRobotCreateService.py, except it uses a different method to transmit "wire" data.

    python iRobotCreateService_broadcaster.py /dev/ttyUSB0

Replace /dev/ttyUSB0 with the appropriate port name.

To run the clients:

    python iRobotCreateClient.py rr+tcp://localhost:2354?service=Create
    python iRobotCreateClient_joystick.py rr+tcp://localhost:2354?service=Create
    
## SimpleWebcamService.py

SimpleWebcamService.py provides access to two webcams.

    python SimpleWebcamService.py
    
To run the clients:

    python SimpleWebcamClient.py rr+tcp://localhost:2355?service=Webcam
    python SimpleWebcamClient_streaming.py rr+tcp://localhost:2355?service=Webcam
    python SimpleWebcamClient_memory.py rr+tcp://localhost:2355?service=Webcam

## Gazebo Simulator

The Robot Raconteur training simulator contains an iRobot Create scene. Install the training simulator
using the instructions here:

https://github.com/robotraconteur-contrib/robotraconteur_training_sim

Conda is used to install the training simulator.

To start the Gazebo simulation on Windows, run:

    conda activate rr_training_sim
    cd %CONDA_PREFIX%\gz_example\create
    run_sim

To start the Gazebo simulation on Linux, run:

    source ~/miniconda3/bin/activate
    conda activate rr_training_sim
    cd $CONDA_PREFIX/gz_example/create
    ./run_sim

Two example services are available that use the simulator. Run each service in a separate terminal after starting
the Gazebo simulation:

    python Gazebo_iRobotCreateService.py

and

    python Gazebo_SimpleWebcamService.py

These services can be used with the rest of the example clients, as they have the same interfaces
as the real drivers.