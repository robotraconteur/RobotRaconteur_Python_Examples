#Simple example Robot Raconteur webcam client
#This program will capture a frame from both webcams and show it
#on the screen
from RobotRaconteur.Client import *
import time
import numpy
import cv2
import sys

#Function to take the data structure returned from the Webcam service
#and convert it to an OpenCV array
def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

#Main program
def main():

    url='rr+tcp://localhost:2355/?service=Webcam'
    if (len(sys.argv)>=2):
        url=sys.argv[1]

    #Start up Robot Raconteur and connect, standard by this point    
    c_host=RRN.ConnectService(url)

    #Use objref's to get the cameras. c_host is a "WebcamHost" type
    #and is used to find the webcams
    c1=c_host.get_Webcams(0)
    c2=c_host.get_Webcams(1)

    #Pull a frame from each camera, c1 and c2
    frame1=WebcamImageToMat(c1.CaptureFrame())
    frame2=WebcamImageToMat(c2.CaptureFrame())

    #Show the images
    cv2.imshow(c1.Name,frame1)
    cv2.imshow(c2.Name,frame2)

    #CV wait for key press on the image window and then destroy
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
