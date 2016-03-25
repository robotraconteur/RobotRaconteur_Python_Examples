#Example iRobot Create client in Python

from RobotRaconteur.Client import *
import time
import numpy
import sys

#Function to call when "Bump" event occurs
def Bumped():
    print "Bump!!"

def main():

    url='rr+tcp://localhost:2354?service=Create'
    if (len(sys.argv)>=2):
        url=sys.argv[1]

    #Instruct Robot Raconteur to use NumPy
    RRN.UseNumPy=True

    #Connect to the service
    c=RRN.ConnectService(url)

    #Start streaming data packets
    c.StartStreaming()

    #Add a function handler for the "Bump" event
    c.Bump += Bumped

    #Connect a WireConnection to the "packets" wire
    wire=c.packets.Connect()

    #Add a callback function for when the wire value changes
    wire.WireValueChanged+=wire_changed

    #Set the play_callback function for this client
    c.play_callback.Function=play_callback

    #Drive a bit
    c.Drive(100,1000)
    time.sleep(5)
    c.Drive(0,1000)
    time.sleep(10)

    #Stop streaming data
    c.StopStreaming()

#Function to call when the wire value changes
def wire_changed(w,value,time):

    val=w.InValue
    #Print the new value to the console.  Comment out this line
    #to see the other output more clearly
    print val.Data

#Callback for when the play button is pressed on the Create
def play_callback(dist,angle):
    return numpy.array([69,16,60,16,69,16],dtype='u1')


if __name__ == '__main__':
    main()
