#Simple example Robot Raconteur webcam service

#Note: This example is intended to demonstrate Robot Raconteur
#and is designed to be simple rather than optimal.

import time
import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s
import threading
import numpy
import traceback
import cv2 as cv

#The service definition of this service.
webcam_servicedef="""
#Service to provide sample interface to webcams
service experimental.createwebcam

option version 0.5

struct WebcamImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct WebcamImage_size
    field int32 width
    field int32 height
    field int32 step
end struct

object Webcam
    property string Name
    function WebcamImage CaptureFrame()

    function void StartStreaming()
    function void StopStreaming()
    pipe WebcamImage FrameStream

    function WebcamImage_size CaptureFrameToBuffer()
    memory uint8[] buffer
    memory uint8[*] multidimbuffer

end object

object WebcamHost
    property string{int32} WebcamNames
    objref Webcam{int32} Webcams
end object


"""

#Class that implements a single webcam
class Webcam_impl(object):
    #Init the camera being passed the camera number and the camera name
    def __init__(self,cameraid,cameraname):
        self._lock=threading.RLock()
        self._framestream=None
        self._framestream_endpoints=dict()
        self._framestream_endpoints_lock=threading.RLock()
        self._streaming=False
        self._cameraname=cameraname

        #Create buffers for memory members
        self._buffer=numpy.array([],dtype="u1")
        self._multidimbuffer=numpy.array([],dtype="u1")



        #Initialize the camera
        with self._lock:
            self._capture=cv.CaptureFromCAM(cameraid)
            cv.SetCaptureProperty(self._capture,cv.CV_CAP_PROP_FRAME_WIDTH,320)
            cv.SetCaptureProperty(self._capture,cv.CV_CAP_PROP_FRAME_HEIGHT,240)

    #Return the camera name
    @property
    def Name(self):
        return self._cameraname

    #Capture a frame and return a WebcamImage structure to the client
    def CaptureFrame(self):
        with self._lock:
            image=RRN.NewStructure("experimental.createwebcam.WebcamImage")
            frame1=cv.QueryFrame(self._capture)
            frame=cv.GetMat(frame1)
            image.width=frame.width
            image.height=frame.height
            image.step=frame.step
            image.data=numpy.frombuffer(frame.tostring(),dtype="u1")

            return image

    #Start the thread that captures images and sends them through connected
    #FrameStream pipes
    def StartStreaming(self):
        if (self._streaming):
            raise Exception("Already streaming")
        self._streaming=True
        t=threading.Thread(target=self.frame_threadfunc)
        t.start()

    #Stop the streaming thread
    def StopStreaming(self):
        if (not self._streaming):
            raise Exception("Not streaming")
        self._streaming=False

    #FrameStream pipe member property getter and setter
    @property
    def FrameStream(self):
        return self._framestream
    @FrameStream.setter
    def FrameStream(self,value):
        self._framestream=value
        #Create the PipeBroadcaster and set backlog to 3 so packets
        #will be dropped if the transport is overloaded
        self._framestream_broadcaster=RR.PipeBroadcaster(value,3)



    #Function that will send a frame at ideally 4 fps, although in reality it
    #will be lower because Python is quite slow.  This is for
    #demonstration only...
    def frame_threadfunc(self):
        #Loop as long as we are streaming
        while(self._streaming):
            #Capture a frame
            try:
                frame=self.CaptureFrame()
            except:
                #TODO: notify the client that streaming has failed
                self._streaming=False
                return
            #Send the new frame to the broadcaster.  Use AsyncSendPacket
            #and a blank handler.  We really don't care when the send finishes
            #since we are using the "backlog" flow control in the broadcaster.
            self._framestream_broadcaster.AsyncSendPacket(frame,lambda: None)

            #Put in a 250 ms delay
            time.sleep(.25)

    #Captures a frame and places the data in the memory buffers
    def CaptureFrameToBuffer(self):

        #Capture and image and place it into the buffer
        image=self.CaptureFrame()

        self._buffer=image.data
        self._multidimbuffer=numpy.concatenate((image.data[2::3].reshape((image.height,image.width,1)),image.data[1::3].reshape((image.height,image.width,1)),image.data[0::3].reshape((image.height,image.width,1))),axis=2)



        #Create and populate the size structure and return it
        size=RRN.NewStructure("experimental.createwebcam.WebcamImage_size")
        size.height=image.height
        size.width=image.width
        size.step=image.step
        return size

    #Return the memories.  It would be better to reuse the memory objects,
    #but for simplicity return new instances when called
    @property
    def buffer(self):
        return RR.ArrayMemory(self._buffer)

    @property
    def multidimbuffer(self):
        return RR.MultiDimArrayMemory(self._multidimbuffer)


    #Shutdown the Webcam
    def Shutdown(self):
        self._streaming=False
        del(self._capture)


#A root class that provides access to multiple cameras
class WebcamHost_impl(object):
    def __init__(self,camera_names):
        cams=dict()
        for i in camera_names:
            ind,name=i
            cam=Webcam_impl(ind,name)
            cams[ind]=cam

        self._cams=cams


    #Returns a map (dict in Python) of the camera names
    @property
    def WebcamNames(self):
        o=dict()
        for ind in self._cams.keys():
            name=self._cams[ind].Name
            o[ind]=name
        return o

    #objref function to return Webcam objects
    def get_Webcams(self,ind):
        #The index for the object may come as a string, so convert to int
        #before using. This is only necessary in Python
        int_ind=int(ind)

        #Return the object and the Robot Raconteur type of the object
        return self._cams[int_ind], "experimental.createwebcam.Webcam"

    #Shutdown all the webcams
    def Shutdown(self):
        for cam in self._cams.itervalues():
            cam.Shutdown()



def main():

    RRN.UseNumPy=True

    #Initialize the webcam host root object
    camera_names=[(0,"Left"),(1,"Right")]
    obj=WebcamHost_impl(camera_names)


    #Create Local transport, start server as name, and register it
    t1=RR.LocalTransport()
    t1.StartServerAsNodeName("experimental.createwebcam.WebcamHost")
    RRN.RegisterTransport(t1)


    #Initialize the transport and register the root object
    t2=RR.TcpTransport()
    RRN.RegisterTransport(t2)
    t2.StartServer(2355)

    #Attempt to load a TLS certificate
    try:
        t2.LoadTlsNodeCertificate()
    except:
        print "warning: could not load TLS certificate"

    t2.EnableNodeAnnounce()

    RRN.RegisterServiceType(webcam_servicedef)
    RRN.RegisterService("Webcam","experimental.createwebcam.WebcamHost",obj)

    c1=obj.get_Webcams(0)[0]
    c1.CaptureFrameToBuffer()

    #Wait for the user to shutdown the service
    raw_input("Server started, press enter to quit...")

    #Shutdown
    obj.Shutdown()

    RRN.Shutdown()


if __name__ == '__main__':
    main()
