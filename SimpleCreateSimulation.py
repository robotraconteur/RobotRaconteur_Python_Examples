#Example simulation of the iRobot Create using Robot 
#Raconteur for communication

#This example is intended to demonstrate how a service
#can utilize Robot Raconteur. It is not intended to be a high
#fidelity simulation of the robot. 

import time
import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s
import threading
import numpy as np
import traceback
import sys
import cv2
import struct

#The service definition of this service.
create_servicedef="""
#Service to provide sample interface to the iRobot Create
service experimental.create

option version 0.5

struct SensorPacket
    field uint8 ID
    field uint8[] Data
end struct

object Create
    option constant int16 DRIVE_STRAIGHT 32767
    option constant int16 SPIN_CLOCKWISE -1
    option constant int16 SPIN_COUNTERCLOCKWISE 1

    function void Drive(int16 velocity, int16 radius)

    function void StartStreaming()
    function void StopStreaming()

    property int32 DistanceTraveled
    property int32 AngleTraveled
    property uint8 Bumpers

    event Bump()

    wire SensorPacket packets

    callback uint8[] play_callback(int32 DistanceTraveled, int32 AngleTraveled)

end object
"""

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

class CreateSim(object):
    
    def __init__(self):
        self._x=0.0
        self._y=0.0
        self._theta=0.0
        self._v=0.0
        self._v_t=0.0
        self._thread=None
        self._thread_event=threading.Event()
        self._mylock=threading.Lock()
        self._running=False
        self.Packets=RR.EventHook()
        #iRobot Create uses a 15 ms update period
        self._dt=0.015
        self._bump=0
        
    def __thread_func(self):
        start_time=time.time()
        next_time=start_time        
        while self._running:
            
            with self._mylock:
                x1= self._x + -np.sin(self._theta)*self._v*self._dt
                y1= self._y + np.cos(self._theta)*self._v*self._dt
                theta1 = self._theta + self._v_t*self._dt
                
                dist=0
                ang=0
                
                if (x1 < 1450 and x1 > -1450 and y1 < 1050 and y1 > -1050):
                    self._x=x1
                    self._y=y1
                    self._theta=theta1
                    
                    dist = self._v * self._dt
                    ang = self._v_t * self._dt * 180.0/np.pi
                    
                    if (np.abs(self._v) > 0 or np.abs(self._v_t) > 0):
                        self._bump = 0
                    
                else:
                    self._bump = 3
            
                p = bytearray(struct.pack(">5BhBh2B", 19, 10, 7, self._bump, 19, dist, 20, ang, 18, 0))
                #Checksum
                p.append(int(0x100 - (np.sum(p[1:]) % 0x100)))
            
            self.Packets.fire(p)
                                        
            next_time+=self._dt
            dt=next_time-time.time()
            if dt > 0:
                self._thread_event.wait(dt)         

    def start_sim(self):
        with self._mylock:
            if (self._running):
                raise Exception("Already running")
            self._thread_event.clear()            
            self._running=True
            self._thread=threading.Thread(target=self.__thread_func)
            self._thread.start()
    
    def stop_sim(self):
        with self._mylock:
            self._running=False
            self._thread_event.set()
            self._thread=None

    def drive(self, velocity, radius):
        with self._mylock:
            
            v=np.clip(velocity, -500, 500)
            v_t=0
            
            if radius == 0 or radius == 1:                
                v_t=v/250.0
                v=0
            elif radius == -1:
                v_t=-v/250.0
                v=0
            elif radius == 32767 or radius == 32768:
                v_t=0
            else:
                r=np.clip(radius, -2000, 2000)
                v_t=float(v)/float(r)                
                        
            self._v=float(v)            
            self._v_t=float(v_t)
    

    def get_image(self):
        with self._mylock:
            x=self._x
            y=self._y
            theta=self._theta
        
        return self._draw_image(x, y, theta)

    def _draw_image(self, x, y, theta):        
        #Draw a simple representation of the simulation
        img = np.zeros((240,320,3), np.uint8) + 255
        cv2.rectangle(img, (0,0), (319,239), (0,0,0), 2)
        
        p=np.array([[x/10+160], [120-y/10]])
        c1=np.cos(theta)
        s1=np.sin(theta)
        R=np.matrix([[c1,s1],[-s1,c1]])    
        
        def t(p1):
            return tuple(((p+R*((p1[0],),(p1[1],))))[:,0].astype(int))
        
        cv2.ellipse(img, t((0,0)), (15,15), 0, 0, 360, (0,0,0), 1)
        cv2.line(img, t((0,10)), t((0,-10)), (0,0,0), 1 )
        cv2.line(img, t((-8,-2)), t((0,-10)), (0,0,0), 1 )
        cv2.line(img, t((8,-2)), t((0,-10)), (0,0,0), 1 )
        
        return img

class Create_impl(object):
    def __init__(self, sim):
        self._sim=sim
        self.Bump=RR.EventHook()        
        sim.Packets += self._packet_handler
        self._Bumpers=0
        self._DistanceTraveled=0
        self._AngleTraveled=0
        self._streaming=False
        self._mylock=threading.Lock()
        self._packets=None
        self._packets_broadcaster=None
        
    def Drive(self, velocity, radius):
        self._sim.drive(velocity, radius)
        
    def _packet_handler(self, p):
        
        with self._mylock:
            if not self._streaming:
                return
            (bump, dist, ang, buttons)=struct.unpack('>3xBxhxhxBx',str(p))
            #print [bump, dist, ang, buttons]
            
            if (bump !=0 and self._Bumpers == 0):
                self.Bump.fire()
            self._Bumpers=bump
            self._DistanceTraveled+=dist
            self._AngleTraveled+=ang
            
            self._SendSensorPackets(p[0], p[1:-1])
    
    @property
    def DistanceTraveled(self):
        with self._mylock:
            return self._DistanceTraveled

    @property
    def AngleTraveled(self):
        with self._mylock:
            return self._AngleTraveled

    @property
    def Bumpers(self):
        with self._mylock:
            return self._Bumpers
        
    @property
    def packets(self):
        return self._packets
    @packets.setter
    def packets(self,value):
        self._packets=value
        #Create the wire broadcaster
        self._packets_broadcaster=RR.WireBroadcaster(self._packets)
    
    def _SendSensorPackets(self,seed,packets):

        #Pack the data into the structure to send to the client
        data=np.frombuffer(packets,dtype='u1')
        #Create the new structure using the "NewStructure" function
        strt=RRN.NewStructure('experimental.create.SensorPacket')
        #Set the data
        strt.ID=seed
        strt.Data=data

        #Set the OutValue for the broadcaster
        self._packets_broadcaster.OutValue=strt
        
    def StartStreaming(self):
        with self._mylock:
            if self._streaming:
                raise Exception("Already streaming")
            self._streaming=True
            
    def StopStreaming(self):
        with self._mylock:
            if not self._streaming:
                raise Exception("Not streaming")
            self._streaming=False

#Class that implements a single webcam
class Webcam_impl(object):
    #Init the camera being passed the camera number and the camera name
    def __init__(self,cameraid,cameraname,host):
        self._lock=threading.RLock()
        self._framestream=None
        self._framestream_endpoints=dict()
        self._framestream_endpoints_lock=threading.RLock()
        self._streaming=False
        self._cameraname=cameraname
        self._cameraid=cameraid

        #Create buffers for memory members
        self._buffer=np.array([],dtype="u1")
        self._multidimbuffer=np.array([],dtype="u1")

        self._host=host

    #Return the camera name
    @property
    def Name(self):
        return self._cameraname

    #Capture a frame and return a WebcamImage structure to the client
    def CaptureFrame(self):
        with self._lock:
            image=RRN.NewStructure("experimental.createwebcam.WebcamImage")
            frame=self._host.sim.get_image()
            if self._cameraid > 0:
                frame=cv2.flip(frame,1)
            image.width=frame.shape[1]
            image.height=frame.shape[0]
            image.step=frame.shape[1]*3
            image.data=frame.reshape(frame.size, order='C')

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
        
        start_time=time.time()
        next_time=start_time
        
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

            #Put in a 50 ms delay
            next_time+=0.05
            dt=next_time-time.time()
            if dt > 0:
                time.sleep(dt)

    #Captures a frame and places the data in the memory buffers
    def CaptureFrameToBuffer(self):

        #Capture and image and place it into the buffer
        image=self.CaptureFrame()

        self._buffer=image.data
        self._multidimbuffer=np.concatenate((image.data[2::3].reshape((image.height,image.width,1)),image.data[1::3].reshape((image.height,image.width,1)),image.data[0::3].reshape((image.height,image.width,1))),axis=2)



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
    def __init__(self,camera_names,sim):
        cams=dict()
        for i in camera_names:
            ind,name=i
            cam=Webcam_impl(ind,name,self)
            cams[ind]=cam

        self._cams=cams
        self.sim=sim


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

if __name__ == '__main__':
    
    RRN.UseNumPy=True
    
    theta=0.0
    x=0.0
    xdir=1.0
    cv2.namedWindow('Simple Create Simulation')
    
    create=CreateSim()
    create.start_sim()
    
    obj=Create_impl(create)
    #Initialize the webcam host root object
    camera_names=[(0,"Left"),(1,"Right")]
    obj2=WebcamHost_impl(camera_names,create)
    
    #Create Local transport, start server as name, and register it
    t1=RR.LocalTransport()
    t1.StartServerAsNodeName("experimental.createsimulation")
    RRN.RegisterTransport(t1)

    #Create the transport, register it, and start the server
    t2=RR.TcpTransport()
    RRN.RegisterTransport(t2)
    t2.StartServer(62354) #random port, any unused port is fine

    #Attempt to load a TLS certificate
    try:
        t2.LoadTlsNodeCertificate()
    except:
        print "warning: could not load TLS certificate"

    t2.EnableNodeAnnounce()

    #Register the service type and the service
    RRN.RegisterServiceType(create_servicedef)
    RRN.RegisterServiceType(webcam_servicedef)
    RRN.RegisterService("Create","experimental.create.Create",obj)
    RRN.RegisterService("Webcam","experimental.createwebcam.WebcamHost",obj2)
    
    start_time=time.time()
    
    while True:        
       
        img=create.get_image()
        
        if cv2.getWindowProperty('Simple Create Simulation',0) < 0:
            break
        cv2.imshow('Simple Create Simulation', img)
        if cv2.waitKey(25) != -1:
            break        
       
        
    cv2.destroyAllWindows()
    
    create.stop_sim()
    
    RRN.Shutdown()
