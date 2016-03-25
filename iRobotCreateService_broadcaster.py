#Example Robot Raconteur service in Python

import serial
import struct
import time
import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s
import threading
import numpy
import traceback
import sys

#Port names and NodeID of this service
serial_port_name="/dev/ttyUSB0"

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

class Create_impl(object):
    def __init__(self):
        self.Bump=RR.EventHook()
        self._lock=threading.RLock()
        self._recv_lock=threading.RLock()
        self._packets=None
        self._play_callback=None
        self._connected_wires=dict()

        self._lastbump=False
        self._Bumpers=0
        self._Play=False
        self._DistanceTraveled=0
        self._AngleTraveled=0
        self._streaming=False
        self._downsample=0
        self._ep=0

    def Drive(self, velocity, radius):
        with self._lock:
            dat=struct.pack(">B2h",137,velocity,radius)
            self._serial.write(dat)

    def StartStreaming(self):
        with self._lock:
            if (self._streaming):
                raise Exception("Already streaming")

            self._ep=RR.ServerEndpoint.GetCurrentEndpoint()
            #Start the thread that receives serial data
            self._streaming=True
            t=threading.Thread(target=self._recv_thread)
            t.start()
            #Send command to start streaming packets after a short delay
            time.sleep(.1)
            command=struct.pack(">6B", 148, 4, 7, 19, 20, 18)
            self._serial.write(command)



    def StopStreaming(self):
        if (not self._streaming):
            raise Exception("Not streaming")
        with self._lock:
            command=struct.pack(">2B", 150, 0)
            self._serial.write(command)
            self._streaming=False

    @property
    def DistanceTraveled(self):
        return self._DistanceTraveled

    @property
    def AngleTraveled(self):
        return self._AngleTraveled

    @property
    def Bumpers(self):
        return self._Bumpers

    @property
    def packets(self):
        return self._packets
    @packets.setter
    def packets(self,value):
        self._packets=value
        #Create the wire broadcaster
        self._packets_broadcaster=RR.WireBroadcaster(self._packets)


    @property
    def play_callback(self):
        return self._play_callback;
    @play_callback.setter
    def play_callback(self,value):
        self._play_callback=value

    def Init(self,port):
        with self._lock:
            self._serial=serial.Serial(port=port,baudrate=57600)
            dat=struct.pack(">4B",128,132,150, 0)
            self._serial.write(dat)
            time.sleep(.1)
            self._serial.flushInput()

    def Shutdown(self):
        with self._lock:
            self._serial.close()

    #Thread function that runs serial receive loop
    def _recv_thread(self):
        try:
            while self._streaming:
                if (not self._streaming): return
                self._ReceiveSensorPackets()
        except:
            #Exception will be thrown when the port is closed
            #just ignore it
            if (self._streaming):

                traceback.print_exc()
            pass

    #Receive the packets and execute the right commands
    def _ReceiveSensorPackets(self):
        while self._serial.inWaiting() > 0:
            seed=struct.unpack('>B',self._serial.read(1))[0]


            if (seed!=19):
                continue
            nbytes=struct.unpack('>B',self._serial.read(1))[0]

            if nbytes==0:
                continue

            packets=self._serial.read(nbytes)

            checksum=self._serial.read(1)


            #Send packet to the client through wire.  If there is a large backlog
            #of packets don't send
            if (self._serial.inWaiting() < 20):

                self._SendSensorPackets(seed,packets)

            readpos=0
            while (readpos < nbytes):
                id=struct.unpack('B',packets[readpos])[0]
                readpos+=1


                #Handle a bumper packet
                if (id==7):
                    flags=struct.unpack("B",packets[readpos])[0]
                    readpos+=1
                    if (((flags & 0x1)!=0) or ((flags & 0x2)!=0)):
                        if (not self._lastbump):
                            self._fire_Bump()
                        self._lastbump=True
                    else:
                        self._lastbump=False
                    self._Bumpers=flags

                #Handle distance packets
                elif (id==19):
                    try:
                        distbytes=packets[readpos:(readpos+2)]
                        self._DistanceTraveled+=struct.unpack(">h",distbytes)[0]
                        readpos+=2
                    except:
                        print struct.unpack("%sB" % len(packets),packets)
                        raise

                #Handle angle packets
                elif (id==20):
                    distbytes=packets[readpos:(readpos+2)]
                    self._DistanceTraveled+=struct.unpack(">h",distbytes)[0]
                    readpos+=2

                #Handle buttons packets
                elif (id==18):
                    buttons=struct.unpack("<B",packets[readpos])[0]
                    play=buttons & 0x1
                    if (play==1):
                        if (not self._Play):
                            self._play()
                        self._Play=True
                    else:
                        self._Play=False
                    readpos+=1
                else:
                    readpos+=1


    def _SendSensorPackets(self,seed,packets):



        #Pack the data into the structure to send to the lient
        data=numpy.frombuffer(packets,dtype='u1')
        #Create the new structure using the "NewStructure" function
        strt=RRN.NewStructure('experimental.create.SensorPacket')
        #Set the data
        strt.ID=seed
        strt.Data=data

        #Set the OutValue for the broadcaster
        self._packets_broadcaster.OutValue=strt

    #Fire the bump event, all connected clients will receive
    def _fire_Bump(self):
        self.Bump.fire()

    def _play(self):
        if (self._ep==0):
            return

        try:
            cb_func=self.play_callback.GetClientFunction(self._ep)
            notes=cb_func(self._DistanceTraveled, self._AngleTraveled)
            notes2=list(notes) + [141,0]

            command=struct.pack("%sB" % (5+len(notes)),140,0,len(notes)/2,*notes2)
            with self._lock:
                self._serial.write(command)


        except:
            traceback.print_exc()



def main():

    #Enable numpy
    RRN.UseNumPy=True

    #Initialize the object in the service
    obj=Create_impl()

    if (len(sys.argv) >=2):
        port=sys.argv[1]
    else:
        port=serial_port_name

    obj.Init(port)

    #Create Local transport, start server as name, and register it
    t1=RR.LocalTransport()
    t1.StartServerAsNodeName("experimental.create.Create")
    RRN.RegisterTransport(t1)

    #Create the transport, register it, and start the server
    t2=RR.TcpTransport()
    RRN.RegisterTransport(t2)
    t2.StartServer(2354) #random port, any unused port is fine

    #Attempt to load a TLS certificate
    try:
        t2.LoadTlsNodeCertificate()
    except:
        print "warning: could not load TLS certificate"

    t2.EnableNodeAnnounce()

    #Register the service type and the service
    RRN.RegisterServiceType(create_servicedef)
    RRN.RegisterService("Create","experimental.create.Create",obj)

    #Wait for the user to stop the server
    raw_input("Server started, press enter to quit...")

    #Shutdown
    obj.Shutdown()

    #You MUST shutdown or risk segfault...
    RRN.Shutdown()

if __name__ == '__main__':
    main()