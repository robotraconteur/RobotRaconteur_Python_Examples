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
import argparse

#Port names and NodeID of this service
serial_port_name="/dev/ttyUSB0"

class Create_impl(object):
    def __init__(self):
        self.Bump=RR.EventHook()
        self._lock=threading.RLock()
        self._recv_lock=threading.RLock()        
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
                        print(struct.unpack("%sB" % len(packets),packets))
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
        strt=RRN.NewStructure('experimental.create2.SensorPacket')
        #Set the data
        strt.ID=seed
        strt.Data=data

        #Set the OutValue for the broadcaster
        self.packets.OutValue=strt

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

    #Accept the names of the nodename and port from command line
    parser = argparse.ArgumentParser(description="Example Robot Raconteur iRobot Create service")    
    parser.add_argument("--nodename",type=str,default="experimental.create2.Create",help="The NodeName to use")
    parser.add_argument("--serialport",type=str,default=serial_port_name,help="The serial port to use")
    parser.add_argument("--tcp-port",type=int,default=2354,help="The listen TCP port")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False)
    args, _ = parser.parse_known_args()

    #Initialize the object in the service
    obj=Create_impl()

    obj.Init(args.serialport)
    
    with RR.ServerNodeSetup(args.nodename,args.tcp_port,argv=sys.argv):
    
        #Register the service type and the service
        RRN.RegisterServiceTypeFromFile("experimental.create2")
        RRN.RegisterService("Create","experimental.create2.Create",obj)
    
        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")
    
        #Shutdown
        obj.Shutdown()

if __name__ == '__main__':
    main()