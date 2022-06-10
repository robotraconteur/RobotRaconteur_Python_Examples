# Simulate iRobot Create robot driver

# See the Robot Raconteur Training Simulator https://github.com/robotraconteur-contrib/robotraconteur_training_sim
# for the simulation environment

import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s
import threading
import sys
import argparse
import numpy as np
import traceback

# This example currently only implements the "Drive" command

class Create_impl(object):
    def __init__(self):
        self.Bump=RR.EventHook()
        self._lock=threading.RLock()

        self._gazebo_server = None
        self._pose_dtype = None
        self._robot_pose = [0,0,0]
        self._omega = 0.0
        self._vel = 0.0

    def Drive(self, velocity, radius):
        if abs(radius) <= 130.0:
            self._omega = (velocity/130.0)
            if (radius < 0):
                self._omega = -self._omega
            self._vel = self._omega * radius
        else:
            self._omega = float(velocity)/float(radius)
            self._vel = float(velocity)

    def _drive_timer_cb(self, timer_evt):
        
        try:
            res, gazebo_client = self._gazebo_server.TryGetDefaultClient()
            if not res:
                return
            gazebo_world = gazebo_client.get_worlds("default")
            gazebo_create = gazebo_world.get_models("rr_create")
            if self._pose_dtype is None:
                self._pose_dtype = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose',gazebo_create)

            self._robot_pose[2] += self._omega * 0.01
            self._robot_pose[0] += np.cos(self._robot_pose[2]) * (self._vel * 1e-3) * 0.01
            self._robot_pose[1] += np.sin(self._robot_pose[2]) * (self._vel * 1e-3) * 0.01

            pose = np.zeros((1,),dtype=self._pose_dtype)
            pose[0]['position']['x'] = self._robot_pose[0]
            pose[0]['position']['y'] = self._robot_pose[1]
            pose[0]['orientation']['w'] = np.cos(self._robot_pose[2]/2.0)
            pose[0]['orientation']['z'] = np.sin(self._robot_pose[2]/2.0)

            gazebo_create.setf_world_pose(pose)
        except:
            self._pose_dtype = None
            traceback.print_exc()

    def StartStreaming(self):
        return
    
    def StopStreaming(self):
        return

    @property
    def DistanceTraveled(self):
        return -1

    @property
    def AngleTraveled(self):
        return -1

    @property
    def Bumpers(self):
        return 0
    
    @property
    def play_callback(self):
        return self._play_callback;
    @play_callback.setter
    def play_callback(self,value):
        self._play_callback=value

    def Init(self, gazebo_url):
        self._gazebo_server = RRN.SubscribeService(gazebo_url)
        
        self._drive_timer = RRN.CreateTimer(0.01,self._drive_timer_cb, False)
        self._drive_timer.Start()


def main():

    #Accept the names of the nodename and port from command line
    parser = argparse.ArgumentParser(description="Example Robot Raconteur iRobot Create service")    
    parser.add_argument("--nodename",type=str,default="experimental.create2.Create",help="The NodeName to use")
    parser.add_argument("--gazebo-url",type=str,
        default="rr+tcp://localhost:11346/?service=GazeboServer",
        help="The URL for the Gazebo plugin. Default 'rr+tcp://localhost:11346/?service=GazeboServer'")
    parser.add_argument("--tcp-port",type=int,default=2354,help="The listen TCP port")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False)
    args, _ = parser.parse_known_args()

    #Initialize the object in the service
    obj=Create_impl()
   
    with RR.ServerNodeSetup(args.nodename,args.tcp_port,argv=sys.argv):
    
        #Register the service type and the service
        RRN.RegisterServiceTypeFromFile("experimental.create2")
        RRN.RegisterService("Create","experimental.create2.Create",obj)

        obj.Init(args.gazebo_url)
    
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