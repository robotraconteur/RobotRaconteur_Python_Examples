import serial
import struct
import time
import RobotRaconteur as RR
import thread
import threading

serial_port_name="/dev/ttyUSB0"

class Create_impl(object):
	def __init__(self):
		self.Bump=RR.EventHook()
		self._lock=threading.RLock()
		self.packets=None
		self.play_callback=None
		
	def Drive(self, velocity, radius):
		with self._lock:
			dat=struct.pack(">B2h",137,velocity,radius)
			self._serial.write(dat)
	
	def StartStreaming(self):
		pass
		
	def StopStreaming(self):
		pass
	
	@property
	def DistanceTraveled(self):
		return 0;
		
	@property
	def AngleTraveled(self):
		return 0;
	
	@property
	def Bumpers(self):
		return 0;

	def Init(self,port):
		self._serial=serial.Serial(port="/dev/ttyUSB0",baudrate=57600)
		dat=struct.pack(">2B",128,131)
		self._serial.write(dat)
		
	def Shutdown(self):
		self._serial.close()
		
def main():
		
	#Initialize the object in the service
	obj=Create_impl()
	obj.Init(serial_port_name)
	
	#Drive a bit to show that it works
	obj.Drive(200,1000)
	time.sleep(1)
	obj.Drive(0,0)
	
	#Shutdown
	obj.Shutdown()
	
	
	
if __name__ == '__main__':
	main()