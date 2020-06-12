import turtle
import copy
from tkthread import tk, TkThread
import threading, time
import traceback
#import RR library
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

#object definition
turtlebot_interface="""
#Service to provide virtual interface to the Duckiebot Drive
service experimental.turtlebot_create

stdver 0.9
struct pose
    field single x
    field single y
    field single angle
end

object turtlesim
	function void drive(string turtlename,single move_speed, single turn_speed)
	function void setpose(string turtlename,pose desire_pose)
	function void setpose_relative(string turtlename,pose desire_pose)
	function void setpencolor(string turtlename,string color)
	function void reset()
	function pose getpose(string turtlename)
	function string spawn(pose desire_pose)
	function void delete(string turtlename)
	property string{list} turtle_namelist

end object
"""
#Actual class object
class create_impl:
	def __init__(self):               			#initialization upon creation
		#turtlesim initialization
		self.screen = turtle.Screen()
		self.screen.bgcolor("lightblue")
		self.dict={"turtle1":turtle.Turtle()}
		self.dict["turtle1"].shape("turtle")
		#tkthread initialization
		self.tkt = TkThread(self.screen._root)  # make the thread-safe callable
		#RR background thread initialization
		self._lock=threading.RLock()
		self._running=False
		#RR property
		self.pose=RRN.NewStructure("experimental.turtlebot_create.pose")
		self.turtle_namelist=["turtle1"]

	def drive(self,turtlename,move_speed,turn_speed):            #Drive function, update new position, this is the one referred in definition
		self.tkt(self.dict[turtlename].forward,move_speed)
		self.tkt(self.dict[turtlename].left,turn_speed)


	def setpose(self,turtlename,desire_pose):            #set a new pose for turtlebot
		self.tkt(self.dict[turtlename].penup)
		self.tkt(self.dict[turtlename].setpos,desire_pose.x,desire_pose.y)
		self.tkt(self.dict[turtlename].seth,desire_pose.angle)
	def setpose_relative(self,turtlename,desire_pose):   #set a new relative pose for turtlebot
		self.tkt(self.dict[turtlename].penup)
		self.pose=self.getpose(turtlename)
		self.tkt(self.dict[turtlename].setpos,self.pose.x+desire_pose.x,self.pose.y+desire_pose.y)
		self.tkt(self.dict[turtlename].seth,self.pose.angle+desire_pose.angle)

	#return the pose of turtlebot
	def getpose(self,turtlename):
		(self.pose.x,self.pose.y)=self.dict[turtlename].pos()
		self.pose.angle=self.dict[turtlename].heading()
		return self.pose

	#set the pen color of turtlebot
	def setpencolor(self,turtlename,color):
		if color=="none":
			self.tkt(self.dict[turtlename].penup)
			return
		self.tkt(self.dict[turtlename].pendown)
		self.tkt(self.dict[turtlename].pencolor,color)


	#reset the screen
	def reset(self):
		self.tkt(self.screen.reset)
	#spawn a new turtlebot
	def spawn(self,desire_pose):
		name="turtle"+str(len(self.dict)+1)
		self.dict[name]=self.tkt(turtle.Turtle)
		self.tkt(self.dict[name].shape,"turtle")
		self.setpose(name,desire_pose)
		self.turtle_namelist.append(name)
		return name
	#delete one turtlebot
	def delete(self,turtlename):
		self.tkt(self.dict[turtlename].clear)
		del self.dict[turtlename]



if __name__ == '__main__':
		
	with RR.ServerNodeSetup("experimental.turtlebot_create", 22222):      #setup RR node with service name and port
		#Register the service type

		RRN.RegisterServiceType(turtlebot_interface)               #register service type

		create_inst=create_impl()                #create object

		#Register the service with definition and object
		RRN.RegisterService("Turtlebot_Service","experimental.turtlebot_create.turtlesim",create_inst)

		#Wait for program exit to quit
		input("Press enter to quit")