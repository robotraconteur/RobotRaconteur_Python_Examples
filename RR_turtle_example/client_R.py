from RobotRaconteur.Client import *		#import RR client library

url='rr+tcp://localhost:22222/?service=Turtlebot_Service'
obj=RRN.ConnectService(url)
#following not working yet
# pose=RRN.GetStructureType("experimental.turtlebot_create.pose",obj)
pose=obj.getpose("turtle1")
i=0

def drawR(turtlename,obj):
	obj.drive(turtlename,200,0)
	obj.drive(turtlename,0,-90)
	for i in range(21):
		obj.drive(turtlename,9,-9)
	obj.drive(turtlename,0,140)
	obj.drive(turtlename,120,0)

while True:
	pose.x=0
	pose.y=0
	pose.angle=90
	obj.setpose("turtle1",pose)
	obj.setpencolor("turtle1","red")
	drawR("turtle1",obj)
	pose.x=30
	pose.y=0
	obj.setpose("turtle1",pose)
	obj.setpencolor("turtle1","green")
	drawR("turtle1",obj)
	# turtlename=obj.spawn(pose)
	# obj.setpencolor(turtlename,"green")
	# drawR(turtlename,obj)
	obj.reset()
	# obj.delete(turtlename)

