import turtle
import time

screen = turtle.Screen()
t1 = turtle.Turtle()
t1.shape("turtle")
screen.bgcolor("lightblue")

t2 = turtle.Turtle()
t2.shape("turtle")

def drive(turtlebot,move_speed,turn_speed):            #Drive function, update new position, this is the one referred in definition

	turtlebot.forward(move_speed)
	turtlebot.left(turn_speed)

def setpose(turtlebot,x,y,angle):            #set a new pose for turtlebot

	turtlebot.setpos(x,y)
	turtlebot.seth(angle)
		



for i in range(50):
	now=time.time()
	drive(t1,5,0)
	print("function call time= ",time.time()-now)

for i in range(50):
	now=time.time()
	drive(t2,-5,0)
	print("function call time= ",time.time()-now)


# t1.clear()
screen.reset()
screen.bgcolor("lightblue")
time.sleep(2)

for i in range(50):
	now=time.time()
	drive(t1,5,0)
	print("function call time= ",time.time()-now)

for i in range(50):
	now=time.time()
	drive(t2,-5,0)
	print("function call time= ",time.time()-now)
