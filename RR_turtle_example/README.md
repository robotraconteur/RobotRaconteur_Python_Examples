# RobotRaconteur turtle module example
Honglu He
![](images/turtle.gif)

## Prerequisite:
### python3.7
### tkthread:
`pip install tkthread`
### tcl-thread:
#### Ubuntu:
`sudo apt-get install tcl-thread`
#### Windows:
Move folder *thread2.8.4* to *C:\<YourPythonInstallationDir>\tcl\tcl8.6*
### pygame:
`pip install pygame`

## Usages:
### Start RR Service:
`python turtlebot_service.py`
### Start RR Client:
#### Draw an 'R': 
`python client_R.py`
#### Keyboard Control: 
`python turtlebot_keyboard.py`

## Service Definition:
```
struct pose                   #pose structure
    field single x
    field single y
    field single angle
end
object turtlesim
  	#drive the turtle
  function void drive(string turtlename,single move_speed, single turn_speed)
	#teleport the turtle in absolute coordinates
  function void setpose(string turtlename,pose desire_pose)
	#teleport the turtle in relative coordinates
  function void setpose_relative(string turtlename,pose desire_pose)
	#set the pencolor of the turtle, color "none" for transparent
  function void setpencolor(string turtlename,string color)
	#reset drawings on board
  function void reset()
	#get the pose of the turtle
  function pose getpose(string turtlename)
```


