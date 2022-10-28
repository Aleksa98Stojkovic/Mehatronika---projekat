"""my_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor
from time import sleep
import os
import math
import rospy
from std_msgs.msg import String
from math import radians, sqrt
try:
    os.system("rm -r /home/milos/meh_ws/src/camera_filter/nodes/*.txt")
except:
    pass
os.system("python3 /home/milos/meh_ws/src/camera_filter/nodes/camera_filter_node.py &")

start = 0
rospy.init_node('turtlebot3')
rate = rospy.Rate(5)
directions1 = []
directions2 = []

def callback1(data):
    #print(data.data)
    
    a = data.data.split('\n')
    if(data.data != "stop camera node"):
        directions1.append(data.data)
        if(data.data == "end"):
            global start
            start = 1
            print("Directions loaded.")
            
def callback2(data):
    #print(data.data)
    
    a = data.data.split('\n')
    if(data.data != "stop camera node"):
        directions2.append(data.data)
        if(data.data == "end"):
            global start
            start = 1
            print("Directions loaded.")


sub1 = rospy.Subscriber('directions1', String, callback1)
sub2 = rospy.Subscriber('directions2', String, callback2)

MAX_SPEED = 2.84 #max_speed


# create the Robot instance.
robot = Robot()
wheelRadius = 0.033
axleLength = 0.16

angularCorrelation = 325.3/360
linearCorrelation = 2.2397

x_step = 0.064
y_step = 0.058

gripperPos0 = 30
gripperPos0InRad = radians(gripperPos0)
gripperPos1 = 0
gripperPos1InRad = radians(gripperPos1)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print("time step = " + str(timestep) + "ms")

while robot.step(timestep) != -1:
    if(start == 0):
        continue
        
    break

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# initialize devices

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftWheelPositionSensor = robot.getDevice('left wheel sensor')
rightWheelPositionSensor = robot.getDevice('right wheel sensor')

leftWheelPositionSensor.enable(10)
rightWheelPositionSensor.enable(10)

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftGripper = robot.getDevice('left gripper motor')
rightGripper = robot.getDevice('right gripper motor')

rightGripper.setPosition(float(-gripperPos0InRad))
leftGripper.setPosition(float(gripperPos0InRad))

directions2.pop(len(directions2) - 2)
directions2.pop(len(directions2) - 2)
directions2.pop(len(directions2) - 2)
directions2.pop(len(directions2) - 2)
print(directions2)

def setSpeed(speed):
    leftMotor.setVelocity(speed*MAX_SPEED)
    rightMotor.setVelocity(speed*MAX_SPEED)

def positionAngular(left_position_sensor, right_position_sensor):
    l = left_position_sensor.getValue()
    r = right_position_sensor.getValue()
    dl = l * wheelRadius
    dr = r * wheelRadius
    da = (dr - dl) / axleLength * angularCorrelation
    return da*180/math.pi
    
def positionLinear(left_position_sensor, right_position_sensor):
    l = left_position_sensor.getValue()
    r = right_position_sensor.getValue()
    dl = l * wheelRadius
    dr = r * wheelRadius
    da = (dr + dl) / axleLength
    return da*wheelRadius * linearCorrelation
    
def goForward(distance_m):
    stop()
    counter = 0
    setDirection("forward")
    startPosition = abs(positionLinear(leftWheelPositionSensor, rightWheelPositionSensor))
    while robot.step(timestep) != -1:
        if(abs(positionLinear(leftWheelPositionSensor, rightWheelPositionSensor)) - startPosition < distance_m):
            continue
        stop()
        break
        
def goBackward(distance_m):
    stop()
    counter = 0
    setDirection("back")
    startPosition = abs(positionLinear(leftWheelPositionSensor, rightWheelPositionSensor))
    while robot.step(timestep) != -1:
        if(abs(positionLinear(leftWheelPositionSensor, rightWheelPositionSensor)) - startPosition < distance_m):
            continue
        stop()
        break
    
def rotate(angle):
    counter = 0
    #setDirection("forward")
    stop()
    if(angle > 0):
        setDirection("left")
    else:
        setDirection("right")
    startAngle = positionAngular(leftWheelPositionSensor, rightWheelPositionSensor)
    while robot.step(timestep) != -1:
                
        if(abs(positionAngular(leftWheelPositionSensor, rightWheelPositionSensor) - (startAngle)) <= abs(angle)):
            continue
        stop()
        break
    #print("stop_rot")
    while robot.step(timestep) != -1:
        if(counter < 500/timestep): #wait for 500ms
            counter+=1 #mora malo da saceka posto kada menja pravac zanese ga malo
            continue
        break
    
def stop():
    setSpeed(0)

def setDirection(direction):
    if(direction == "fwd" or direction == "forward"):
        leftMotor.setVelocity(-1*MAX_SPEED)
        rightMotor.setVelocity(-1*MAX_SPEED)
    elif(direction == "bck" or direction == "back"):
        leftMotor.setVelocity(1*MAX_SPEED)
        rightMotor.setVelocity(1*MAX_SPEED)
    elif(direction == "l" or direction == "left"):
        leftMotor.setVelocity(-0.2*MAX_SPEED)
        rightMotor.setVelocity(0.2*MAX_SPEED)
    elif(direction == "r" or direction == "right"):
        leftMotor.setVelocity(0.2*MAX_SPEED)
        rightMotor.setVelocity(-0.2*MAX_SPEED)
        
def execDirections1(directions1):
    i = 0
    prevState = directions1[0]
    curState = directions1[0]
    
    while(directions1[i] != "end"):
    
        if curState == "start":   
            pass
        elif curState == "gore":
            if prevState == "start":   
                goForward(x_step)
            elif prevState == "gore":
                goForward(x_step)
                pass
            elif prevState == "dole":
                rotate(180)
                goForward(x_step)
                pass
            elif prevState == "levo":
                rotate(-90)
                goForward(x_step)
                pass
            elif prevState == "desno":
                rotate(90)
                goForward(x_step)
                pass
        elif curState == "dole":
            if prevState == "start":
                rotate(180)
                goForward(x_step)
            elif prevState == "gore":
                rotate(180)
                goForward(x_step)
                pass
            elif prevState == "dole":
                goForward(x_step)
                pass
            elif prevState == "levo":
                rotate(90)
                goForward(x_step)
                pass
            elif prevState == "desno":
                rotate(-90)
                goForward(x_step)
                pass
        elif curState == "levo":
            if prevState == "start":
                rotate(90)
                goForward(y_step)
            elif prevState == "gore":
                rotate(90)
                goForward(y_step)
                pass
            elif prevState == "dole":
                rotate(-90)
                goForward(y_step)
                pass
            elif prevState == "levo":
                goForward(y_step)
                pass
            elif prevState == "desno":
                rotate(180)
                goForward(y_step)
                pass
        elif curState == "desno":
            if prevState == "start":
                rotate(-90)
                goForward(y_step)
            elif prevState == "gore":
                rotate(-90)
                goForward(y_step)
                pass
            elif prevState == "dole":
                rotate(90)
                goForward(y_step)
                pass
            elif prevState == "levo":
                rotate(180)
                goForward(y_step)
                pass
            elif prevState == "desno":
                goForward(y_step)
                pass
    
            
        prevState = curState
        curState = directions1[i+1]
        i+=1
        
        if(prevState != curState):
            stop()
            counter = 0
            while robot.step(timestep) != -1:
                if(counter < 500/timestep):
                    counter+=1 
                    continue
                stop()
                break
                
    leftGripper.setPosition(float(gripperPos1InRad))
    rightGripper.setPosition(float(gripperPos1InRad))
    
def execDirections2(directions2):
    i = 0
    prevState = directions1[-2]
    curState = directions1[-2]
    
    while(directions2[i] != "end"):
    
        if curState == "start":   
            pass
        elif curState == "gore":
            #proveri sva prev stanja
            if prevState == "start":   
                goForward(x_step)
            elif prevState == "gore":
                goForward(x_step)
                pass
            elif prevState == "dole":
                #proveri sva prev stanja
                rotate(180)
                goForward(x_step)
                pass
            elif prevState == "levo":
                #proveri sva prev stanja
                rotate(-90)
                goForward(x_step)
                pass
            elif prevState == "desno":
                rotate(135)
                goForward(x_step)
                pass
        elif curState == "dole":
            if prevState == "start":
                rotate(180)
                goForward(x_step)
            elif prevState == "gore":
                rotate(180)
                goForward(x_step)
                pass
            elif prevState == "dole":
                #proveri sva prev stanja
                goForward(x_step)
                pass
            elif prevState == "levo":
                #proveri sva prev stanja
                rotate(90)
                goForward(x_step)
                pass
            elif prevState == "desno":
                rotate(-135)
                goForward(x_step)
                pass
        elif curState == "levo":
            if prevState == "start":
                rotate(90)
                goForward(y_step)
            elif prevState == "gore":
                rotate(140)
                goForward(y_step)
                pass
            elif prevState == "dole":
                #proveri sva prev stanja
                rotate(-135)
                goForward(y_step)
                pass
            elif prevState == "levo":
                #proveri sva prev stanja
                #rotate(90)
                goForward(y_step)
                pass
            elif prevState == "desno":
                rotate(180)
                goForward(y_step)
                pass
        elif curState == "desno":
            if prevState == "start":
                rotate(-90)
                goForward(y_step)
            elif prevState == "gore":
                rotate(-135)
                goForward(y_step)
                pass
            elif prevState == "dole":
                #proveri sva prev stanja
                rotate(90)
                goForward(y_step)
                pass
            elif prevState == "levo":
                #proveri sva prev stanja
                rotate(180)
                goForward(y_step)
                pass
            elif prevState == "desno":
                goForward(y_step)
                pass
    
            
        prevState = curState
        curState = directions2[i+1]
        i+=1
        
        if(prevState != curState):
            stop()
            counter = 0
            #setDirection("forward")
            #startPosition = abs(positionLinear(leftWheelPositionSensor, rightWheelPositionSensor))
            while robot.step(timestep) != -1:
                if(counter < 500/timestep): #wait for 500ms
                    counter+=1 #mora malo da saceka posto kada menja pravac zanese ga malo
                    continue
                #print("stop")
                stop()
                break
                
    rightGripper.setPosition(float(-gripperPos0InRad))
    leftGripper.setPosition(float(gripperPos0InRad))
    
           
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    execDirections1(directions1)
    
    execDirections2(directions2)

    break
    pass
    

# Enter here exit cleanup code.
