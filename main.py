#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


#IMPORTANT! OUR CODE USES NEGATIVE VALUES ON THE WHEELS TO MOVE FORWARS AS WE USED GEARS
def stop():
    left.hold()
    right.hold()
    return


def grab(turn):
    turn = turn + 1
    # make turn in second block
    if turn == 2:
        stop()
        ev3.speaker.beep(500, 2000)
        right.run(300)
        left.run(300)
        wait(400)
        stop()
        right.run(-300)
        left.run(300)
        wait(1750)
        stop()
        return turn
    # make beep 
    stop()
    ev3.speaker.beep(500, 2000)
    #correct direction
    right.run(-300)
    left.run(300)
    wait(80)
    # move to object 
    right.run(-300)
    left.run(-300)
    wait(1000)
    stop()
    #close the claw
    claw.run_time(-500, 2600)
    # turn 
    right.run(-300)
    left.run(300)
    wait(1250)
    stop()
    #move forward
    right.run(-300)
    left.run(-300)
    wait(1500)
    stop()
    #open claw
    claw.run_time(500, 2000)
    #move backwards
    right.run(300)
    left.run(300)
    wait(1500)
    stop()
    #turn to orignal position
    right.run(300)
    left.run(-300)
    wait(1270)
    stop() 
    #update number of objecets
    return turn


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
wheelDiameter = 40
axlTrack=105
ev3 = EV3Brick()
light = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S2)
left = Motor(Port.B)
right = Motor(Port.C)
claw = Motor(Port.D)
bot = DriveBase(left,right,wheelDiameter,axlTrack)
turn=0

# Write your program here.
target = 15
lpk = 40
lpk2 = 100
lpk3 = 0.0009
prev = 0
integral = 0

while True: 
    if ultrasonic_sensor.distance(False) <= 120:
        turn = grab(turn)
        continue

    error = (light.reflection()-target)
    integral = integral + error
    deriv = prev - error
    correction = error*lpk + deriv*lpk2 + integral*(lpk3)
    prev = error
    left.run(-150+correction)
    right.run(-150-correction)

   