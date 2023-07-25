"""Tail_controller_1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard, Supervisor
import csv
import numpy as np
# create the Robot instance.

keyboard = Keyboard()
robot = Supervisor()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
gpsTip = robot.getDevice("gpsTip")
gpsClub = robot.getDevice("gpsClub")
gyro = robot.getDevice("gyro")
#parameters
#delay between tail blow 
m0 = robot.getDevice("M0") 
m1 = robot.getDevice("M1")
m2 = robot.getDevice("M2")
m3 = robot.getDevice("M3")
m4 = robot.getDevice("M4")
m5 = robot.getDevice("M5")
m6 = robot.getDevice("M6")
p0 = robot.getDevice("p0") 
p1 = robot.getDevice("p1")
p2 = robot.getDevice("p2")
p3 = robot.getDevice("p3")
p4 = robot.getDevice("p4")
p5 = robot.getDevice("p5")
p6 = robot.getDevice("p6")
mbody = robot.getDevice("Mbody")

p0.enable(timestep)
p1.enable(timestep)
p2.enable(timestep)
p3.enable(timestep)
p4.enable(timestep)
p5.enable(timestep)
p6.enable(timestep)
#mbody.enableTorqueFeedback(timestep)
m0.enableTorqueFeedback(timestep)
m1.enableTorqueFeedback(timestep)
m2.enableTorqueFeedback(timestep)
m3.enableTorqueFeedback(timestep)
m4.enableTorqueFeedback(timestep)
m5.enableTorqueFeedback(timestep)
m6.enableTorqueFeedback(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
gpsTip.enable(timestep)
gpsClub.enable(timestep)
gyro.enable(timestep)

m0.setPosition(float('inf'))
m1.setPosition(float('inf'))
m2.setPosition(float('inf'))
m3.setPosition(float('inf'))
m4.setPosition(float('inf'))
m5.setPosition(float('inf'))
m6.setPosition(float('inf'))

def rightBlow():
    m0.setTorque(1600)
    m1.setTorque(1200)
    m2.setTorque(836)
    m3.setTorque(581)
    m4.setTorque(403)
    m5.setTorque(280)
    m6.setTorque(195)

def leftBlow(factor):
    m0.setTorque(-1600*factor)
    m1.setTorque(-1200*factor)
    m2.setTorque(-836*factor)
    m3.setTorque(-581*factor)
    m4.setTorque(-403*factor)
    m5.setTorque(-280*factor)
    m6.setTorque(-195*factor)
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
def instance(body_velocity =5, body_rotation_deg = 30, blow_timer_s = 0.4, body_rot_offset_s = 0, factor = 1):
    timer = 0
    phase = 0
    maxSpeedClub = 1
    maxSpeedTip = 1
    body_offset = body_rot_offset_s*1000/timestep
    blow_timer = blow_timer_s*1000/timestep
    body_rotation = body_rotation_deg*0.0174533
    mbody.setVelocity(body_velocity)
    threshold = 1
    ang_speed = []
    velocity = []
    E = 0
    E_tot = 0
    prev_E = 0
    pos0 = []; pos1 = []; pos2 = []; pos3=[];pos4=[];pos5=[];pos6=[]
    while timer < 400:
        #E_body += mbody.getTorqueFeedback()*mbody.getVelocity()*timestep
        ang_speed.append(gyro.getValues()[2])
        timer +=1
        robot.step(timestep)
        if timer < blow_timer and phase == 0:
            leftBlow(factor)
            mbody.setPosition(-body_rotation/2)
            mbody.setVelocity(body_velocity)
            phase +=1
            
        elif timer > blow_timer and phase < 3:
            if body_offset > 0:
                if phase == 1:
                    rightBlow()
                    phase +=1
                    
                elif timer > blow_timer + body_offset and phase ==2:
                    mbody.setPosition(body_rotation/2)
                    mbody.setVelocity(body_velocity)
                    phase +=1
                    
            elif body_offset < 0:
                if phase ==1:
                    mbody.setPosition(body_rotation/2)
                    mbody.setVelocity(body_velocity)
                    phase +=1
                    
                elif timer > blow_timer - body_offset and phase ==2:
                    rightBlow()
                    phase +=1
                    
            elif body_offset == 0:
                mbody.setPosition(body_rotation/2)
                mbody.setVelocity(body_velocity)
                rightBlow()
                phase +=2
        velocity.append(gpsTip.getSpeed()*np.sign(gpsTip.getValues()[1]))
        speedTip = gpsTip.getSpeed()
        speedClub = gpsClub.getSpeed()
        if speedTip> maxSpeedTip:
            maxSpeedTip = speedTip
        if speedClub> maxSpeedClub:
            maxSpeedClub = speedClub
        pos0.append(p0.getValue())
        pos1.append(p1.getValue())
        pos2.append(p2.getValue())
        pos3.append(p3.getValue())
        pos4.append(p4.getValue())
        pos5.append(p5.getValue())
        pos6.append(p6.getValue())
        prev_E = E
        E = 1/2*320*((gyro.getValues()[2])**2)
        if E-prev_E >0:
            E_tot += E-prev_E
        pass
    #return (maxSpeedTip, E_body, E_tail)#maxSpeedClub
    E_t = (max(pos0)-factor*min(pos0))*1600 + (max(pos1)-factor*min(pos1))*1200 + (max(pos2)-factor*min(pos2))*836 + (max(pos3)-factor*min(pos3))*581\
    +(max(pos4)-factor*min(pos4))*403+(max(pos5)-factor*min(pos5))*280+(max(pos6)-factor*min(pos6))*195
    
    return maxSpeedTip
    

timer = 0
phase = 0
body_velocity =5
body_rotation_deg = 30 
blow_timer_s = 0.8
body_rot_offset_s = 0
E_t = []
E_b = []
print(instance())


