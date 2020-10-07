from vpython import *
#GlowScript 2.7 VPython
from visual import *
from visual.graph import *

#loopRate = 50

springTheta = prompt("Enter the angle above the horizontal the spring makes with the ground:")
springMaxCompression = prompt("Enter the amount of compression (0 < x < 3):")
springTheta = radians(springTheta)

#springTheta = radians(30)
#springCompression = 1

springConstant = 200
springUncompressedLength = 3

bounceEnergyConstantLost = 5
bounceEnergyPercentLost = .2
energyLostSlidingPerSecond = 10

groundY = 0;

ground = box(pos=vec(0, groundY-0.5-0.5, 0), size=vec(50, 1, 1), color=color.green)

spring = helix(pos=vec(-25, groundY, 0), size=vec(springUncompressedLength, 0.5, 0.5), color=color.orange)
spring.compression = 0

object = sphere(radius = 0.5, color=color.blue)
object.mass = 5
object.velocity = vec(0,0,0)
object.force = vec(0,0,0)

dragCoefficient = 0.8

gravity = vec(0, -9.8, 0)

t = 0
dt = 0.02

spring.axis = vec(cos(springTheta), sin(springTheta), 0)
spring.coils = 10

#=============FUNCTION DEFINITIONS================#
    
def applyForce(target, force):
    target.force += force

def updatePosition(target):
    acceleration = target.force / target.mass                           
    initialVelocity = target.velocity
    target.pos = target.pos + initialVelocity * dt + .5 * acceleration * dt * dt
    target.velocity = initialVelocity + acceleration * dt
    
def restoringForceOnObject(): #positive, hopefully
    return springConstant * spring.compression * norm(spring.axis)

def updateSpringCompression():
    if object.pos.x > spring.pos.x + springUncompressedLength * cos(springTheta):
        spring.compression = 0
    else:
        spring.compression = max(0, springUncompressedLength - comp(object.pos - spring.pos, spring.axis))
    spring.size.x = springUncompressedLength - spring.compression
    
def kineticEnergy(object):
    return .5 * object.mass * mag2(object.velocity)
    
def objectTouchedGround():
    objectKEBefore = kineticEnergy(object)
    if (object.velocity.y != 0):
        objectKEAfter = max(objectKEBefore * (1 - bounceEnergyPercentLost) - bounceEnergyConstantLost, 0)
        speedAfterSquared = objectKEAfter * 2 / object.mass  # KE equation

        # |v|^2 = Vx^2 + Vy^2
        # make sure x and z momentum is conserved because energy lost only affects y velocity
        
        velYAfterSquared = max(speedAfterSquared - object.velocity.x**2 - object.velocity.z**2, 0)
        
        # bounces if object still have vertical velocity after collision
        if velYAfterSquared > 0:
            global numberOfBounces
            numberOfBounces += 1
            
        object.velocity = vec(object.velocity.x, sqrt(velYAfterSquared), object.velocity.z)
    else:
        objectKEAfter = max(objectKEBefore - energyLostSlidingPerSecond * dt, 0)
        speedRemainsRatio = sqrt(objectKEAfter / objectKEBefore) # vb4 / vafter = sqrt(KEb4 / KEafter)
        
        object.velocity = object.velocity * speedRemainsRatio


    
#=============SIMULATION LOGIC====================#

#object.pos = spring.pos + spring.axis * (springUncompressedLength - spring.compression)
#spring.size.x = (springUncompressedLength - spring.compression)

shouldRun = True

numberOfBounces = 0

gdisplay(x=100, y=500, xtitle='time (sec)', ytitle='P.E. (cyan), K.E. (red), Mechanical Energy (yellow)')
keCurve = gcurve(color=color.red)
uCurve = gcurve(color=color.cyan)
totCurve = gcurve(color=color.yellow)

# Spring winding
while spring.compression < springMaxCompression:
    compressSpeed = 2
    spring.compression = min(springMaxCompression, spring.compression + compressSpeed * dt)
    spring.size.x = springUncompressedLength - spring.compression
    object.pos = spring.pos + spring.axis       #spring.axis actually accounts for its own length
    rate(50)
    
objectInitialX = object.pos.x

spring.compression = springMaxCompression
attach_trail(object)

#scene.camera.follow(object)
#scene.autoscale = False
#scene.range = 5

while shouldRun:
    object.force = vec(0,0,0)
    restoringForce = restoringForceOnObject()
    applyForce(object, restoringForce)                          #force of the spring, should be zero when ball leaves the spring
    applyForce(object, -dragCoefficient * object.velocity)      #drag force
    
    if object.pos.y > groundY:
        applyForce(object, gravity * object.mass)                   # force of gravity only when object is above ground                                                     

    updatePosition(object)
    updateSpringCompression()    
    
    if object.pos.y <= groundY:     # object touches ground
        object.pos.y = 0
        objectTouchedGround()
            
    if kineticEnergy(object) <= 0:
        shouldRun = False
            
    potentialEnergy = object.mass * object.pos.y * -gravity.y
    kinEnergy = kineticEnergy(object)
    
    keCurve.plot(pos=(t, kinEnergy))
    uCurve.plot(pos=(t, potentialEnergy))
    totCurve.plot(pos=(t, kinEnergy + potentialEnergy))
    t = t + dt
    rate(50)

print("Simulation complete!!!")
print("Time object traveled: ",t, " s")
print("Horizontal distance traveled: ",object.pos.x - objectInitialX, " m")
print("Object bounces ", numberOfBounces, " time(s)")