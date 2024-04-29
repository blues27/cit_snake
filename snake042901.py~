import pybullet as p
import time
import math

import pybullet_data

import random 

import numpy as np

from deap import algorithms
from deap import base
from deap import creator
from deap import tools 

# optimize: A, w, phi
# angle(n,t) = A * sin(w t + n * phi) 
'''    
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness = creator.FitnessMin)
'''
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness = creator.FitnessMax)


'''
IND_SIZE = 10

toolbox = base.Toolbox()
toolbox.register("attribute",
                 random.random)

toolbox.register("individual",
                 tools.initRepeat,
                 creator.Individual,
                 toolbox.attribute,
                 n=IND_SIZE)

toolbox.register("population",
                 tools.initRepeat,
                 list,
                 toolbox.individual)
'''


gridwidth = 0.18 
cylinderRadius = 0.02
cylinderHeight = 0.50

boxHalfLength = 0.05
boxHalfHeight = 0.2
boxHalfWidth = 15

def make_environment():
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [boxHalfLength, boxHalfWidth, boxHalfHeight])
    
    p.createMultiBody(baseMass = 0,
                      baseCollisionShapeIndex=colBoxId,
                      basePosition=[15, 0, 0.0])
    
    p.createMultiBody(baseMass = 0,
                      baseCollisionShapeIndex=colBoxId,
                      basePosition=[-15, 0, 0.0])
    
    orn = p.getQuaternionFromEuler([0, 0, 1.5707963])
    
    p.createMultiBody(baseMass = 0,
                      baseCollisionShapeIndex=colBoxId,
                      basePosition=[0,15, 0.0],
                      baseOrientation=orn)
    
    p.createMultiBody(baseMass = 0,
                      baseCollisionShapeIndex=colBoxId,
                      basePosition=[0,-15, 0.0],
                      baseOrientation=orn)

    colCylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=cylinderRadius, height=cylinderHeight)
    visualShapeId = -1
    mass = 0
    useMaximalCoordinates = 0
    sphereUid = p.createMultiBody(mass,
                                  colCylinderId,
                                  visualShapeId,
                                  [ 0 , 0 , 0],
                                  useMaximalCoordinates=useMaximalCoordinates)            
    
    newList = [i for i in range(100) if (i+1) * gridwidth < (3.0 - gridwidth) ]
    
    print(newList)
    for i in newList :
        sphereUid = p.createMultiBody(mass,
                                      colCylinderId,
                                      visualShapeId,
                                      [gridwidth *  (i + 1) ,  0 , 0],
                                      useMaximalCoordinates=useMaximalCoordinates)            

        sphereUid = p.createMultiBody(mass,
                                      colCylinderId,
                                      visualShapeId,
                                      [gridwidth * -(i + 1) ,  0 , 0],
                                      useMaximalCoordinates=useMaximalCoordinates)            

        sphereUid = p.createMultiBody(mass,
                                      colCylinderId,
                                      visualShapeId,
                                      [ 0 , gridwidth *  (i + 1) , 0],
                                      useMaximalCoordinates=useMaximalCoordinates)            

        sphereUid = p.createMultiBody(mass,
                                      colCylinderId,
                                      visualShapeId,
                                      [ 0 , gridwidth * -(i + 1) , 0],
                                      useMaximalCoordinates=useMaximalCoordinates)            


    for i in newList :
        for j in newList :
            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [gridwidth *  (i + 1) , gridwidth *  (j + 1) , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            

            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [gridwidth * -(i + 1) , gridwidth *  (j + 1) , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            
    
            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [gridwidth *  (i + 1) , gridwidth * -(j + 1), 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            

            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [gridwidth * -(i + 1) , gridwidth * -(j + 1) , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            
    
   
def load_robot():
    
    robotId = p.loadURDF("multisections.urdf", basePosition=[0, 0, 3.0],  useMaximalCoordinates = 0 )

    print(p.getBasePositionAndOrientation(robotId))
    p.resetBasePositionAndOrientation(robotId,[ gridwidth * 0.5, 0.0, 0.0],[0.0, 0.0, 0.0, 1.0])
    print(p.getBasePositionAndOrientation(robotId))
    anistropicFriction = [1, 0.01, 0.01]
    p.changeDynamics(robotId, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
    
    for i in range( p.getNumJoints(robotId) ):
        print(p.getJointInfo(robotId, i))
        p.changeDynamics(robotId, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

    return robotId

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0, -10)
p.setRealTimeSimulation(0)

planeId = p.loadURDF("plane.urdf")
make_environment()

robotId = load_robot()

print()

#                  1   2   3   4   5   6   7   8   9  10  11  12
section_joints = [ 0,  1,  2,  3,  4,  5,  6,  7,  8, 9 ]

m_curveNumber = 2
m_segmentNumber = 11
m_segmentLength = 0.142

m_waveLength = m_segmentLength * m_segmentNumber 

m_phaseOffset = math.pi * 2.0 / (m_segmentNumber / m_curveNumber )
m_wavePeriod = 1.0
m_waveFreq  = math.pi * 2.0 / m_wavePeriod 

m_waveAmplitude = math.pi * 2.0 * ( 10.0 / 360.0 )

m_offsetMin = -math.pi * (20.0 / 180.0)
m_offsetMax =  math.pi * (20.0 / 180.0)

#our steering value
m_steering = 0.0

t = 0 
dt = 0.01

while True:
    keys = p.getKeyboardEvents()
    for k, v in keys.items():

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            m_steering = m_steering - 0.01
            if m_steering < m_offsetMin: 
                m_steering = m_offsetMin
                
        #    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
        #      m_steering = 0

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
            m_steering = m_steering + 0.01
            if m_steering > m_offsetMax: 
                m_steering = m_offsetMax

        #if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
        #      m_steering = 0
        ##


    ##  angle(n,t) = A * sin(w t + n * phi) 
    
    base_pos,base_ori = p.getBasePositionAndOrientation(robotId)        
    #print(base_pos)
    
    
    jointx = []
    jointy = []

    jointx.clear()
    jointy.clear()

    for i in range(len(section_joints)) :
        joint = section_joints[i] 
        m_velocity = 0.8
        #phase =  m_waveAmplitude * math.sin( math.pi / 2.0 * (m_velocity * t - m_segmentLength * i) / ( m_waveLength / m_curveNumber / 4.0 ) )  - m_steering 

        phase =  m_waveAmplitude * math.sin( m_waveFreq * t - ((i -4)  *  m_phaseOffset )) - m_steering 
        
        #link_state =  p.getLinkState(robotId,i)
        #jointx.append(link_state[0][0])
        #jointy.append(link_state[0][1]) 

        link_state =  p.getLinkState(robotId,joint)
        
        jointx.append(link_state[0][1])
        jointy.append(link_state[0][0])
       
        if i >= 1 : 
            d_linkstart =  p.getLinkState(robotId,i-1)
            d_linkstop  =  p.getLinkState(robotId,i)
            #p.addUserDebugLine([d_linkstart[0][0], d_linkstart[0][1], 0.2], [d_linkstop[0][0], d_linkstop[0][1], 0.2], [1, 0, 0])
            #print( [d_linkstart[0][0], d_linkstart[0][1], 0.2])
            #print( [d_linkstop[0][0], d_linkstop[0][1], 0.2])            
        #print(p.getJointInfo(robotId, i))
        
        #map phase to curvature
        targetPos =  phase
        
##        if joint == 1 :
##            print("target Pos = " + str(phase * 180.0 / math.pi) + "  deg. "  )  

        #set our motor
        p.setJointMotorControl2(robotId,
                                joint,
                                p.POSITION_CONTROL,
                                targetPosition=targetPos,
                                force=300)

    #print()
    #print(jointy)

    x = np.array(jointx)
    y = np.array(jointy)    

    a = ((x * y).mean() - (x.mean() * y.mean())) / ((x ** 2).mean() - x.mean() ** 2)
    b = -(a * x.mean()) + y.mean()    

    #a,b = np.linalg.solve(x,y)

    k = np.polyfit(jointx, jointy , 1)

    #print( str(k) + " : " + str(a) + " , " + str(b) )
    #print(str(jointx) + " , " + str(jointy) + " , " + str(k) )
    
    p.addUserDebugLine([ k[0] * jointx[0] + k[1], jointx[0], 0.2], [ k[0] * jointx[9] + k[1], jointx[9],0.2], [1, 0, 0],lifeTime = 0.5 )    
    
    p.stepSimulation()
    t += dt
    time.sleep(dt)
        

p.disconnect()

