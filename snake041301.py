import pybullet as p
import time
import math

import pybullet_data


def create_wall(ewns):
    boxHalfLength = 0.05
    boxHalfWidth = 0.5
    boxHalfHeight = 0.2

    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [boxHalfLength, boxHalfWidth, boxHalfHeight])
    mass = 1
    visualShapeID = -1
    segmentStart = 0
    segmentLength = 5
    
    for i in range(segmentLength):
        p.createMultiBody(baseMass = 0,
                          baseCollisionShapeIndex=colBoxId,
                          basePosition=[segmentStart, 0, -0.2])
        segmentStart = segmentStart - 1

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.setRealTimeSimulation(0)

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
'''
robotId = p.loadURDF("multisections.urdf",cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
'''

create_wall(1)


robotId = p.loadURDF("multisections.urdf")

anistropicFriction = [1, 0.01, 0.01]
p.changeDynamics(robotId, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)

for i in range( p.getNumJoints(robotId) ):
    print(p.getJointInfo(robotId, i))
    p.changeDynamics(robotId, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

print()

#                  1   2   3   4   5   6   7   8   9  10  11  12
section_joints = [ 0,  1,  2,  3,  4,  5,  6,  7,  8, 9 ]

SNAKE_NORMAL_PERIOD = 0.1  #1.5

m_curveNumber = 2
m_segmentNumber = 11
m_segmentLength = 0.142

m_waveLength = m_segmentLength * m_segmentNumber  # 6

m_phaseOffset = math.pi * 2.0 / m_segmentNumber
m_wavePeriod = 2.0
m_waveFreq  = math.pi * 2.0 / m_wavePeriod 

m_waveAmplitude = math.pi * 2.0 * ( 20.0 / 360.0 )

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
        ##

    base_pos,base_ori = p.getBasePositionAndOrientation(robotId)        
    #print(base_pos)
    
    for i in range(len(section_joints)) :
        joint = section_joints[i] 
        m_velocity = 0.8
        phase =  m_waveAmplitude * math.sin( math.pi / 2.0 * (m_velocity * t - m_segmentLength * i) / ( m_waveLength / m_curveNumber / 4.0 ) )  - m_steering 

        #phase =  m_waveAmplitude * math.sin( m_waveFreq * t - ((i -4)  *  m_phaseOffset )) - m_steering 

        
        if i >= 1 : 
            d_linkstart =  p.getLinkState(robotId,i-1)
            d_linkstop  =  p.getLinkState(robotId,i)
            #p.addUserDebugLine([d_linkstart[0][0], d_linkstart[0][1], 0.2], [d_linkstop[0][0], d_linkstop[0][1], 0.2], [1, 0, 0])
            #print( [d_linkstart[0][0], d_linkstart[0][1], 0.2])
            #print( [d_linkstop[0][0], d_linkstop[0][1], 0.2])            
        #print(p.getJointInfo(robotId, i))
        
        #map phase to curvature
        targetPos = phase
        
##        if joint == 1 :
##            print("target Pos = " + str(phase * 180.0 / math.pi) + "  deg. "  )  

        #set our motor
        p.setJointMotorControl2(robotId,
                                joint,
                                p.POSITION_CONTROL,
                                targetPosition=targetPos,
                                force=300)

    #print()
    p.stepSimulation()
    t += dt
    time.sleep(dt)
        

p.disconnect()

