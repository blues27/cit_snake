import pybullet as p
import pybullet_data
import time
import math

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

IND_SIZE = 10

toolbox = base.Toolbox()

toolbox.register("attr_float",
                 random.random)

toolbox.register("individual",
                 tools.initRepeat,
                 creator.Individual,
                 toolbox.attr_float,
                 n=IND_SIZE)

toolbox.register("population",
                 tools.initRepeat,
                 list,
                 toolbox.individual)

#toolbox.register("evaluate", ackley)  // traveled distance 

toolbox.register("mate", tools.cxBlend, alpha=0.2)

toolbox.register("mutate", tools.mutGaussian,
                 mu=[0.0, 0.0], sigma=[200.0, 200.0], indpb=0.2)

toolbox.register("select", tools.selTournament, tournsize=3)


class RobotSim:

    #constant value

    robot_sim_period = 20
    
    gridwidth = 0.18 
    cylinderRadius = 0.02
    cylinderHeight = 0.50
    
    boxHalfLength = 0.05
    boxHalfHeight = 0.2
    boxHalfWidth = 15

    robotId = -1 
    planeId = -1
    physicsClient = -1 

    t_sim = 0 
    dt_sim = 0.01
    
    def make_environment(self):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [self.boxHalfLength, self.boxHalfWidth, self.boxHalfHeight])
        
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

        colCylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.cylinderRadius, height=self.cylinderHeight)
        visualShapeId = -1
        mass = 0
        useMaximalCoordinates = 0
        sphereUid = p.createMultiBody(mass,
                                      colCylinderId,
                                      visualShapeId,
                                      [ 0 , 0 , 0],
                                      useMaximalCoordinates=useMaximalCoordinates)            
    
        newList = [i for i in range(100) if (i+1) * self.gridwidth < (3.0 - self.gridwidth) ]
    
        print(newList)
        for i in newList :
            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [self.gridwidth *  (i + 1) ,  0 , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            

            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [self.gridwidth * -(i + 1) ,  0 , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            

            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [ 0 , self.gridwidth *  (i + 1) , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            

            sphereUid = p.createMultiBody(mass,
                                          colCylinderId,
                                          visualShapeId,
                                          [ 0 , self.gridwidth * -(i + 1) , 0],
                                          useMaximalCoordinates=useMaximalCoordinates)            


            for i in newList :
                for j in newList :
                    sphereUid = p.createMultiBody(mass,
                                                  colCylinderId,
                                                  visualShapeId,
                                                  [self.gridwidth *  (i + 1) , self.gridwidth *  (j + 1) , 0],
                                                  useMaximalCoordinates=useMaximalCoordinates)            

                    sphereUid = p.createMultiBody(mass,
                                                  colCylinderId,
                                                  visualShapeId,
                                                  [self.gridwidth * -(i + 1) , self.gridwidth *  (j + 1) , 0],
                                                  useMaximalCoordinates=useMaximalCoordinates)            
            
                    sphereUid = p.createMultiBody(mass,
                                                  colCylinderId,
                                                  visualShapeId,
                                                  [self.gridwidth *  (i + 1) , self.gridwidth * -(j + 1), 0],
                                                  useMaximalCoordinates=useMaximalCoordinates)            

                    sphereUid = p.createMultiBody(mass,
                                                  colCylinderId,
                                                  visualShapeId,
                                                  [self.gridwidth * -(i + 1) , self.gridwidth * -(j + 1) , 0],
                                                  useMaximalCoordinates=useMaximalCoordinates)            
            
   
    def load_robot(self):
    
        robot_id = p.loadURDF("multisections.urdf", basePosition=[0, 0, 3.0],  useMaximalCoordinates = 0 )
        print(p.getBasePositionAndOrientation(robot_id))

        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 0.0],[0.0, 0.0, 0.0, 1.0])
        print(p.getBasePositionAndOrientation(robot_id))

        anistropicFriction = [1, 0.01, 0.01]
        p.changeDynamics(robot_id, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
    
        for i in range( p.getNumJoints(robot_id) ):
            print(p.getJointInfo(robot_id, i))
            p.changeDynamics(robot_id, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

        return robot_id

    def reset_robot(self,robot_id):
        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 10.0],[0.0, 0.0, 0.0, 1.0])

        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 0.2],[0.0, 0.0, 0.0, 1.0])    

        anistropicFriction = [1, 0.01, 0.01]
        for i in range( p.getNumJoints(robot_id) ):
            print(p.getJointInfo(robot_id, i))
            p.changeDynamics(robot_id, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

            p.setJointMotorControl2(robot_id,
                                    i,
                                    p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=300)
        

        return robot_id

    def __init__(self):
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0, -10)
        p.setRealTimeSimulation(0)

        planeId = p.loadURDF("plane.urdf")
        #make_environment()

        self.robotId = self.load_robot()

        print()

    def one_trial(self):
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

        self.reset_robot(self.robotId)
        
        base_pos,base_ori = p.getBasePositionAndOrientation(self.robotId)
        print(base_pos)
        
        while self.t_sim < 20.0 : #True:

            print("t = " + str(self.t_sim))
            
            keys = p.getKeyboardEvents()
            for k, v in keys.items():

                if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.reset_robot(self.robotId)


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

            jointx = []
            jointy = []

            jointx.clear()
            jointy.clear()

            for i in range(len(section_joints)) :
                joint = section_joints[i] 
                m_velocity = 0.8

                phase =  m_waveAmplitude * math.sin( m_waveFreq * self.t_sim - ((i -4)  *  m_phaseOffset )) - m_steering 

                link_state =  p.getLinkState(self.robotId,joint)
                jointx.append(link_state[0][1])
                jointy.append(link_state[0][0])

                #map phase to curvature
                targetPos =  phase

                #set our motor
                p.setJointMotorControl2(self.robotId,
                                        joint,
                                        p.POSITION_CONTROL,
                                        targetPosition=targetPos,
                                        force=300)


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
            self.t_sim += self.dt_sim
            #time.sleep(dt)
            
            
        base_pos,base_ori = p.getBasePositionAndOrientation(self.robotId)
        print(base_pos)
        
    def end_sim(self):
        p.disconnect()


robotsim = RobotSim()
robotsim.one_trial()



