import pybullet as p
import pybullet_data
import time
import math

import numpy as np

class GridClass:
    pointx1 = 0.0
    pointy1 = 0.0
    pointx2 = 0.0
    pointy2 = 0.0

    def __init__(self,x1,y1,x2,y2):
        self.pointx1 = x1
        self.pointy1 = y1
        self.pointx2 = x2
        self.pointy2 = y2        
                 
    def is_withingrid(self,pos):
        ret = 0
        if self.pointx1 < self.pointx2 :
            minx = self.pointx1
            maxx = self.pointx2
        else :
            minx = self.pointx2
            maxx = self.pointx1

        if self.pointy1 < self.pointy2 :
            miny = self.pointy1
            maxy = self.pointy2
        else :
            miny = self.pointy2
            maxy = self.pointy1
            
        if (pos[0] > minx) and (pos[0] < maxx) and (pos[1] > miny) and (pos[1] < maxy):
            ret = 1
            
        return ret
            
class RobotSim:

    #constant value

    robot_sim_period = 300
    
    gridwidth = 0.18 
    cylinderRadius = 0.02
    cylinderHeight = 0.75
    
    boxHalfLength = 0.05
    boxHalfHeight = 0.2
    boxHalfWidth = 2.0

    robotId = -1 
    planeId = -1
    physicsClient = -1 

    t_sim = 0 
    dt_sim = 1.0 / 240.
    
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
    init_pos = []
    init_ori = []
    end_pos = []
    end_ori = []
    newListH = []
    newListV = []

    rewardtable = [] 

    gridtable = []
    
    
    #def calc_array_index(self, col, row):
    #return 
         
    #def init_rewardtable(self):
        
    def make_environment(self):
        
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents = [self.boxHalfLength, self.boxHalfWidth, self.boxHalfHeight])
        
        p.createMultiBody(baseMass = 0,
                          baseCollisionShapeIndex=colBoxId,
                          basePosition=[self.boxHalfWidth, 0, 0.0])
    
        p.createMultiBody(baseMass = 0,
                          baseCollisionShapeIndex=colBoxId,
                          basePosition=[-self.boxHalfWidth, 0, 0.0])
    
        orn = p.getQuaternionFromEuler([0, 0, 1.5707963])
    
        p.createMultiBody(baseMass = 0,
                          baseCollisionShapeIndex=colBoxId,
                          basePosition=[0,self.boxHalfWidth, 0.0],
                          baseOrientation=orn)
    
        p.createMultiBody(baseMass = 0,
                          baseCollisionShapeIndex=colBoxId,
                          basePosition=[0,-self.boxHalfWidth, 0.0],
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
    
        #newList = [i for i in range(100) if (i+1) * self.gridwidth < (self.boxHalfWidth - self.gridwidth) ]
    
        newListV = [i for i in range(100) if (i+1) * self.gridwidth < (self.boxHalfWidth - self.gridwidth) ]
        newListH = [i for i in range(100) if (i+1) * self.gridwidth < (self.boxHalfWidth - self.gridwidth) ]        

        self.newListV  = newListV
        self.newListH  = newListH
        
        print(self.newListV)
        print(self.newListH)
        
        for i in newListH :
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
            
        for i in newListV :
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


        for i in newListH :
            for j in newListV :
                sphereUid = p.createMultiBody(mass,
                                              colCylinderId,
                                              visualShapeId,
                                              [self.gridwidth *  (i + 1) , self.gridwidth *  (j + 1) , 0],
                                              useMaximalCoordinates=useMaximalCoordinates)
                
                point1 = GridClass(self.gridwidth *  (i) , self.gridwidth *  (j), self.gridwidth *  (i + 1) , self.gridwidth *  (j + 1))
                self.gridtable.append(point1)
                    
                sphereUid = p.createMultiBody(mass,
                                              colCylinderId,
                                              visualShapeId,
                                              [self.gridwidth * -(i + 1) , self.gridwidth *  (j + 1) , 0],
                                              useMaximalCoordinates=useMaximalCoordinates)            
                
                point2 = GridClass(self.gridwidth * -(i) , self.gridwidth *  (j), self.gridwidth * -(i + 1) , self.gridwidth *  (j + 1))
                self.gridtable.append(point2)

                sphereUid = p.createMultiBody(mass,
                                              colCylinderId,
                                              visualShapeId,
                                              [self.gridwidth *  (i + 1) , self.gridwidth * -(j + 1), 0],
                                              useMaximalCoordinates=useMaximalCoordinates)            
                
                point3 = GridClass(self.gridwidth *  (i) , self.gridwidth * -(j), self.gridwidth *  (i + 1) , self.gridwidth *  -(j + 1))
                self.gridtable.append(point3)

                sphereUid = p.createMultiBody(mass,
                                              colCylinderId,
                                              visualShapeId,
                                              [self.gridwidth * -(i + 1) , self.gridwidth * -(j + 1) , 0],
                                              useMaximalCoordinates=useMaximalCoordinates)            
                
                point4 = GridClass(self.gridwidth * -(i) , self.gridwidth * -(j), self.gridwidth * -(i + 1) , self.gridwidth *  -(j + 1))
                self.gridtable.append(point4)

        for grid in self.gridtable:
            print( "[(" + str(grid.pointx1) + "," +str(grid.pointy1) + "),(" + str(grid.pointx2) + "," +str(grid.pointy2) + ")]" )
        print(len(self.gridtable))

        
    def load_robot(self):
    
        robot_id = p.loadURDF("multisections.urdf", basePosition=[0, 0, 3.0],  useMaximalCoordinates = 0 )
        #print(p.getBasePositionAndOrientation(robot_id))

        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 0.1],[0.0, 0.0, 0.0, 1.0])
        #print(p.getBasePositionAndOrientation(robot_id))

        anistropicFriction = [1, 0.01, 0.01]
        p.changeDynamics(robot_id, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
    
        for i in range( p.getNumJoints(robot_id) ):
            #print(p.getJointInfo(robot_id, i))
            p.changeDynamics(robot_id, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

        return robot_id

    def reset_robot(self,robot_id):

        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 10.0],[0.0, 0.0, 0.0, 1.0])

        anistropicFriction = [1, 0.01, 0.01]
        p.changeDynamics(robot_id, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
    
        for i in range( p.getNumJoints(robot_id) ):
            #print(p.getJointInfo(robot_id, i))
            p.changeDynamics(robot_id, i, lateralFriction=2, anisotropicFriction=anistropicFriction)
      
            p.resetJointState(robot_id,
                              i,
                              targetValue=0, 
                              targetVelocity=0)
        
        p.resetBasePositionAndOrientation(robot_id,[ self.gridwidth * 0.5, 0.0, 0.1],[0.0, 0.0, 0.0, 1.0])    

    def printparam(self):
        print("m_waveAmplitude = %f"% self.m_waveAmplitude)
        print("m_waveFreq = %f"% self.m_waveFreq)
        print("m_phaseOffset = %f"% self.m_phaseOffset)
        print("m_steering = %f"% self.m_steering)    
        
    def __init__(self):
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0, -10)
        p.setRealTimeSimulation(0)

        planeId = p.loadURDF("plane.urdf")
        self.make_environment()

        self.printparam()        
        self.robotId = self.load_robot()

    def one_trial(self):
        #                  1   2   3   4   5   6   7   8   9  10  11  12
        section_joints = [ 0,  1,  2,  3,  4,  5,  6,  7,  8, 9 ]
        self.t_sim = 0
        
        self.reset_robot(self.robotId)
        
        #print("1:")
        #print(p.getBasePositionAndOrientation(self.robotId))
        for i in range(240) :
            p.stepSimulation()

        #print("2:")
        #print(p.getBasePositionAndOrientation(self.robotId))
        
        self.init_pos,self.init_ori = p.getBasePositionAndOrientation(self.robotId)
        
        print("start: " + str(self.init_pos))
        
        while self.t_sim < self.robot_sim_period : #True:

            #print("t = " + str(self.t_sim))
            
            keys = p.getKeyboardEvents()
            for k, v in keys.items():

                if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.reset_robot(self.robotId)


                if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.m_steering = self.m_steering - 0.01
                    if self.m_steering < self.m_offsetMin: 
                        self.m_steering = self.m_offsetMin

                #    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                #      m_steering = 0

                if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
                    self.m_steering = self.m_steering + 0.01
                    if self.m_steering > self.m_offsetMax: 
                        self.m_steering = self.m_offsetMax

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

                phase =  self.m_waveAmplitude * math.sin( self.m_waveFreq * self.t_sim - ((i -4)  *  self.m_phaseOffset )) - self.m_steering 

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
            pos,ori = p.getBasePositionAndOrientation(self.robotId)
            if pos[2] > 0.1 :
                print( "pos[2] = ",pos[2])
                self.t_sim = self.robot_sim_period
            
        self.end_pos,self.end_ori = p.getBasePositionAndOrientation(self.robotId)
        print("end : " + str(self.end_pos))
        
    def end_sim(self):
        p.disconnect()
       
robotsim = RobotSim()

robotsim.m_waveAmplitude = 4.315718403214737
robotsim.m_waveFreq = 1.9506199911905657
robotsim.m_phaseOffset = 0.6375509005797202
robotsim.m_steering = -0.011831502522241223

robotsim.one_trial()
print("Restart")
robotsim.one_trial()

print("start:")
print(robotsim.init_pos)
print("end:")
print(robotsim.end_pos)


import random 

from deap import algorithms
from deap import base
from deap import creator
from deap import tools 

lastbestgene = [ 4.687669, 4.343982, 2.331904, 0.000000 ] 

def calc_dist(a1,b1):
    p = np.array(a1)
    q = np.array(b1)
    dist = np.linalg.norm(q-p)

    return dist

def calc_eval(individual):
    # phase =  self.m_waveAmplitude * math.sin( self.m_waveFreq * self.t_sim - ((i -4)  *  self.m_phaseOffset )) - self.m_steering 
    robotsim.m_waveAmplitude = individual[0]
    robotsim.m_waveFreq = individual[1]
    robotsim.m_phaseOffset = individual[2]
    robotsim.m_steering = individual[3]

    robotsim.printparam()            
    robotsim.one_trial()

    dist = calc_dist(robotsim.init_pos,robotsim.end_pos)
    if robotsim.end_pos [2] > 0.1 :
        dist = 0.0 

    print("distance = " + str(dist))        
    return dist,

def calc_parameter(num):
    ret_val = 0.0
    #print("num:", num) 
    if num%4 == 0:
        ret_val = math.pi * 2.0 * ( random.uniform(0.0, 360.0) / 360.0 )  # m_waveAmplitude
    elif num%4 == 1:
        ret_val = math.pi * 2.0 / ( random.uniform(1.0, 10.0) )  # m_waveFreq
    elif num%4 == 2: 
        ret_val = math.pi * 2.0 / ( robotsim.m_segmentNumber / random.uniform(1.0, 5.0))
    elif num%4 == 3:
        ret_val = random.uniform(robotsim.m_offsetMin, robotsim.m_offsetMax)
    else :
        ret_val = 0.0
    #print("ret_val:", ret_val)        
    return ret_val 
        
def calc_parameters(container):
    params = []

    for i in range(len(robotsim.gridtable)):
        if random.random() < 0.25 :
            params.append(lastbestgene[0])
            params.append(lastbestgene[1])
            params.append(lastbestgene[2])
            params.append(lastbestgene[3])            
        else:
            params.append(calc_parameter(0))
            params.append(calc_parameter(1))
            params.append(calc_parameter(2))
            params.append(calc_parameter(3))
    
    return container(params)

    
def calc_mutation(individual, indpb):
    for i in range(len(individual)):
        if random.random() < indpb :
            individual[i] = calc_parameter(i)
    return individual, 
        
# optimize: A, w, phi
# angle(n,t) = A * sin(w t + n * phi) 
'''    
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness = creator.FitnessMin)
'''
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness = creator.FitnessMax)

toolbox = base.Toolbox()

toolbox.register("attr_float",
                 random.random)

toolbox.register("individual",
                 calc_parameters,
                 creator.Individual)

toolbox.register("population",
                 tools.initRepeat,
                 list,
                 toolbox.individual)

toolbox.register("evaluate", calc_eval)  # traveled distance 
toolbox.register("mate", tools.cxBlend, alpha=0.2)
toolbox.register("mutate", calc_mutation, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

random.seed(1)

N_GEN = 100
POP_SIZE = 100
CX_PB = 0.5
MUT_PB = 0.2

pop = toolbox.population(n=POP_SIZE)

print("Start of evolution")
    
fitnesses = list(map(toolbox.evaluate, pop))

for ind, fit in zip(pop, fitnesses):
    ind.fitness.values = fit
    
print("  Evaluated %i individuals" % len(pop))

fits = [ind.fitness.values[0] for ind in pop]

g = 0

while g < N_GEN:

    g = g + 1
    print("-- Generation %i --" % g)

    offspring = toolbox.select(pop, len(pop))
    offspring = list(map(toolbox.clone, offspring))

    for child1, child2 in zip(offspring[::2], offspring[1::2]):

        if random.random() < CX_PB:
            toolbox.mate(child1, child2)

            del child1.fitness.values
            del child2.fitness.values

    for mutant in offspring:
        if random.random() < MUT_PB:
            toolbox.mutate(mutant)

            del mutant.fitness.values


    invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
    fitnesses = map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    print("  Evaluated %i individuals" % len(invalid_ind))

    pop[:] = offspring

    fits = [ind.fitness.values[0] for ind in pop]

    
    length = len(pop)
    mean = sum(fits) / length
    sum2 = sum(x*x for x in fits)
    std = abs(sum2 / length - mean**2)**0.5

    print("  Min %s" % min(fits))
    print("  Max %s" % max(fits))
    print("  Avg %s" % mean)
    print("  Std %s" % std)

    print("-- End of (successful) evolution --")

best_ind = tools.selBest(pop, 1)[0]
print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))
