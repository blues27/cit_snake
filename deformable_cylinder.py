import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

ballId = p.loadSoftBody("Cylinder.vtk" , scale = 1.0, mass = 1., useNeoHookean = 0 , useBendingSprings=1, useMassSpring=1, springElasticStiffness=1 , springDampingStiffness=10 , useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.025)


while p.isConnected():

  p.stepSimulation()  
  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  p.setGravity(0,0,-10)
  #sleep(1./240.)
  
#p.resetSimulation()
#p.stopStateLogging(logId)
