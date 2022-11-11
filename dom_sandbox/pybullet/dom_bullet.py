# Mostly based on pybullet example https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py 

import pybullet as p
from time import sleep
import pybullet_data

if __name__ == "__main__":
  physicsClient = p.connect(p.GUI)

  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

  gravZ=-10
  p.setGravity(0, 0, gravZ)

  planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
  #planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

  # clothId = p.loadURDF("bullet_official/cloth_z_up.urdf", [0,0,2], flags=p.URDF_USE_SELF_COLLISION)
  boxId = p.loadURDF("franka_panda/panda.urdf", [0,0,0])
  # boxId = p.loadURDF("/Users/nicholasprayogo/code/dom_sandbox/dom_sandbox/pybullet/ur_e_description/urdf/ur5e.urdf.xacro")
  
  clothId = p.loadSoftBody("bullet_official/cloth_z_up.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
  # clothId = p.loadSoftBody("cloth_z_up.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)

  print(f"Box ID: {boxId}")
  print(f"Cloth ID: {boxId}")

  # obj syntax
  # # - comments, which are ignored
  # mtllib - the filename of a materials definition file
  # o - gives the name of a model. All data between this line and the next o line is a single model
  # v - defines the x, y, and z value of a single vertex
  # usemtl - use a specific color and material definition for the following polygons.
  # s - turn smooth shading off or on; flat shading is used when smooth shading is off.
  # f - defines the vertices that compose a face. Note that faces can have more than 3 vertices. In this example the faces have four vertices which define quad polygons. These must be divided into triangles before WebGL rendering.

  # p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

  # anchors cloth to ground
  # arguments:  int softBodyUniqueId, int nodeIndex, int bodyUniqueId, int linkIndex, const double bodyFramePosition[3]
  # can also anchor to box
  # p.createSoftBodyAnchor(clothId ,15,boxId,-1)
  p.createSoftBodyAnchor(clothId ,19,boxId,-1, [-0.5,-0.5,0])
  p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

  p.createSoftBodyAnchor(clothId  ,24,-1,-1)
  p.createSoftBodyAnchor(clothId ,20,-1,-1)
  # info = p.getJointInfo(boxId, 12)
  # print(info)



  # boxid needs to be parent
  # joint type fixed works, gear doesn't
  # p.createConstraint(boxId, 12, clothId, 19,
  #         jointType= p.JOINT_FIXED,
  #        jointAxis=[0, 0, 0],
  #        parentFramePosition=[0, 0, 0],
  #        childFramePosition=[0, 0, 0]
  #        )

  debug = True

  if debug:
    data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    print("--------------")
    print("data=",data)
    print(data[0])
    print(data[1])
    text_uid = []
    for i in range(data[0]):
        pos = data[1][i]
        uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
        text_uid.append(uid)

  while p.isConnected():
    p.getCameraImage(320,200)

    if debug:
      data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
      for i in range(data[0]):
        pos = data[1][i]
        uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1], replaceItemUniqueId=text_uid[i])

    p.setGravity(0,0,gravZ)
    p.stepSimulation()
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    #sleep(1./240.)

  # source: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_torus.py
  #
  # import pybullet as p
  # from time import sleep
  # import pybullet_data
  #
  # physicsClient = p.connect(p.GUI)
  #
  # # p.setAdditionalSearchPath(pybullet_data.getDataPath())
  #
  # # p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
  # # p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])
  # # p.setGravity(0, 0, -10)
  #
  # # tex = p.loadTexture("uvmap.png")
  # # planeId = p.loadURDF("plane.urdf", [0,0,-2])
  #
  # # boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)
  #
  # # cloth was obtained from dedo
  # bunnyId = p.loadSoftBody("data/cloth/shirt_0.obj", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800 )
  # # p.changeVisualShape(bunnyId, -1, rgbaColor=[1,1,1,1],  flags=0)
  #
  # # bunny2 = p.loadURDF("data/torus_deform.urdf", [0,1,0.5], flags=p.URDF_USE_SELF_COLLISION)
  #
  # p.createSoftBodyAnchor(bunnyId, 0,0,0)
  # p.createSoftBodyAnchor(bunnyId, 24, -1, -1)
  # # p.changeVisualShape(bunny2, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)
  # # p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
  # # p.setRealTimeSimulation(0)
  #
  # while p.isConnected():
  #   p.stepSimulation()
  #   # p.getCameraImage(s
