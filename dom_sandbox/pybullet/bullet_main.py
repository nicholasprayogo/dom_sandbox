

import pybullet as p
import time 
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--manipulator_xml_path', type=str, required=False, default = "data/franka_panda/panda.urdf")
    parser.add_argument('--cloth_xml_path', type=str, required=False, default= "data/bullet_official/cloth_z_up.obj")
    
    args = parser.parse_args()
    manipulator_xml_path = args.manipulator_xml_path
    cloth_xml_path = args.cloth_xml_path
    
    physicsClient = p.connect(p.GUI)
#   p.setAdditionalSearchPath(pd.getDataPath())
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    # RESET_USE_SIMPLE_BROADPHASE
    # RESET_USE_DEFORMABLE_WORLD
    # RESET_USE_DISCRETE_DYNAMICS_WORLD

#   boxId = p.loadURDF("/Users/nicholasprayogo/code/dom_sandbox/dom_sandbox/pybullet/ur_e_description/urdf/ur5e.urdf.xacro")
#   boxId = p.loadURDF("franka_panda/panda.urdf", [0,0,0])
#   robot_id = p.loadMJCF("data/franka_emika_panda/panda.xml")
#   robot_id = p.loadMJCF("data/mjcf_ur5e/ur5e.xml")
#   boxId = p.loadURDF("data/ur_description/urdf/ur10_robot.urdf")
    
    gravZ = -10
    p.setGravity(0, 0, gravZ)
  
    boxId = p.loadURDF(manipulator_xml_path, [0,0,0])
    clothId = p.loadSoftBody(, basePosition = [0, 0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
    
    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    
    # anchor nodes 20 and 24 to ground
    p.createSoftBodyAnchor(clothId ,20,-1,-1)
    p.createSoftBodyAnchor(clothId ,24,-1,-1)
    # p.createSoftBodyAnchor(clothId ,10,-1,-1)
    # p.createSoftBodyAnchor(clothId ,14,-1,-1)
    # p.createSoftBodyAnchor(clothId ,19,boxId,-1, [-0.5,-0.5,0])
    
    debug = True 
    
    
    if debug:
        data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        print("--------------")
        print("data=",data)
        print(data[0])
        print(data[1])
        
        # show vertice locations 
        text_uid = []
        for i in range(data[0]):
            pos = data[1][i]
            uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
            text_uid.append(uid)
        
    while p.isConnected():
        
        # use this to show position of cloth vertices
        if debug:
            # args: bodyUniqueId, linkIndex
            # Since vertices with different normals are duplicated, the can be more vertices than in the original mesh.  You can receive the simulation vertices by using flags
            data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
            for i in range(data[0]):
                pos = data[1][i]
                uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1], replaceItemUniqueId=text_uid[i])
                
        p.stepSimulation()
        time.sleep(1/60)