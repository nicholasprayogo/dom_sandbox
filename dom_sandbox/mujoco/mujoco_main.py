import mujoco
import mujoco_viewer
# need to install pyyaml too!
import argparse
import random 

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--xml_path', type=str, required=False, default = "data/cloth_panda/cloth_panda.xml")
    parser.add_argument('--control_mode', type=str, required=False, default= "freefall")
    
    args = parser.parse_args()
    
    control_mode = args.control_mode
    xml_path = args.xml_path

    # TODO how to load urdf
    # xml_path = "dom_sandbox/dom_sandbox/pybullet/data/ur_description/urdf/ur10_robot.urdf"
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # create the viewer object
    viewer = mujoco_viewer.MujocoViewer(model, data)

    magnitude = 2
    
    # 167 + 9 
    print(data.qpos.shape)
    
    # simulate and render
    for _ in range(10000):
        if viewer.is_alive:
            
            mujoco.mj_step(model, data)
            viewer.render()
            
            # TODO explore control later
            if control_mode == "force_actuator":
                action = random.choice([magnitude,-magnitude])
                # can use named access
                # actuator_id = model.actuator('root').id
                actuator_id = model.actuator('elbow').id
                data.ctrl[actuator_id] = action

            elif control_mode == "force_to_cloth":
                # action = random.choice([magnitude,-magnitude])

                # external force
                data.xfrc_applied[15] = magnitude
                # named access
                # model.body('B3_5')
                
        else:
            break

    # close
    viewer.close()
