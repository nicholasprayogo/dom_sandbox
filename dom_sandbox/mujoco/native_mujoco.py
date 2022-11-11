import mujoco as mj
import sys
import random
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--xml_path', type=str, required=False, default = "cloth_gripper.xml")
    parser.add_argument('--control_mode', type=str, required=False, default= "freefall")
    
    args = parser.parse_args()
    
    control_mode = args.control_mode
    xml_path = args.xml_path

    # The mujoco.GLContext class only provides basic offscreen rendering capability
    #ctx = mj.GLContext(max_width, max_height)
    #ctx.make_current()

    # xml_path = "cloth_z_up.xml"
    # with open(xml_path, 'r') as file:
    #     xml_string = file.read()

    # cam.type = mj.mjtCamera.mjCAMERA_FIXED
    # https://github.com/deepmind/dm_control/blob/644b3c6844cafd611cb2dfe82bc68bb6aed97292/dm_control/mujoco/engine.py#L689
    #TODO can do somehting similar to https://github.com/deepmind/dm_control/blob/644b3c6844cafd611cb2dfe82bc68bb6aed97292/dm_control/mujoco/engine.py#L603

    cam = mj.MjvCamera()
    opt = mj.MjvOption()

    # MuJoCo itself expects users to set up a working OpenGL context before calling any of its mjr_ rendering routine
    # Once the context is created, it must be made current before MuJoCo rendering functions can be called, which you can do so via ctx.make_current(). Note that a context can only be made current on one thread at any given time, and all subsequent rendering calls must be made on the same thread.

    # creds to https://github.com/deepmind/mujoco/issues/194#issuecomment-1072377617
    # based on https://mujoco.readthedocs.io/en/latest/programming.html#visualizationhttps://mujoco.readthedocs.io/en/latest/programming.html#visualizationhttps
    # might be useful to use this one https://github.com/rohanpsingh/mujoco-python-viewer
    # possible using qtpy? https://github.com/deepmind/mujoco/discussions/214

    # init GLFW, create window, make OpenGL context current, request v-sync
    mj.glfw.glfw.init()

    height, width = int(1200/2), int(900/2)

    window = mj.glfw.glfw.create_window(height, width, "Demo", None, None)
    mj.glfw.glfw.make_context_current(window)
    mj.glfw.glfw.swap_interval(1)

    # initialize visualization data structures
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)

    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)
    # print(data.named.xfrc_applied[0])
    # print(dir(data))

    # print(data.ctrl)
    # outputs 4 since have 4 actuators, must use index (or obtain index from naming primitve below)

    # print(data.xfrc_applied.shape)
    # (91, 6) for  cloth_gripper
    # (82, 6) for cloth

    # sys.exit()

    # with OpenGL
    # create scene and context

    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
    
#     typedef enum _mjtFontScale
# {
#     mjFONTSCALE_50      = 50,       // 50% scale, suitable for low-res rendering
#     mjFONTSCALE_100     = 100,      // normal scale, suitable in the absence of DPI scaling
#     mjFONTSCALE_150     = 150,      // 150% scale
#     mjFONTSCALE_200     = 200,      // 200% scale
#     mjFONTSCALE_250     = 250,      // 250% scale
#     mjFONTSCALE_300     = 300       // 300% scale
# } mjtFontScale;

    # How is mjrContext related to an OpenGL context?
    # An OpenGL context is what enables the application to talk to the video driver and send rendering commands.
    # It must exist and must be current in the calling thread before mjr_makeContext is called.
    # GLFW and related libraries provide the necessary functions as shown above.

    # more details here: https://mujoco.readthedocs.io/en/latest/programming.html#context-and-gpu-resources

    magnitude = 2

    while not mj.glfw.glfw.window_should_close(window):
        simstart = data.time

        while (data.time - simstart < 1.0/60.0):
            mj.mj_step(model, data)

        #viewport = mj.MjrRect(0, 0, 0, 0)
        #mj.glfw.glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, height, width)

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

            # can also use force 
            # model.body('B3_5')

    # Example output for body
    #         >>> model.body('B3_5')
    # <_MjModelBodyViews
    #   dofadr: array([0], dtype=int32)
    #   dofnum: array([6], dtype=int32)
    #   geomadr: array([2], dtype=int32)
    #   geomnum: array([1], dtype=int32)
    #   id: 1
    #   inertia: array([5.89107527e-06, 5.89107527e-06, 1.17809725e-05])
    #   invweight0: array([7.66231540e+00, 4.08681601e+04])
    #   ipos: array([0., 0., 0.])
    #   iquat: array([1., 0., 0., 0.])
    #   jntadr: array([0], dtype=int32)
    #   jntnum: array([1], dtype=int32)
    #   mass: array([0.02827433])
    #   mocapid: array([-1], dtype=int32)
    #   name: 'B3_5'
    #   parentid: array([0], dtype=int32)
    #   pos: array([0., 0., 1.])
    #   quat: array([1., 0., 0., 0.])
    #   rootid: array([1], dtype=int32)
    #   sameframe: array([1], dtype=uint8)
    #   simple: array([0], dtype=uint8)
    #   subtreemass: array([2.29022104])
    #   user: array([], dtype=float64)
    #   weldid: array([1], dtype=int32)
    # >

        elif control_mode == "freefall":
            pass
        
        #mj.mjv_updateScene(model, data, opt, None, cam, 0, scene)
        
        # update scene with newest data/model state
        # void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                    #  const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn);
                    
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        
        # render the scene
        mj.mjr_render(viewport, scene, context)
        
        # standard procedure for OpenGL
        # swap OpenGL buffers (blocking call due to v-sync)
        mj.glfw.glfw.swap_buffers(window)
        # process pending GUI events, call GLFW callbacks
        mj.glfw.glfw.poll_events()

    # close GLFW, free visualization storage
    mj.glfw.glfw.terminate()
