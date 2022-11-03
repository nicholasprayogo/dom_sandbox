# from dm_control


# Load a model from an MJCF XML string.
# xml_string = """
# <mujoco>
#   <worldbody>
#     <light name="top" pos="0 0 1.5"/>
#     <geom name="floor" type="plane" size="1 1 .1"/>
#     <body name="box" pos="0 0 .3">
#       <joint name="up_down" type="slide" axis="0 0 1"/>
#       <geom name="box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
#       <geom name="sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
#     </body>
#   </worldbody>
# </mujoco>
# """

# library = "mujoco_native"
# dm_control
# mujoco python bindings
import matplotlib.pyplot as plt

import PIL.Image
#@title Other imports and helper functions

#@title All `dm_control` imports required for this tutorial

# The basic mujoco wrapper.
from dm_control import mujoco

# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

# PyMJCF
from dm_control import mjcf

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control.composer import variation

# Imports for Composer tutorial example
from dm_control.composer.variation import distributions
from dm_control.composer.variation import noises
from dm_control.locomotion.arenas import floors

# Control Suite
from dm_control import suite

# Run through corridor example
from dm_control.locomotion.walkers import cmu_humanoid
from dm_control.locomotion.arenas import corridors as corridor_arenas
from dm_control.locomotion.tasks import corridors as corridor_tasks

# Soccer
from dm_control.locomotion import soccer

# Manipulation
from dm_control import manipulation

# General
import copy
import os
import itertools
from IPython.display import clear_output
import numpy as np
# Graphics-related
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from IPython.display import HTML
import PIL.Image
# Internal loading of video libraries.

# Use svg backend for figure rendering
# %config InlineBackend.figure_format = 'svg'

# Font sizes
SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12
plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

def display_video(frames, framerate=30, path = "animation.mp4" ):
    height, width, _ = frames[0].shape
    dpi = 70
    orig_backend = matplotlib.get_backend()
    matplotlib.use('Agg')  # Switch to headless 'Agg' to inhibit figure rendering.
    fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
    matplotlib.use(orig_backend)  # Switch back to the original backend.
    ax.set_axis_off()
    ax.set_aspect('equal')
    ax.set_position([0, 0, 1, 1])
    im = ax.imshow(frames[0])
    def update(frame):
      im.set_data(frame)
      return [im]
    interval = 1000/framerate
    anim = animation.FuncAnimation(fig=fig, func=update, frames=frames,
                                   interval=interval, blit=True, repeat=False)

    f = path

    writervideo = animation.FFMpegWriter(fps=60)
    anim.save(f, writer=writervideo)

    # return HTML(anim.to_html5_video())

# Seed numpy's global RNG so that cell outputs are deterministic. We also try to
# use RandomState instances that are local to a single cell wherever possible.
np.random.seed(42)

library = "mujoco_native"

if library == "mujoco_native":
    import mujoco
    import mujoco as mj
    xml_path = "cloth.xml"
    # with open(xml_path, 'r') as file:
    #     xml_string = file.read()

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # physics = mujoco.Physics.from_xml_string(xml_string)
    # Render the default camera view as a numpy array of pixels.
    # pixels = model.render()

    # https://mujoco.readthedocs.io/en/latest/python.html#rendering
    # max_width = 100
    # max_height = 100

    # MuJoCo itself expects users to set up a working OpenGL context before calling any of its mjr_ rendering routine
    # Once the context is created, it must be made current before MuJoCo rendering functions can be called, which you can do so via ctx.make_current(). Note that a context can only be made current on one thread at any given time, and all subsequent rendering calls must be made on the same thread.
    # ctx = mujoco.GLContext(max_width, max_height)
    # ctx.make_current()

    try:
        # context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        # mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
        #
        # scene = mujoco.MjvScene(model, maxgeom=1000)
        # # https://mujoco.readthedocs.io/en/latest/APIreference.html#mjrrect
        #
        # while data.time < 1:
        #     rect = mujoco._render.MjrRect(0,0, max_width, max_height)
        #
        #     mujoco.mj_step(model, data)
        #     print(data.geom_xpos)
        #
        #     mujoco.mjv_updateScene(
        #         model, data, mujoco.MjvOption(), mujoco.MjvPerturb(),
        #         mujoco.MjvCamera(), mujoco.mjtCatBit.mjCAT_ALL, scene
        #         )
        #
        #     mujoco.mjr_render(rect, scene, context)

        # ////////// separated

        # creds to https://github.com/deepmind/mujoco/issues/194#issuecomment-1072377617
        # based on https://mujoco.readthedocs.io/en/latest/programming.html#visualizationhttps://mujoco.readthedocs.io/en/latest/programming.html#visualizationhttps
        # might be useful to use this one https://github.com/rohanpsingh/mujoco-python-viewer
        # possible using qtpy? https://github.com/deepmind/mujoco/discussions/214

        max_width = 100
        max_height = 100

        # The mujoco.GLContext class only provides basic offscreen rendering capability

        #ctx = mj.GLContext(max_width, max_height)
        #ctx.make_current()
        xml_path = "cloth.xml"
        # with open(xml_path, 'r') as file:
        #     xml_string = file.read()

        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        cam = mj.MjvCamera()
        opt = mj.MjvOption()

        mj.glfw.glfw.init()
        window = mj.glfw.glfw.create_window(1200, 900, "Demo", None, None)
        mj.glfw.glfw.make_context_current(window)
        mj.glfw.glfw.swap_interval(1)

        mj.mjv_defaultCamera(cam)
        mj.mjv_defaultOption(opt)

        model = mj.MjModel.from_xml_path(xml_path)
        data = mj.MjData(model)
        print(data.ctrl["elbow"])
        scene = mj.MjvScene(model, maxgeom=10000)
        context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

        while not mj.glfw.glfw.window_should_close(window):
            simstart = data.time

            while (data.time - simstart < 1.0/60.0):
                mj.mj_step(model, data)

            #viewport = mj.MjrRect(0, 0, 0, 0)
            #mj.glfw.glfw.get_framebuffer_size(window)
            viewport = mj.MjrRect(0, 0, 1200, 900)

            #mj.mjv_updateScene(model, data, opt, None, cam, 0, scene)
            mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)

            mj.glfw.glfw.swap_buffers(window)
            mj.glfw.glfw.poll_events()

        # while physics.data.time < duration:
        #     # physics.named.data.xfrc_applied[0, :3] = dist * 20
        #     # with physics.reset_context():]
        #
        #     # apply force on actuator
        #     if mode == "force_actuator":
        #         action = random.choice([magnitude,-magnitude])
        #         physics.named.data.ctrl["elbow"] = action
        #     # physics.named.data.ctrl["shoulder"] = 5
        #
        #     # apply force directly to body
        #
        #     if mode == "force_to_cloth":
        #         physics.named.data.xfrc_applied["B3_5", 0] = magnitude
        #
        #     elif mode ==  "freefall":
        #         pass
        #
        #     physics.step()

        mj.glfw.glfw.terminate()

    except Exception as e:
        print(e)
        # ctx.free()
        # context.free()
        # The context is freed automatically when the ctx object is deleted, but in some multi-threaded scenario it may be necessary to explicitly free the underlying OpenGL context. To do so, call ctx.free(), after which point it is the user’s responsibility to ensure that no further rendering calls are made on the context.

elif library == "dm_control":
    from dm_control import mujoco

    xml_path = "cloth_gripper.xml"
    # xml_path = "asset/panda_sim/franka_panda.xml"
    with open(xml_path, 'r') as file:
        xml_string = file.read()

    physics = mujoco.Physics.from_xml_string(xml_string)

    duration = 5    # (seconds)
    framerate = 30  # (Hz)

    # Visualize the joint axis
    scene_option = mujoco.wrapper.core.MjvOption()
    scene_option.flags[enums.mjtVisFlag.mjVIS_JOINT] = True

    # Simulate and display video.
    frames = []
    physics.reset()  # Reset state and time

    # Control signal frequency, phase, amplitude.
    # freq = 5
    # phase = 2 * np.pi * random_state.rand(len(actuators))
    # amp = 0.9

    # https://github.com/deepmind/dm_control/blob/main/dm_control/mujoco/tutorial.ipynb
    # constant actuator signal

    import random

    # mode = "force_to_cloth"
    mode = "force_actuator_conjug_random"
    # mode = "freefall"
    magnitude = 2

    while physics.data.time < duration:
        # physics.named.data.xfrc_applied[0, :3] = dist * 20
        # with physics.reset_context():]

        # apply force on actuator
        if mode == "force_actuator":
            action = random.choice([magnitude,-magnitude])
            physics.named.data.ctrl["elbow"] = action
        # physics.named.data.ctrl["shoulder"] = 5

        # apply force directly to body

        if mode == "force_to_cloth":
            physics.named.data.xfrc_applied["B3_5", 0] = magnitude

        elif mode ==  "freefall":
            pass

        physics.step()
        if len(frames) < physics.data.time * framerate:
            pixels = physics.render(scene_option=scene_option)
            frames.append(pixels)

    display_video(frames, framerate, path=f"{mode}_{magnitude}.mp4")
    # while physics.time() < 1.:
    #     physics.step()
    #     # Render the default camera view as a numpy array of pixels.
    #     pixels = physics.render()
    #     im = PIL.Image.fromarray(pixels)
    #     plt.imshow(im)
    #     plt.show()

# while physics.time() < 1.:
#   physics.step()

# Reset the simulation, move the slide joint upwards and recompute derived
# # quantities (e.g. the positions of the body and geoms).
# with physics.reset_context():
#   physics.named.data.qpos['up_down'] = 0.5
#
# # Print the positions of the geoms.
# print(physics.named.data.geom_xpos)
# # FieldIndexer(geom_xpos):
# #            x         y         z
# # 0  floor [ 0         0         0       ]
# # 1    box [ 0         0         0.8     ]
# # 2 sphere [ 0.2       0.2       1       ]
#
# # Advance the simulation for 1 second.

#
# # Print the new z-positions of the 'box' and 'sphere' geoms.
# print(physics.named.data.geom_xpos[['box', 'sphere'], 'z'])
